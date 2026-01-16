#include "rc_input.h"
#include "config.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"

#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#define RC_INPUT_TASK_STACK     4096
#define RC_INPUT_TASK_PRIO      6
#define RC_EVENT_QUEUE_LEN      24

#define RC_RMT_RESOLUTION_HZ    1000000
#define RC_RMT_MEM_BLOCKS       64

#define RC_DEBUG_LOG_MS         500
#define RC_RMT_LOG_MS           200
#define RC_SIGNAL_TIMEOUT_MS    200

#define RC_IDLE_THRESHOLD_US    5000
#define RC_FILTER_MIN_US        300
#define RC_MIN_VALID_US         900
#define RC_MAX_VALID_US         2100
#define RC_NEUTRAL_US           1500

#define RC_SIGNAL_MIN_NS        1000
#define RC_SIGNAL_MAX_NS        30000000

typedef struct {
    uint8_t ch_index;
    uint8_t slot;
    uint16_t pulse_us;
    uint16_t dur0;
    uint16_t dur1;
    uint8_t level0;
    uint8_t level1;
    uint8_t num_symbols;
    int64_t timestamp_ms;
    bool valid;
} rc_event_t;

typedef struct {
    uint8_t ch_index;
    uint8_t slot;
    gpio_num_t gpio;
    rmt_channel_handle_t rmt_chan;
    rmt_symbol_word_t symbols[RC_RMT_MEM_BLOCKS];
} rc_channel_t;

typedef struct {
    uint8_t ch_index;
    gpio_num_t gpio;
} rc_gpio_channel_t;

typedef enum {
    RC_SRC_NONE = 0,
    RC_SRC_RMT = 1,
    RC_SRC_GPIO = 2,
} rc_source_t;

static const char *TAG = "RC_INPUT";

rc_input_t g_rc;

static rc_channel_t s_channels[] = {
    { .ch_index = 0, .slot = 0, .gpio = RC_CH1_GPIO },
    { .ch_index = 1, .slot = 1, .gpio = RC_CH2_GPIO },
    { .ch_index = 4, .slot = 2, .gpio = RC_CH5_GPIO },
};

static rc_gpio_channel_t s_gpio_channels[] = {
    { .ch_index = 0, .gpio = RC_CH1_GPIO },
    { .ch_index = 1, .gpio = RC_CH2_GPIO },
    { .ch_index = 4, .gpio = RC_CH5_GPIO },
};

static QueueHandle_t s_rc_queue;
static TaskHandle_t s_rc_task;
static portMUX_TYPE s_rc_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_initialized;
static bool s_gpio_initialized;

static volatile int64_t s_gpio_last_rise_us[8];
static volatile uint16_t s_gpio_last_pulse_us[8];
static volatile bool s_gpio_last_level[8];
static volatile rc_source_t s_last_source[8];

static const rmt_receive_config_t s_rx_config = {
    .signal_range_min_ns = RC_SIGNAL_MIN_NS,
    .signal_range_max_ns = RC_SIGNAL_MAX_NS,
};

static const char *rc_source_str(rc_source_t src)
{
    switch (src) {
        case RC_SRC_RMT:
            return "rmt";
        case RC_SRC_GPIO:
            return "gpio";
        default:
            return "none";
    }
}

static void rc_store_sample_from_isr(uint8_t ch_index, uint16_t pulse_us, int64_t now_ms,
                                     rc_source_t source)
{
    if (ch_index >= (uint8_t)(sizeof(g_rc.ch_us) / sizeof(g_rc.ch_us[0]))) {
        return;
    }
    portENTER_CRITICAL_ISR(&s_rc_lock);
    g_rc.valid[ch_index] = true;
    g_rc.ch_us[ch_index] = pulse_us;
    g_rc.last_update_ms[ch_index] = now_ms;
    s_last_source[ch_index] = source;
    portEXIT_CRITICAL_ISR(&s_rc_lock);
}

static void rc_store_sample_from_task(uint8_t ch_index, uint16_t pulse_us, int64_t now_ms,
                                      rc_source_t source)
{
    if (ch_index >= (uint8_t)(sizeof(g_rc.ch_us) / sizeof(g_rc.ch_us[0]))) {
        return;
    }
    portENTER_CRITICAL(&s_rc_lock);
    g_rc.valid[ch_index] = true;
    g_rc.ch_us[ch_index] = pulse_us;
    g_rc.last_update_ms[ch_index] = now_ms;
    s_last_source[ch_index] = source;
    portEXIT_CRITICAL(&s_rc_lock);
}
static bool rc_parse_pulse_us(const rmt_symbol_word_t *symbols, size_t num_symbols,
                              uint16_t *out_pulse_us)
{
    if (!symbols || !out_pulse_us) {
        return false;
    }

    for (size_t i = 0; i < num_symbols; i++) {
        const rmt_symbol_word_t sym = symbols[i];
        if (sym.level0) {
            uint32_t dur0 = sym.duration0;
            if (dur0 >= RC_MIN_VALID_US && dur0 <= RC_MAX_VALID_US) {
                *out_pulse_us = (uint16_t)dur0;
                return true;
            }
        }
        if (sym.level1) {
            uint32_t dur1 = sym.duration1;
            if (dur1 >= RC_MIN_VALID_US && dur1 <= RC_MAX_VALID_US) {
                *out_pulse_us = (uint16_t)dur1;
                return true;
            }
        }
    }

    return false;
}

static bool IRAM_ATTR rc_rx_done_callback(rmt_channel_handle_t channel,
                                          const rmt_rx_done_event_data_t *edata,
                                          void *user_data)
{
    (void)channel;

    rc_channel_t *ch = (rc_channel_t *)user_data;
    if (!ch || !edata) {
        return false;
    }

    uint16_t pulse_us = 0;
    bool valid = rc_parse_pulse_us(edata->received_symbols, edata->num_symbols, &pulse_us);

    rc_event_t evt = {
        .ch_index = ch->ch_index,
        .slot = ch->slot,
        .pulse_us = pulse_us,
        .dur0 = 0,
        .dur1 = 0,
        .level0 = 0,
        .level1 = 0,
        .num_symbols = (uint8_t)(edata->num_symbols > 255 ? 255 : edata->num_symbols),
        .valid = valid,
        .timestamp_ms = (int64_t)xTaskGetTickCountFromISR() * portTICK_PERIOD_MS,
    };

    if (edata->num_symbols > 0 && edata->received_symbols) {
        rmt_symbol_word_t sym = edata->received_symbols[0];
        evt.dur0 = (uint16_t)sym.duration0;
        evt.dur1 = (uint16_t)sym.duration1;
        evt.level0 = sym.level0 ? 1 : 0;
        evt.level1 = sym.level1 ? 1 : 0;
    }

    BaseType_t hp_task_woken = pdFALSE;
    if (s_rc_queue) {
        xQueueSendFromISR(s_rc_queue, &evt, &hp_task_woken);
    }

    (void)rmt_receive(ch->rmt_chan, ch->symbols, sizeof(ch->symbols), &s_rx_config);

    return hp_task_woken == pdTRUE;
}

static void IRAM_ATTR rc_gpio_isr(void *arg)
{
    rc_gpio_channel_t *ch = (rc_gpio_channel_t *)arg;
    if (!ch) {
        return;
    }

    int level = gpio_get_level(ch->gpio);
    int64_t now_us = esp_timer_get_time();
    uint8_t ch_index = ch->ch_index;

    if (level) {
        s_gpio_last_rise_us[ch_index] = now_us;
        s_gpio_last_level[ch_index] = true;
        return;
    }

    if (!s_gpio_last_level[ch_index]) {
        return;
    }

    int64_t rise_us = s_gpio_last_rise_us[ch_index];
    s_gpio_last_level[ch_index] = false;

    if (rise_us <= 0 || now_us <= rise_us) {
        return;
    }

    int64_t pulse_us = now_us - rise_us;
    if (pulse_us < RC_FILTER_MIN_US || pulse_us > (RC_SIGNAL_MAX_NS / 1000)) {
        return;
    }

    if (pulse_us >= RC_MIN_VALID_US && pulse_us <= RC_MAX_VALID_US) {
        s_gpio_last_pulse_us[ch_index] = (uint16_t)pulse_us;
        rc_store_sample_from_isr(ch_index, (uint16_t)pulse_us, now_us / 1000, RC_SRC_GPIO);
    }
}

static void rc_input_task(void *arg)
{
    (void)arg;
    int64_t last_snapshot_log_ms = 0;
    int64_t last_rmt_log_ms = 0;
    bool last_valid_state[8] = {false};

    for (;;) {
        rc_event_t evt;
        bool got_evt = (xQueueReceive(s_rc_queue, &evt, pdMS_TO_TICKS(50)) == pdTRUE);
        int64_t now_ms = esp_timer_get_time() / 1000;

        if (got_evt) {
            if (evt.valid) {
                rc_store_sample_from_task(evt.ch_index, evt.pulse_us, now_ms, RC_SRC_RMT);
            }

            if (now_ms - last_rmt_log_ms >= RC_RMT_LOG_MS) {
                last_rmt_log_ms = now_ms;
                ESP_LOGI(TAG,
                         "RMT ch=%u sym0 l0=%u d0=%u l1=%u d1=%u num=%u pulse=%u valid=%d",
                         evt.ch_index, evt.level0, evt.dur0, evt.level1, evt.dur1,
                         evt.num_symbols, evt.pulse_us, evt.valid ? 1 : 0);
            }
        }

        if (now_ms - last_snapshot_log_ms >= RC_DEBUG_LOG_MS) {
            rc_input_t snapshot = {0};
            rc_source_t source_snapshot[8] = {RC_SRC_NONE};

            portENTER_CRITICAL(&s_rc_lock);
            snapshot = g_rc;
            for (size_t i = 0; i < sizeof(source_snapshot) / sizeof(source_snapshot[0]); i++) {
                source_snapshot[i] = s_last_source[i];
            }
            portEXIT_CRITICAL(&s_rc_lock);

            uint8_t ch_indices[] = {0, 1, 4};
            for (size_t i = 0; i < sizeof(ch_indices) / sizeof(ch_indices[0]); i++) {
                uint8_t ch = ch_indices[i];
                int64_t age_ms = (snapshot.last_update_ms[ch] > 0)
                    ? (now_ms - snapshot.last_update_ms[ch])
                    : -1;
                bool valid_now = snapshot.valid[ch] && (age_ms >= 0) && (age_ms <= RC_SIGNAL_TIMEOUT_MS);
                if (valid_now != last_valid_state[ch]) {
                    last_valid_state[ch] = valid_now;
                    ESP_LOGW(TAG, "RC ch%u %s (age=%lldms)", ch + 1,
                             valid_now ? "valid" : "timeout",
                             (long long)age_ms);
                }
            }

            ESP_LOGI(TAG,
                     "RC snapshot ch1=%u(us,%s,age=%lld) ch2=%u(us,%s,age=%lld) ch5=%u(us,%s,age=%lld)",
                     snapshot.ch_us[0], rc_source_str(source_snapshot[0]),
                     (long long)(snapshot.last_update_ms[0] > 0 ? now_ms - snapshot.last_update_ms[0] : -1),
                     snapshot.ch_us[1], rc_source_str(source_snapshot[1]),
                     (long long)(snapshot.last_update_ms[1] > 0 ? now_ms - snapshot.last_update_ms[1] : -1),
                     snapshot.ch_us[4], rc_source_str(source_snapshot[4]),
                     (long long)(snapshot.last_update_ms[4] > 0 ? now_ms - snapshot.last_update_ms[4] : -1));

            ESP_LOGI(TAG,
                     "GPIO raw ch1=%u ch2=%u ch5=%u (idle_thr=%uus)",
                     (unsigned)s_gpio_last_pulse_us[0],
                     (unsigned)s_gpio_last_pulse_us[1],
                     (unsigned)s_gpio_last_pulse_us[4],
                     RC_IDLE_THRESHOLD_US);

            last_snapshot_log_ms = now_ms;
        }
    }
}

void rc_input_get_snapshot(rc_input_t *out)
{
    if (!out) {
        return;
    }

    portENTER_CRITICAL(&s_rc_lock);
    *out = g_rc;
    portEXIT_CRITICAL(&s_rc_lock);
}

static void rc_input_init_defaults(void)
{
    portENTER_CRITICAL(&s_rc_lock);
    for (size_t i = 0; i < sizeof(g_rc.ch_us) / sizeof(g_rc.ch_us[0]); i++) {
        g_rc.ch_us[i] = RC_NEUTRAL_US;
        g_rc.valid[i] = false;
        g_rc.last_update_ms[i] = 0;
        s_last_source[i] = RC_SRC_NONE;
        s_gpio_last_rise_us[i] = 0;
        s_gpio_last_pulse_us[i] = 0;
        s_gpio_last_level[i] = false;
    }
    portEXIT_CRITICAL(&s_rc_lock);
}

static void rc_gpio_init(void)
{
    if (s_gpio_initialized) {
        return;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };

    for (size_t i = 0; i < sizeof(s_gpio_channels) / sizeof(s_gpio_channels[0]); i++) {
        io_conf.pin_bit_mask |= (1ULL << s_gpio_channels[i].gpio);
    }

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "GPIO ISR service install failed: %s", esp_err_to_name(err));
        return;
    }

    for (size_t i = 0; i < sizeof(s_gpio_channels) / sizeof(s_gpio_channels[0]); i++) {
        rc_gpio_channel_t *ch = &s_gpio_channels[i];
        err = gpio_isr_handler_add(ch->gpio, rc_gpio_isr, ch);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "GPIO ISR add failed for GPIO %d: %s",
                     ch->gpio, esp_err_to_name(err));
        }
        if (ch->gpio >= GPIO_NUM_34) {
            ESP_LOGW(TAG, "GPIO %d has no internal pull-ups; ensure external drive.", ch->gpio);
        }
    }

    s_gpio_initialized = true;
}

void rc_input_init(void)
{
    if (s_initialized) {
        return;
    }

    s_rc_queue = xQueueCreate(RC_EVENT_QUEUE_LEN, sizeof(rc_event_t));
    if (!s_rc_queue) {
        ESP_LOGE(TAG, "Event queue create failed");
        return;
    }

    rc_input_init_defaults();
    ESP_LOGI(TAG,
             "RMT RX cfg: min_ns=%u max_ns=%u (idle~%uus) mem=%u",
             (unsigned)RC_SIGNAL_MIN_NS,
             (unsigned)RC_SIGNAL_MAX_NS,
             RC_IDLE_THRESHOLD_US,
             RC_RMT_MEM_BLOCKS);

    for (size_t i = 0; i < sizeof(s_channels) / sizeof(s_channels[0]); i++) {
        rc_channel_t *ch = &s_channels[i];
        rmt_rx_channel_config_t rx_cfg = {
            .gpio_num = ch->gpio,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = RC_RMT_RESOLUTION_HZ,
            .mem_block_symbols = RC_RMT_MEM_BLOCKS,
            .flags = {
                .invert_in = false,
                .with_dma = false,
            },
        };

        esp_err_t err = rmt_new_rx_channel(&rx_cfg, &ch->rmt_chan);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "RMT channel init failed for GPIO %d: %s",
                     ch->gpio, esp_err_to_name(err));
            continue;
        }

        rmt_rx_event_callbacks_t cbs = {
            .on_recv_done = rc_rx_done_callback,
        };
        err = rmt_rx_register_event_callbacks(ch->rmt_chan, &cbs, ch);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "RMT callback register failed for GPIO %d: %s",
                     ch->gpio, esp_err_to_name(err));
            continue;
        }

        err = rmt_enable(ch->rmt_chan);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "RMT enable failed for GPIO %d: %s",
                     ch->gpio, esp_err_to_name(err));
            continue;
        }

        err = rmt_receive(ch->rmt_chan, ch->symbols, sizeof(ch->symbols), &s_rx_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "RMT receive start failed for GPIO %d: %s",
                     ch->gpio, esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "RMT receive started on GPIO %d", ch->gpio);
        }
    }

    rc_gpio_init();

    s_initialized = true;
}

void rc_input_start(void)
{
    if (!s_initialized) {
        rc_input_init();
    }

    if (!s_rc_task) {
        xTaskCreate(rc_input_task, "rc_input_task", RC_INPUT_TASK_STACK,
                    NULL, RC_INPUT_TASK_PRIO, &s_rc_task);
    }
}
