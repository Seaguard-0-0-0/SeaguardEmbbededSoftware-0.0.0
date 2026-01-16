#include "motor_out.h"
#include "rc_input.h"
#include "config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#define MOTOR_CONTROL_TASK_STACK    4096
#define MOTOR_CONTROL_TASK_PRIO     5
#define MOTOR_CONTROL_PERIOD_MS     20
#define MOTOR_LOG_PERIOD_MS         500

#define RC_CH1_INDEX                0
#define RC_CH2_INDEX                1
#define RC_CH5_INDEX                4

#define RC_SIGNAL_TIMEOUT_MS        200
#define RC_ENABLE_THRESHOLD_US      1600
#define RC_NEUTRAL_US               1500
#define RC_RANGE_US                 500

#define LEDC_TIMER_RESOLUTION       LEDC_TIMER_16_BIT
#define LEDC_TIMER_FREQ_HZ          50
#define LEDC_SPEED_MODE             LEDC_LOW_SPEED_MODE
#define LEDC_TIMER                  LEDC_TIMER_0
#define LEDC_CHANNEL_LEFT           LEDC_CHANNEL_0
#define LEDC_CHANNEL_RIGHT          LEDC_CHANNEL_1
#define LEDC_MAX_DUTY               ((1U << LEDC_TIMER_RESOLUTION) - 1)

static const char *TAG = "MOTOR_OUT";

volatile bool g_esp_override = false;
static TaskHandle_t s_motor_task;

static uint32_t esc_us_to_duty(uint16_t us)
{
    const uint32_t period_us = 1000000 / LEDC_TIMER_FREQ_HZ;
    return (uint32_t)((uint64_t)us * LEDC_MAX_DUTY / period_us);
}

static uint16_t clamp_us(int32_t us)
{
    if (us < ESC_MIN_US) {
        return ESC_MIN_US;
    }
    if (us > ESC_MAX_US) {
        return ESC_MAX_US;
    }
    return (uint16_t)us;
}

static float clamp_float(float value)
{
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

void motor_out_set_us(uint16_t left_us, uint16_t right_us)
{
    left_us = clamp_us(left_us);
    right_us = clamp_us(right_us);

    uint32_t left_duty = esc_us_to_duty(left_us);
    uint32_t right_duty = esc_us_to_duty(right_us);

    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_LEFT, left_duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_LEFT);

    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_RIGHT, right_duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_RIGHT);
}

void motor_out_stop(void)
{
    motor_out_set_us(ESC_NEUTRAL_US, ESC_NEUTRAL_US);
}

void motor_out_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_SPEED_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_RESOLUTION,
        .freq_hz = LEDC_TIMER_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t left_cfg = {
        .gpio_num = MOTOR_LEFT_GPIO,
        .speed_mode = LEDC_SPEED_MODE,
        .channel = LEDC_CHANNEL_LEFT,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config_t right_cfg = {
        .gpio_num = MOTOR_RIGHT_GPIO,
        .speed_mode = LEDC_SPEED_MODE,
        .channel = LEDC_CHANNEL_RIGHT,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&left_cfg));
    ESP_ERROR_CHECK(ledc_channel_config(&right_cfg));

    motor_out_stop();
}

static bool rc_channel_valid(const rc_input_t *rc, uint8_t index, int64_t now_ms)
{
    if (!rc || index >= (uint8_t)(sizeof(rc->ch_us) / sizeof(rc->ch_us[0]))) {
        return false;
    }
    if (!rc->valid[index]) {
        return false;
    }
    if ((now_ms - rc->last_update_ms[index]) > RC_SIGNAL_TIMEOUT_MS) {
        return false;
    }
    return true;
}

static void motor_control_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_log_ms = 0;

    for (;;) {
        rc_input_t rc = {0};
        rc_input_get_snapshot(&rc);

        int64_t now_ms = esp_timer_get_time() / 1000;
        bool ch1_ok = rc_channel_valid(&rc, RC_CH1_INDEX, now_ms);
        bool ch2_ok = rc_channel_valid(&rc, RC_CH2_INDEX, now_ms);
        bool ch5_ok = rc_channel_valid(&rc, RC_CH5_INDEX, now_ms);
        bool rc_signal_ok = ch1_ok && ch2_ok && ch5_ok;
        bool rc_enabled = rc_signal_ok && (rc.ch_us[RC_CH5_INDEX] > RC_ENABLE_THRESHOLD_US);
        bool allow_rc = rc_enabled && !g_esp_override;

        uint16_t left_us = ESC_NEUTRAL_US;
        uint16_t right_us = ESC_NEUTRAL_US;

        if (allow_rc) {
            float throttle = ((float)rc.ch_us[RC_CH2_INDEX] - RC_NEUTRAL_US) / RC_RANGE_US;
            float steering = ((float)rc.ch_us[RC_CH1_INDEX] - RC_NEUTRAL_US) / RC_RANGE_US;
            throttle = clamp_float(throttle);
            steering = clamp_float(steering);

            float left = clamp_float(throttle + steering);
            float right = clamp_float(throttle - steering);

            left_us = clamp_us((int32_t)(ESC_NEUTRAL_US + left * RC_RANGE_US));
            right_us = clamp_us((int32_t)(ESC_NEUTRAL_US + right * RC_RANGE_US));
            motor_out_set_us(left_us, right_us);
        } else {
            motor_out_stop();
        }

        if (now_ms - last_log_ms >= MOTOR_LOG_PERIOD_MS) {
            last_log_ms = now_ms;
            ESP_LOGI(TAG, "rc_ok=%d enabled=%d override=%d left_us=%u right_us=%u",
                     rc_signal_ok, rc_enabled, g_esp_override ? 1 : 0, left_us, right_us);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
    }
}

void motor_out_start(void)
{
    if (!s_motor_task) {
        xTaskCreate(motor_control_task, "motor_control_task", MOTOR_CONTROL_TASK_STACK,
                    NULL, MOTOR_CONTROL_TASK_PRIO, &s_motor_task);
    }
}
