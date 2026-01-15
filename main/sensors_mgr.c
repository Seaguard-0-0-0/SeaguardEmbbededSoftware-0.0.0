#include "sensors_mgr.h"
#include "sensor_data.h"
#include "wifi_mgr.h"
#include "mqtt_mgr.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_clk.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_rom_sys.h"
#include "soc/soc_caps.h"

#include "onewire_bus.h"
#include "ds18b20.h"
#include "dht.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#if SOC_TEMP_SENSOR_SUPPORTED
#include "driver/temperature_sensor.h"
#endif

// HC-SR04 pins.
#define ULTRASONIC_TRIG_GPIO    GPIO_NUM_25
#define ULTRASONIC_ECHO_GPIO    GPIO_NUM_26

// I2C for MPU6050.
#define I2C_PORT            I2C_NUM_0
#define I2C_SDA_GPIO        GPIO_NUM_21
#define I2C_SCL_GPIO        GPIO_NUM_22
#define I2C_FREQ_HZ         400000

// DS18B20 and DHT11 pins.
#define DS18B20_GPIO        GPIO_NUM_4
#define DHT11_GPIO          GPIO_NUM_5

// Optional ADC for battery voltage.
#define USE_ADC_VBAT        0
#if USE_ADC_VBAT
#define VBAT_ADC_CHANNEL    ADC_CHANNEL_6
#define VBAT_ADC_ATTEN      ADC_ATTEN_DB_11
#define VBAT_DIVIDER_RATIO  2.0f
#endif

// Timing (milliseconds)
#define DS18B20_READ_INTERVAL   2000
#define DHT11_READ_INTERVAL     2500
#define ULTRASONIC_TASK_MS      300
#define MPU_TASK_MS             150
#define SYSTEM_TASK_MS          2000

// Ultrasonic timing (microseconds)
#define ULTRASONIC_TIMEOUT_US   30000
#define ULTRASONIC_TRIGGER_US   10

#define TASK_STACK_SIZE         4096

// MPU6050 registers
#define MPU6050_ADDR            0x68
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_ACCEL_XOUT  0x3B
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C

static const char *TAG = "SENSORS_MGR";

static TaskHandle_t s_ds18b20_task;
static TaskHandle_t s_dht11_task;
static TaskHandle_t s_ultrasonic_task;
static TaskHandle_t s_mpu_task;
static TaskHandle_t s_system_task;
static bool s_started;

static onewire_bus_handle_t s_onewire_bus;
static ds18b20_device_handle_t s_ds18b20_device;
static bool s_mpu_ready;
static int64_t s_mpu_last_init_ms;

#if SOC_TEMP_SENSOR_SUPPORTED
static temperature_sensor_handle_t s_temp_handle;
static bool s_temp_ready;
#endif

#if USE_ADC_VBAT
static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali;
static bool s_adc_ready;
static bool s_adc_cali_ready;
#endif

static char s_mac_str[18];
static int s_chip_model;
static int s_chip_revision;
static int s_chip_cores;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static esp_err_t init_ds18b20(void)
{
    if (!s_onewire_bus) {
        onewire_bus_config_t bus_config = {
            .bus_gpio_num = DS18B20_GPIO,
        };
        onewire_bus_rmt_config_t rmt_config = {
            .max_rx_bytes = 10,
        };
        esp_err_t ret = onewire_new_bus_rmt(&bus_config, &rmt_config, &s_onewire_bus);
        if (ret != ESP_OK) {
            s_onewire_bus = NULL;
            return ret;
        }
    }

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_device;
    esp_err_t ret = onewire_new_device_iter(s_onewire_bus, &iter);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = onewire_device_iter_get_next(iter, &next_device);
    if (ret == ESP_OK) {
        s_ds18b20_device = NULL;
        ds18b20_config_t ds_cfg = {};
        ret = ds18b20_new_device(&next_device, &ds_cfg, &s_ds18b20_device);
        if (ret == ESP_OK) {
            (void)ds18b20_set_resolution(s_ds18b20_device, DS18B20_RESOLUTION_12B);
        }
    } else {
        s_ds18b20_device = NULL;
        ret = ESP_FAIL;
    }

    onewire_del_device_iter(iter);
    return ret;
}

static void ds18b20_task(void *pvParameters)
{
    (void)pvParameters;
    bool sensor_ready = (s_ds18b20_device != NULL);

    for (;;) {
        if (!sensor_ready) {
            if (init_ds18b20() == ESP_OK) {
                sensor_ready = true;
                ESP_LOGI(TAG, "DS18B20 ready");
            } else {
                sensor_data_set_ds18b20(0.0f, false);
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
        }

        esp_err_t err = ds18b20_trigger_temperature_conversion(s_ds18b20_device);
        if (err == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(800));
            float temperature = 0.0f;
            err = ds18b20_get_temperature(s_ds18b20_device, &temperature);
            if (err == ESP_OK) {
                sensor_data_set_ds18b20(temperature, true);
            } else {
                sensor_data_set_ds18b20(0.0f, false);
                sensor_ready = false;
            }
        } else {
            sensor_data_set_ds18b20(0.0f, false);
            sensor_ready = false;
        }

        vTaskDelay(pdMS_TO_TICKS(DS18B20_READ_INTERVAL));
    }
}

static void dht11_task(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        float temperature = 0.0f;
        float humidity = 0.0f;
        esp_err_t err = dht_read_float_data(DHT_TYPE_DHT11, DHT11_GPIO,
                                            &humidity, &temperature);
        if (err == ESP_OK) {
            sensor_data_set_dht11(temperature, humidity, true);
        } else {
            sensor_data_set_dht11(0.0f, 0.0f, false);
        }
        vTaskDelay(pdMS_TO_TICKS(DHT11_READ_INTERVAL));
    }
}

static void ultrasonic_init(void)
{
    gpio_config_t trig = {
        .pin_bit_mask = 1ULL << ULTRASONIC_TRIG_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t echo = {
        .pin_bit_mask = 1ULL << ULTRASONIC_ECHO_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig);
    gpio_config(&echo);
    gpio_set_level(ULTRASONIC_TRIG_GPIO, 0);
}

static bool ultrasonic_read_cm(float *out_cm)
{
    if (!out_cm) {
        return false;
    }

    gpio_set_level(ULTRASONIC_TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level(ULTRASONIC_TRIG_GPIO, 1);
    esp_rom_delay_us(ULTRASONIC_TRIGGER_US);
    gpio_set_level(ULTRASONIC_TRIG_GPIO, 0);

    int64_t start_wait = esp_timer_get_time();
    while (gpio_get_level(ULTRASONIC_ECHO_GPIO) == 0) {
        if (esp_timer_get_time() - start_wait > ULTRASONIC_TIMEOUT_US) {
            return false;
        }
        esp_rom_delay_us(2);
    }

    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(ULTRASONIC_ECHO_GPIO) == 1) {
        if (esp_timer_get_time() - echo_start > ULTRASONIC_TIMEOUT_US) {
            return false;
        }
        esp_rom_delay_us(2);
    }

    int64_t echo_end = esp_timer_get_time();
    int64_t pulse_us = echo_end - echo_start;
    *out_cm = (float)pulse_us * 0.0343f / 2.0f;
    return true;
}

static void ultrasonic_task(void *arg)
{
    (void)arg;
    const TickType_t delay = pdMS_TO_TICKS(ULTRASONIC_TASK_MS);

    for (;;) {
        float cm = 0.0f;
        bool ok = ultrasonic_read_cm(&cm);
        sensor_data_set_ultrasonic(cm, ok);
        vTaskDelay(delay);
    }
}

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

static esp_err_t mpu6050_write(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_write_to_device(I2C_PORT, MPU6050_ADDR,
                                      data, sizeof(data), pdMS_TO_TICKS(100));
}

static esp_err_t mpu6050_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, MPU6050_ADDR,
                                        &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static bool mpu6050_init(void)
{
    if (mpu6050_write(MPU6050_REG_PWR_MGMT_1, 0x00) != ESP_OK) {
        return false;
    }
    (void)mpu6050_write(MPU6050_REG_GYRO_CONFIG, 0x00);
    (void)mpu6050_write(MPU6050_REG_ACCEL_CONFIG, 0x00);
    vTaskDelay(pdMS_TO_TICKS(20));
    return true;
}

static bool mpu6050_read_sample(float *ax, float *ay, float *az,
                                float *gx, float *gy, float *gz, float *temp_c)
{
    uint8_t raw[14] = {0};
    if (mpu6050_read(MPU6050_REG_ACCEL_XOUT, raw, sizeof(raw)) != ESP_OK) {
        return false;
    }

    int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t temp_raw = (int16_t)((raw[6] << 8) | raw[7]);
    int16_t gx_raw = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

    *ax = (float)ax_raw / 16384.0f;
    *ay = (float)ay_raw / 16384.0f;
    *az = (float)az_raw / 16384.0f;

    *gx = (float)gx_raw / 131.0f;
    *gy = (float)gy_raw / 131.0f;
    *gz = (float)gz_raw / 131.0f;

    *temp_c = ((float)temp_raw / 340.0f) + 36.53f;
    return true;
}

static void mpu6050_task(void *arg)
{
    (void)arg;
    const TickType_t delay = pdMS_TO_TICKS(MPU_TASK_MS);

    for (;;) {
        int64_t now = now_ms();
        if (!s_mpu_ready && (now - s_mpu_last_init_ms) > 2000) {
            s_mpu_ready = mpu6050_init();
            s_mpu_last_init_ms = now;
        }

        bool ok = false;
        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        float gx = 0.0f, gy = 0.0f, gz = 0.0f;
        float temp_c = 0.0f;

        if (s_mpu_ready) {
            ok = mpu6050_read_sample(&ax, &ay, &az, &gx, &gy, &gz, &temp_c);
            if (!ok) {
                s_mpu_ready = false;
            }
        }

        sensor_data_set_mpu(ax, ay, az, gx, gy, gz, temp_c, ok);
        vTaskDelay(delay);
    }
}

#if SOC_TEMP_SENSOR_SUPPORTED
static void cpu_temp_init(void)
{
    temperature_sensor_config_t temp_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    if (temperature_sensor_install(&temp_cfg, &s_temp_handle) == ESP_OK) {
        if (temperature_sensor_enable(s_temp_handle) == ESP_OK) {
            s_temp_ready = true;
        }
    }
}
#endif

static float read_cpu_temp(void)
{
#if SOC_TEMP_SENSOR_SUPPORTED
    if (!s_temp_ready) {
        return -1000.0f;
    }
    float temp_c = 0.0f;
    if (temperature_sensor_get_celsius(s_temp_handle, &temp_c) != ESP_OK) {
        return -1000.0f;
    }
    return temp_c;
#else
    return -1000.0f;
#endif
}

#if USE_ADC_VBAT
static void adc_vbat_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&init_cfg, &s_adc_handle) == ESP_OK) {
        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = VBAT_ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_oneshot_config_channel(s_adc_handle, VBAT_ADC_CHANNEL, &chan_cfg) == ESP_OK) {
            s_adc_ready = true;
        }
    }

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = VBAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc_cali) == ESP_OK) {
        s_adc_cali_ready = true;
    }
}

static float read_vbat(void)
{
    if (!s_adc_ready) {
        return -1.0f;
    }

    int raw = 0;
    if (adc_oneshot_read(s_adc_handle, VBAT_ADC_CHANNEL, &raw) != ESP_OK) {
        return -1.0f;
    }

    int mv = 0;
    if (s_adc_cali_ready && adc_cali_raw_to_voltage(s_adc_cali, raw, &mv) == ESP_OK) {
        return (mv / 1000.0f) * VBAT_DIVIDER_RATIO;
    }

    return -1.0f;
}
#else
static float read_vbat(void)
{
    return -1.0f;
}
#endif

static void system_task(void *arg)
{
    (void)arg;
    const TickType_t delay = pdMS_TO_TICKS(SYSTEM_TASK_MS);

    for (;;) {
        sensor_system_t sys = {0};
        sys.wifi_connected = wifi_is_connected();
        sys.mqtt_connected = mqtt_is_connected();
        sys.wifi_rssi = wifi_get_rssi();
        sys.wifi_channel = wifi_get_channel();
        wifi_get_ssid(sys.ssid, sizeof(sys.ssid));
        wifi_get_ip(sys.ip, sizeof(sys.ip));

        sys.uptime_ms = (uint32_t)now_ms();
        sys.free_heap = esp_get_free_heap_size();
        sys.min_free_heap = esp_get_minimum_free_heap_size();
        sys.cpu_hz = esp_clk_cpu_freq();
        sys.reset_reason = (int)esp_reset_reason();
        sys.vbat = read_vbat();
        sys.cpu_temp = read_cpu_temp();

        sys.chip_model = s_chip_model;
        sys.chip_revision = s_chip_revision;
        sys.chip_cores = s_chip_cores;
        strlcpy(sys.mac, s_mac_str, sizeof(sys.mac));

        sensor_data_set_system(&sys);
        vTaskDelay(delay);
    }
}

void sensors_init(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    s_chip_model = (int)chip_info.model;
    s_chip_revision = (int)chip_info.revision;
    s_chip_cores = (int)chip_info.cores;

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_mac_str, sizeof(s_mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    sensor_data_set_chip_info(s_chip_model, s_chip_revision, s_chip_cores);
    sensor_data_set_mac(s_mac_str);

    esp_err_t i2c_err = i2c_master_init();
    if (i2c_err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(i2c_err));
    } else {
        s_mpu_ready = mpu6050_init();
        s_mpu_last_init_ms = now_ms();
    }

    ultrasonic_init();

#if SOC_TEMP_SENSOR_SUPPORTED
    cpu_temp_init();
#endif

#if USE_ADC_VBAT
    adc_vbat_init();
#endif
}

void sensors_start(void)
{
    if (s_started) {
        return;
    }

    xTaskCreate(ds18b20_task, "ds18b20_task", TASK_STACK_SIZE, NULL, 5, &s_ds18b20_task);
    xTaskCreate(dht11_task, "dht11_task", TASK_STACK_SIZE, NULL, 5, &s_dht11_task);
    xTaskCreate(ultrasonic_task, "ultrasonic_task", TASK_STACK_SIZE, NULL, 5, &s_ultrasonic_task);
    xTaskCreate(mpu6050_task, "mpu6050_task", TASK_STACK_SIZE, NULL, 5, &s_mpu_task);
    xTaskCreate(system_task, "system_task", TASK_STACK_SIZE, NULL, 5, &s_system_task);

    s_started = true;
    ESP_LOGI(TAG, "Sensor tasks started");
}

void sensors_stop(void)
{
    if (s_ds18b20_task) {
        vTaskDelete(s_ds18b20_task);
        s_ds18b20_task = NULL;
    }
    if (s_dht11_task) {
        vTaskDelete(s_dht11_task);
        s_dht11_task = NULL;
    }
    if (s_ultrasonic_task) {
        vTaskDelete(s_ultrasonic_task);
        s_ultrasonic_task = NULL;
    }
    if (s_mpu_task) {
        vTaskDelete(s_mpu_task);
        s_mpu_task = NULL;
    }
    if (s_system_task) {
        vTaskDelete(s_system_task);
        s_system_task = NULL;
    }
    s_started = false;
    ESP_LOGI(TAG, "Sensor tasks stopped");
}
