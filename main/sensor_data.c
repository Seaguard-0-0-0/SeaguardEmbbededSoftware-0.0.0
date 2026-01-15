#include "sensor_data.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "SENSOR_DATA";

sensor_data_t g_sensor_data;

static SemaphoreHandle_t s_data_mutex;

void sensor_data_init(void)
{
    if (s_data_mutex) {
        return;
    }

    s_data_mutex = xSemaphoreCreateMutex();
    if (!s_data_mutex) {
        ESP_LOGE(TAG, "Mutex create failed");
        return;
    }

    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(&g_sensor_data, 0, sizeof(g_sensor_data));
        g_sensor_data.system.vbat = -1.0f;
        g_sensor_data.system.cpu_temp = -1000.0f;
        g_sensor_data.system.wifi_channel = -1;
        strlcpy(g_sensor_data.system.ip, "0.0.0.0", sizeof(g_sensor_data.system.ip));
        xSemaphoreGive(s_data_mutex);
    }
}

void sensor_data_lock(void)
{
    if (!s_data_mutex) {
        return;
    }
    (void)xSemaphoreTake(s_data_mutex, portMAX_DELAY);
}

void sensor_data_unlock(void)
{
    if (!s_data_mutex) {
        return;
    }
    xSemaphoreGive(s_data_mutex);
}

void sensor_data_set_timestamp(int64_t ts_ms)
{
    if (!s_data_mutex) {
        return;
    }
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_sensor_data.timestamp_ms = ts_ms;
        xSemaphoreGive(s_data_mutex);
    }
}

void sensor_data_set_ds18b20(float temp, bool valid)
{
    if (!s_data_mutex) {
        return;
    }
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_sensor_data.ds18b20.temp = temp;
        g_sensor_data.ds18b20.valid = valid;
        xSemaphoreGive(s_data_mutex);
    }
}

void sensor_data_set_dht11(float temp, float humidity, bool valid)
{
    if (!s_data_mutex) {
        return;
    }
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_sensor_data.dht11.temp = temp;
        g_sensor_data.dht11.humidity = humidity;
        g_sensor_data.dht11.valid = valid;
        xSemaphoreGive(s_data_mutex);
    }
}

void sensor_data_set_ultrasonic(float cm, bool ok)
{
    if (!s_data_mutex) {
        return;
    }
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_sensor_data.ultrasonic.ultrasonic_cm = cm;
        g_sensor_data.ultrasonic.ultrasonic_ok = ok;
        xSemaphoreGive(s_data_mutex);
    }
}

void sensor_data_set_mpu(float ax, float ay, float az,
                         float gx, float gy, float gz,
                         float temp_c, bool ok)
{
    if (!s_data_mutex) {
        return;
    }
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_sensor_data.mpu6050.ax = ax;
        g_sensor_data.mpu6050.ay = ay;
        g_sensor_data.mpu6050.az = az;
        g_sensor_data.mpu6050.gx = gx;
        g_sensor_data.mpu6050.gy = gy;
        g_sensor_data.mpu6050.gz = gz;
        g_sensor_data.mpu6050.temp_c = temp_c;
        g_sensor_data.mpu6050.ok = ok;
        xSemaphoreGive(s_data_mutex);
    }
}

void sensor_data_set_system(const sensor_system_t *sys)
{
    if (!sys || !s_data_mutex) {
        return;
    }
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_sensor_data.system = *sys;
        xSemaphoreGive(s_data_mutex);
    }
}

void sensor_data_set_chip_info(int model, int revision, int cores)
{
    if (!s_data_mutex) {
        return;
    }
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_sensor_data.system.chip_model = model;
        g_sensor_data.system.chip_revision = revision;
        g_sensor_data.system.chip_cores = cores;
        xSemaphoreGive(s_data_mutex);
    }
}

void sensor_data_set_mac(const char *mac)
{
    if (!mac || !s_data_mutex) {
        return;
    }
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        strlcpy(g_sensor_data.system.mac, mac, sizeof(g_sensor_data.system.mac));
        xSemaphoreGive(s_data_mutex);
    }
}

void sensor_data_get_copy(sensor_data_t *out)
{
    if (!out || !s_data_mutex) {
        return;
    }
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *out = g_sensor_data;
        xSemaphoreGive(s_data_mutex);
    } else {
        memset(out, 0, sizeof(*out));
    }
}
