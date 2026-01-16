#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "cJSON.h"

#include "config.h"
#include "wifi_mgr.h"
#include "mqtt_mgr.h"
#include "sensors_mgr.h"
#include "sensor_data.h"
#include "rc_input.h"
#include "motor_out.h"

#define TOPIC_MAX_LEN 128

static const char *TAG = "APP_MAIN";

static char s_topic_sensors[TOPIC_MAX_LEN];
static char s_topic_ack[TOPIC_MAX_LEN];
static char s_topic_control_prefix[TOPIC_MAX_LEN];
static char s_topic_rc[TOPIC_MAX_LEN];

#define RC_PUBLISH_INTERVAL_MS  200
#define RC_PUBLISH_TIMEOUT_MS   200
#define RC_PUBLISH_LOG_MS       1000

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static bool build_topic(char *out, size_t out_len, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(out, out_len, fmt, args);
    va_end(args);
    return (written >= 0 && (size_t)written < out_len);
}

static void init_topics(void)
{
    bool ok = true;
    ok &= build_topic(s_topic_sensors, sizeof(s_topic_sensors),
                      "seaguard/%s/sensors", BOAT_ID);
    ok &= build_topic(s_topic_ack, sizeof(s_topic_ack),
                      "seaguard/%s/ack", BOAT_ID);
    ok &= build_topic(s_topic_control_prefix, sizeof(s_topic_control_prefix),
                      "seaguard/%s/control/", BOAT_ID);
    ok &= build_topic(s_topic_rc, sizeof(s_topic_rc),
                      "seaguard/%s/rc", BOAT_ID);

    if (!ok) {
        ESP_LOGE(TAG, "Topic build failed");
    }
}

static void mqtt_publish_json(const char *topic, cJSON *json)
{
    if (!topic || !json) {
        return;
    }
    char *payload = cJSON_PrintUnformatted(json);
    if (!payload) {
        ESP_LOGW(TAG, "JSON encode failed");
        return;
    }
    if (!mqtt_publish(topic, payload)) {
        ESP_LOGW(TAG, "Publish failed");
    }
    cJSON_free(payload);
}

static void motor_hw_forward(const char *speed)
{
    ESP_LOGI(TAG, "Motor forward speed=%s", speed ? speed : "none");
    // Add PWM/ESC code here.
}

static void motor_hw_back(const char *speed)
{
    ESP_LOGI(TAG, "Motor back speed=%s", speed ? speed : "none");
    // Add PWM/ESC code here.
}

static void motor_hw_left(const char *speed)
{
    ESP_LOGI(TAG, "Motor left speed=%s", speed ? speed : "none");
    // Add PWM/ESC code here.
}

static void motor_hw_right(const char *speed)
{
    ESP_LOGI(TAG, "Motor right speed=%s", speed ? speed : "none");
    // Add PWM/ESC code here.
}

static void motor_hw_stop(void)
{
    ESP_LOGI(TAG, "Motor stop");
    // Add PWM/ESC code here.
}

static const char *payload_speed_string(cJSON *payload)
{
    if (!payload) {
        return NULL;
    }
    if (cJSON_IsString(payload) && payload->valuestring) {
        return payload->valuestring;
    }
    if (cJSON_IsObject(payload)) {
        cJSON *speed = cJSON_GetObjectItemCaseSensitive(payload, "speed");
        if (cJSON_IsString(speed) && speed->valuestring) {
            return speed->valuestring;
        }
    }
    return NULL;
}

static bool motor_apply_action(const char *action, cJSON *payload,
                               char *reason_out, size_t reason_len)
{
    if (!action) {
        if (reason_out && reason_len > 0) {
            strlcpy(reason_out, "missing action", reason_len);
        }
        return false;
    }

    const char *speed = payload_speed_string(payload);

    if (strcmp(action, "forward") == 0) {
        motor_hw_forward(speed);
        return true;
    }
    if (strcmp(action, "back") == 0) {
        motor_hw_back(speed);
        return true;
    }
    if (strcmp(action, "left") == 0) {
        motor_hw_left(speed);
        return true;
    }
    if (strcmp(action, "right") == 0) {
        motor_hw_right(speed);
        return true;
    }
    if (strcmp(action, "stop") == 0) {
        motor_hw_stop();
        return true;
    }

    if (reason_out && reason_len > 0) {
        strlcpy(reason_out, "unknown action", reason_len);
    }
    return false;
}

static void mqtt_send_ack(const char *cmd_id, const char *action, bool ok, const char *reason)
{
    if (!cmd_id || !action) {
        return;
    }

    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return;
    }

    cJSON_AddStringToObject(root, "cmdId", cmd_id);
    cJSON_AddStringToObject(root, "action", action);
    cJSON_AddBoolToObject(root, "ok", ok);
    if (!ok && reason && reason[0] != '\0') {
        cJSON_AddStringToObject(root, "reason", reason);
    }
    cJSON_AddNumberToObject(root, "timestamp", (double)now_ms());

    mqtt_publish_json(s_topic_ack, root);
    cJSON_Delete(root);
}

static bool rc_channel_recent(const rc_input_t *rc, size_t index, int64_t now_ms)
{
    if (!rc || index >= (sizeof(rc->ch_us) / sizeof(rc->ch_us[0]))) {
        return false;
    }
    if (!rc->valid[index]) {
        return false;
    }
    if (rc->last_update_ms[index] <= 0) {
        return false;
    }
    return (now_ms - rc->last_update_ms[index]) <= RC_PUBLISH_TIMEOUT_MS;
}

static void publish_rc_task(void *arg)
{
    (void)arg;
    const TickType_t delay = pdMS_TO_TICKS(RC_PUBLISH_INTERVAL_MS);
    int64_t last_log_ms = 0;

    for (;;) {
        if (!mqtt_is_connected()) {
            vTaskDelay(delay);
            continue;
        }

        rc_input_t rc = {0};
        rc_input_get_snapshot(&rc);

        int64_t now = now_ms();
        bool ch1_ok = rc_channel_recent(&rc, 0, now);
        bool ch2_ok = rc_channel_recent(&rc, 1, now);
        bool ch5_ok = rc_channel_recent(&rc, 4, now);

        if (!(ch1_ok && ch2_ok && ch5_ok)) {
            vTaskDelay(delay);
            continue;
        }

        cJSON *root = cJSON_CreateObject();
        if (!root) {
            vTaskDelay(delay);
            continue;
        }

        cJSON_AddNumberToObject(root, "ch1", rc.ch_us[0]);
        cJSON_AddNumberToObject(root, "ch2", rc.ch_us[1]);
        cJSON_AddNumberToObject(root, "ch5", rc.ch_us[4]);
        cJSON_AddNumberToObject(root, "timestamp", (double)now);

        mqtt_publish_json(s_topic_rc, root);
        cJSON_Delete(root);

        if (now - last_log_ms >= RC_PUBLISH_LOG_MS) {
            last_log_ms = now;
            ESP_LOGI(TAG, "RC publish ch1=%u ch2=%u ch5=%u topic=%s",
                     rc.ch_us[0], rc.ch_us[1], rc.ch_us[4], s_topic_rc);
        }

        vTaskDelay(delay);
    }
}

static const char *topic_action_from_control(const char *topic)
{
    if (!topic || s_topic_control_prefix[0] == '\0') {
        return NULL;
    }
    size_t prefix_len = strlen(s_topic_control_prefix);
    if (strncmp(topic, s_topic_control_prefix, prefix_len) != 0) {
        return NULL;
    }
    const char *last_slash = strrchr(topic, '/');
    if (!last_slash || *(last_slash + 1) == '\0') {
        return NULL;
    }
    return last_slash + 1;
}

static void control_callback(const char *topic, const char *json_payload)
{
    if (!topic || !json_payload) {
        return;
    }

    const char *topic_action = topic_action_from_control(topic);
    cJSON *root = cJSON_Parse(json_payload);
    if (!root) {
        ESP_LOGW(TAG, "Bad control JSON");
        return;
    }

    cJSON *cmd_id_item = cJSON_GetObjectItemCaseSensitive(root, "cmdId");
    if (!cJSON_IsString(cmd_id_item) || !cmd_id_item->valuestring) {
        ESP_LOGW(TAG, "Missing cmdId, ignore");
        cJSON_Delete(root);
        return;
    }
    const char *cmd_id = cmd_id_item->valuestring;

    cJSON *action_item = cJSON_GetObjectItemCaseSensitive(root, "action");
    const char *json_action = (cJSON_IsString(action_item) && action_item->valuestring)
        ? action_item->valuestring : NULL;

    if (topic_action && json_action && strcmp(topic_action, json_action) != 0) {
        mqtt_send_ack(cmd_id, topic_action, false, "action mismatch");
        cJSON_Delete(root);
        return;
    }

    const char *action = topic_action ? topic_action : json_action;
    if (!action) {
        mqtt_send_ack(cmd_id, "unknown", false, "missing action");
        cJSON_Delete(root);
        return;
    }

    cJSON *payload_item = cJSON_GetObjectItemCaseSensitive(root, "payload");
    char reason[64] = {0};
    bool ok = motor_apply_action(action, payload_item, reason, sizeof(reason));
    mqtt_send_ack(cmd_id, action, ok, ok ? NULL : reason);
    cJSON_Delete(root);
}

static void publish_task(void *arg)
{
    (void)arg;
    const TickType_t delay = pdMS_TO_TICKS(1000);

    for (;;) {
        if (!mqtt_is_connected()) {
            vTaskDelay(delay);
            continue;
        }

        sensor_data_set_timestamp(now_ms());
        sensor_data_t snapshot = {0};
        sensor_data_get_copy(&snapshot);

        cJSON *root = cJSON_CreateObject();
        if (!root) {
            vTaskDelay(delay);
            continue;
        }

        cJSON_AddNumberToObject(root, "timestamp", (double)snapshot.timestamp_ms);

        cJSON *ds18b20 = cJSON_AddObjectToObject(root, "ds18b20");
        if (ds18b20) {
            cJSON_AddNumberToObject(ds18b20, "temp_c", snapshot.ds18b20.temp);
            cJSON_AddBoolToObject(ds18b20, "valid", snapshot.ds18b20.valid);
        }

        cJSON *dht11 = cJSON_AddObjectToObject(root, "dht11");
        if (dht11) {
            cJSON_AddNumberToObject(dht11, "temp_c", snapshot.dht11.temp);
            cJSON_AddNumberToObject(dht11, "humidity", snapshot.dht11.humidity);
            cJSON_AddBoolToObject(dht11, "valid", snapshot.dht11.valid);
        }

        cJSON *ultra = cJSON_AddObjectToObject(root, "ultrasonic");
        if (ultra) {
            cJSON_AddNumberToObject(ultra, "cm", snapshot.ultrasonic.ultrasonic_cm);
            cJSON_AddBoolToObject(ultra, "ok", snapshot.ultrasonic.ultrasonic_ok);
        }

        cJSON *mpu = cJSON_AddObjectToObject(root, "mpu6050");
        if (mpu) {
            cJSON_AddNumberToObject(mpu, "ax", snapshot.mpu6050.ax);
            cJSON_AddNumberToObject(mpu, "ay", snapshot.mpu6050.ay);
            cJSON_AddNumberToObject(mpu, "az", snapshot.mpu6050.az);
            cJSON_AddNumberToObject(mpu, "gx", snapshot.mpu6050.gx);
            cJSON_AddNumberToObject(mpu, "gy", snapshot.mpu6050.gy);
            cJSON_AddNumberToObject(mpu, "gz", snapshot.mpu6050.gz);
            cJSON_AddNumberToObject(mpu, "temp", snapshot.mpu6050.temp_c);
            cJSON_AddBoolToObject(mpu, "ok", snapshot.mpu6050.ok);
        }

        cJSON *sys = cJSON_AddObjectToObject(root, "system");
        if (sys) {
            cJSON_AddBoolToObject(sys, "wifi_connected", snapshot.system.wifi_connected);
            cJSON_AddBoolToObject(sys, "mqtt_connected", snapshot.system.mqtt_connected);
            cJSON_AddNumberToObject(sys, "rssi", snapshot.system.wifi_rssi);
            cJSON_AddStringToObject(sys, "ssid", snapshot.system.ssid);
            cJSON_AddStringToObject(sys, "ip", snapshot.system.ip);
            cJSON_AddStringToObject(sys, "mac", snapshot.system.mac);
            cJSON_AddNumberToObject(sys, "uptime_ms", snapshot.system.uptime_ms);
            cJSON_AddNumberToObject(sys, "free_heap", snapshot.system.free_heap);
            cJSON_AddNumberToObject(sys, "min_free_heap", snapshot.system.min_free_heap);
            cJSON_AddNumberToObject(sys, "cpu_hz", snapshot.system.cpu_hz);
            cJSON_AddNumberToObject(sys, "reset_reason", snapshot.system.reset_reason);
            cJSON_AddNumberToObject(sys, "wifi_channel", snapshot.system.wifi_channel);
            cJSON_AddNumberToObject(sys, "vbat", snapshot.system.vbat);
            cJSON_AddNumberToObject(sys, "cpu_temp", snapshot.system.cpu_temp);
            cJSON_AddNumberToObject(sys, "chip_model", snapshot.system.chip_model);
            cJSON_AddNumberToObject(sys, "chip_revision", snapshot.system.chip_revision);
            cJSON_AddNumberToObject(sys, "chip_cores", snapshot.system.chip_cores);
        }

        mqtt_publish_json(s_topic_sensors, root);
        cJSON_Delete(root);
        vTaskDelay(delay);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    sensor_data_init();
    init_topics();

    wifi_init_sta();
    while (!wifi_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    mqtt_set_control_callback(control_callback);
    mqtt_init_start(MQTT_BROKER_URI, BOAT_ID);

    sensors_init();
    sensors_start();

    motor_out_init();
    rc_input_init();
    rc_input_start();
    motor_out_start();

    xTaskCreate(publish_task, "publish_task", 4096, NULL, 5, NULL);
    xTaskCreate(publish_rc_task, "publish_rc_task", 3072, NULL, 5, NULL);
}
