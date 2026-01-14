#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "cJSON.h"

// Set Wi-Fi and MQTT here.
#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASS           "YOUR_WIFI_PASS"
#define MQTT_BROKER_URI     "mqtt://192.168.1.10:1883"
#define BOAT_ID             "boat1"

#define WIFI_MAXIMUM_RETRY  5
#define TOPIC_MAX_LEN       128

#define WIFI_CONNECTED_BIT  BIT0
#define MQTT_CONNECTED_BIT  BIT1
#define WIFI_FAIL_BIT       BIT2

#define SENSORS_PUBLISH_MS  2000
#define GPS_PUBLISH_MS      1000
#define STATUS_PUBLISH_MS   5000

static const char *TAG = "APP";

static EventGroupHandle_t s_event_group;
static int s_retry_num;
static esp_mqtt_client_handle_t s_mqtt_client;
static bool s_mqtt_started;

static char s_topic_sensors[TOPIC_MAX_LEN];
static char s_topic_gps[TOPIC_MAX_LEN];
static char s_topic_status[TOPIC_MAX_LEN];
static char s_topic_ack[TOPIC_MAX_LEN];
static char s_topic_control_prefix[TOPIC_MAX_LEN];
static char s_topic_control_sub[TOPIC_MAX_LEN];

typedef struct {
    float temp_c;
    float humidity;
    float battery_v;
    bool valid;
} sensor_sample_t;

typedef struct {
    double lat;
    double lon;
    float speed;
    bool valid;
} gps_fix_t;

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

    if (written < 0 || (size_t)written >= out_len) {
        return false;
    }
    return true;
}

static void init_topics(void)
{
    bool ok = true;
    ok &= build_topic(s_topic_sensors, sizeof(s_topic_sensors), "seaguard/%s/sensors", BOAT_ID);
    ok &= build_topic(s_topic_gps, sizeof(s_topic_gps), "seaguard/%s/gps", BOAT_ID);
    ok &= build_topic(s_topic_status, sizeof(s_topic_status), "seaguard/%s/status", BOAT_ID);
    ok &= build_topic(s_topic_ack, sizeof(s_topic_ack), "seaguard/%s/ack", BOAT_ID);
    ok &= build_topic(s_topic_control_prefix, sizeof(s_topic_control_prefix), "seaguard/%s/control/", BOAT_ID);
    ok &= build_topic(s_topic_control_sub, sizeof(s_topic_control_sub), "%s#", s_topic_control_prefix);

    if (!ok) {
        ESP_LOGE(TAG, "Topic build failed, BOAT_ID too long");
    }
}

static bool mqtt_is_connected(void)
{
    if (!s_mqtt_client || !s_event_group) {
        return false;
    }
    EventBits_t bits = xEventGroupGetBits(s_event_group);
    return (bits & MQTT_CONNECTED_BIT) != 0;
}

static void mqtt_publish_json(const char *topic, cJSON *json)
{
    if (!topic || !json || !mqtt_is_connected()) {
        return;
    }

    char *payload = cJSON_PrintUnformatted(json);
    if (!payload) {
        ESP_LOGW(TAG, "JSON encode failed");
        return;
    }

    (void)esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, 0, 0);
    cJSON_free(payload);
}

static void mqtt_send_ack(const char *cmdId, const char *action, bool ok, const char *reason)
{
    if (!cmdId || !action) {
        return;
    }

    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return;
    }

    cJSON_AddStringToObject(root, "cmdId", cmdId);
    cJSON_AddStringToObject(root, "action", action);
    cJSON_AddBoolToObject(root, "ok", ok);
    if (!ok && reason && reason[0] != '\0') {
        cJSON_AddStringToObject(root, "reason", reason);
    }
    cJSON_AddNumberToObject(root, "timestamp", (double)now_ms());

    mqtt_publish_json(s_topic_ack, root);
    ESP_LOGI(TAG, "ACK sent cmdId=%s ok=%s", cmdId, ok ? "true" : "false");
    cJSON_Delete(root);
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

static void set_reason(char *reasonOut, size_t reasonLen, const char *msg)
{
    if (!reasonOut || reasonLen == 0) {
        return;
    }
    if (!msg) {
        reasonOut[0] = '\0';
        return;
    }
    strlcpy(reasonOut, msg, reasonLen);
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

static bool motor_apply_action(const char *action, cJSON *payload, char *reasonOut, size_t reasonLen)
{
    if (!action) {
        set_reason(reasonOut, reasonLen, "missing action");
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

    set_reason(reasonOut, reasonLen, "unknown action");
    return false;
}

static bool read_sensors(sensor_sample_t *out)
{
    if (!out) {
        return false;
    }
    out->temp_c = 0.0f;
    out->humidity = 0.0f;
    out->battery_v = 0.0f;
    out->valid = false;
    return out->valid;
}

static bool read_gps(gps_fix_t *out)
{
    if (!out) {
        return false;
    }
    out->lat = 0.0;
    out->lon = 0.0;
    out->speed = 0.0f;
    out->valid = false;
    return out->valid;
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

static void handle_control_message(const char *topic, const char *payload, size_t payload_len)
{
    if (!topic || !payload) {
        return;
    }

    const char *topic_action = topic_action_from_control(topic);
    cJSON *root = cJSON_ParseWithLength(payload, payload_len);
    if (!root) {
        ESP_LOGW(TAG, "Bad JSON, ignore");
        return;
    }

    cJSON *cmdId_item = cJSON_GetObjectItemCaseSensitive(root, "cmdId");
    if (!cJSON_IsString(cmdId_item) || !cmdId_item->valuestring) {
        ESP_LOGW(TAG, "Missing cmdId, ignore");
        cJSON_Delete(root);
        return;
    }

    const char *cmdId = cmdId_item->valuestring;

    cJSON *action_item = cJSON_GetObjectItemCaseSensitive(root, "action");
    const char *json_action = (cJSON_IsString(action_item) && action_item->valuestring) ? action_item->valuestring : NULL;

    if (topic_action && json_action && strcmp(topic_action, json_action) != 0) {
        mqtt_send_ack(cmdId, topic_action, false, "action mismatch");
        cJSON_Delete(root);
        return;
    }

    const char *action = topic_action ? topic_action : json_action;
    if (!action) {
        mqtt_send_ack(cmdId, "unknown", false, "missing action");
        cJSON_Delete(root);
        return;
    }

    cJSON *payload_item = cJSON_GetObjectItemCaseSensitive(root, "payload");
    char reason[64] = {0};
    bool ok = motor_apply_action(action, payload_item, reason, sizeof(reason));
    mqtt_send_ack(cmdId, action, ok, ok ? NULL : reason);
    cJSON_Delete(root);
}

static void publish_sensors_task(void *arg)
{
    (void)arg;
    const TickType_t delay = pdMS_TO_TICKS(SENSORS_PUBLISH_MS);

    for (;;) {
        if (!mqtt_is_connected()) {
            vTaskDelay(delay);
            continue;
        }

        sensor_sample_t sample = {0};
        read_sensors(&sample);

        cJSON *root = cJSON_CreateObject();
        if (!root) {
            vTaskDelay(delay);
            continue;
        }

        cJSON_AddNumberToObject(root, "temp_c", sample.temp_c);
        //cJSON_AddNumberToObject(root, "temperature", sample.temp_c);
        cJSON_AddNumberToObject(root, "humidity", sample.humidity);
        cJSON_AddNumberToObject(root, "battery_v", sample.battery_v);
        //cJSON_AddNumberToObject(root, "voltage", sample.battery_v);
        cJSON_AddBoolToObject(root, "valid", sample.valid);
        cJSON_AddNumberToObject(root, "timestamp", (double)now_ms());

        mqtt_publish_json(s_topic_sensors, root);
        ESP_LOGI(TAG, "Sensors sent");

        cJSON_Delete(root);
        vTaskDelay(delay);
    }
}

static void publish_gps_task(void *arg)
{
    (void)arg;
    const TickType_t delay = pdMS_TO_TICKS(GPS_PUBLISH_MS);

    for (;;) {
        if (!mqtt_is_connected()) {
            vTaskDelay(delay);
            continue;
        }

        gps_fix_t fix = {0};
        if (!read_gps(&fix) || !fix.valid) {
            vTaskDelay(delay);
            continue;
        }

        cJSON *root = cJSON_CreateObject();
        if (!root) {
            vTaskDelay(delay);
            continue;
        }

        cJSON_AddNumberToObject(root, "lat", fix.lat);
        cJSON_AddNumberToObject(root, "lon", fix.lon);
        cJSON_AddNumberToObject(root, "speed", fix.speed);
        cJSON_AddNumberToObject(root, "timestamp", (double)now_ms());

        mqtt_publish_json(s_topic_gps, root);
        ESP_LOGI(TAG, "GPS sent");

        cJSON_Delete(root);
        vTaskDelay(delay);
    }
}

static void publish_status_task(void *arg)
{
    (void)arg;
    const TickType_t delay = pdMS_TO_TICKS(STATUS_PUBLISH_MS);

    for (;;) {
        if (!mqtt_is_connected()) {
            vTaskDelay(delay);
            continue;
        }

        int rssi = -127;
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            rssi = ap_info.rssi;
        }

        cJSON *root = cJSON_CreateObject();
        if (!root) {
            vTaskDelay(delay);
            continue;
        }

        cJSON_AddBoolToObject(root, "online", true);
        cJSON_AddNumberToObject(root, "rssi", rssi);
        cJSON_AddNumberToObject(root, "uptime_ms", (double)now_ms());
        cJSON_AddNumberToObject(root, "timestamp", (double)now_ms());

        mqtt_publish_json(s_topic_status, root);
        ESP_LOGI(TAG, "Status sent");

        cJSON_Delete(root);
        vTaskDelay(delay);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED: {
            ESP_LOGI(TAG, "MQTT connected");
            xEventGroupSetBits(s_event_group, MQTT_CONNECTED_BIT);
            int msg_id = esp_mqtt_client_subscribe(s_mqtt_client, s_topic_control_sub, 0);
            ESP_LOGI(TAG, "Subscribed to %s msg_id=%d", s_topic_control_sub, msg_id);
            break;
        }
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            xEventGroupClearBits(s_event_group, MQTT_CONNECTED_BIT);
            break;
        case MQTT_EVENT_DATA: {
            char *topic = NULL;
            char *data = NULL;

            if (event->topic_len > 0) {
                topic = calloc(1, event->topic_len + 1);
            }
            if (event->data_len > 0) {
                data = calloc(1, event->data_len + 1);
            }
            if (!topic || !data) {
                free(topic);
                free(data);
                break;
            }

            memcpy(topic, event->topic, event->topic_len);
            memcpy(data, event->data, event->data_len);

            ESP_LOGI(TAG, "MQTT RX | topic=%s payload=%s", topic, data);

            if (topic_action_from_control(topic)) {
                handle_control_message(topic, data, (size_t)event->data_len);
            }

            free(topic);
            free(data);
            break;
        }
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error");
            break;
        default:
            break;
    }
}

static void mqtt_start(void)
{
    if (s_mqtt_started) {
        return;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_mqtt_client) {
        ESP_LOGE(TAG, "MQTT init failed");
        return;
    }

    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
    s_mqtt_started = true;
}

static void mqtt_start_task(void *arg)
{
    (void)arg;
    xEventGroupWaitBits(s_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    mqtt_start();
    vTaskDelete(NULL);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
        } else {
            xEventGroupSetBits(s_event_group, WIFI_FAIL_BIT);
        }
        xEventGroupClearBits(s_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "Wi-Fi disconnected");
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        xEventGroupSetBits(s_event_group, WIFI_CONNECTED_BIT);
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi got IP " IPSTR, IP2STR(&event->ip_info.ip));
        return;
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    if (strlen(WIFI_PASS) == 0) {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi init done");
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_event_group = xEventGroupCreate();
    if (!s_event_group) {
        ESP_LOGE(TAG, "Event group create failed");
        return;
    }

    init_topics();
    wifi_init_sta();

    xTaskCreate(mqtt_start_task, "mqtt_start_task", 4096, NULL, 5, NULL);
    xTaskCreate(publish_sensors_task, "publish_sensors_task", 4096, NULL, 5, NULL);
    xTaskCreate(publish_gps_task, "publish_gps_task", 4096, NULL, 5, NULL);
    xTaskCreate(publish_status_task, "publish_status_task", 4096, NULL, 5, NULL);
}
