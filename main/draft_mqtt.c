#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "mqtt_client.h"



#define MQTT_BROKER_URI "mqtt://YOUR_SERVER_IP:1883"
// Example: "mqtt://192.168.1.50:1883"

// Topics (match your Node server)
#define TOPIC_SENSORS   "seaguard/sensors"
#define TOPIC_CONTROL_SUB "seaguard/control/#"   // receive any control under this tree
// ==========================

static esp_mqtt_client_handle_t mqtt_client = NULL;


// ----------------------
// Handle incoming command
// ----------------------
static void handle_control_command(const char *topic, const char *payload)
{
    ESP_LOGI(TAG, "CONTROL RX | topic=%s payload=%s", topic, payload);

    // Example: decide based on topic/payload
    // Node API might publish payload like "SLOW" to topic "seaguard/control/forward"
    // so you can parse last topic segment:
    const char *last_slash = strrchr(topic, '/');
    const char *cmd_topic = (last_slash) ? last_slash + 1 : topic;

    ESP_LOGI(TAG, "Parsed cmd_topic=%s payload=%s", cmd_topic, payload);

    // TODO: Replace with your motor/servo logic
    // Example:
    // if (strcmp(cmd_topic, "forward") == 0) { motor_forward(payload); }
    // if (strcmp(cmd_topic, "stop") == 0) { motor_stop(); }

    // Quick demo:
    if (strstr(topic, "forward")) {
        ESP_LOGI(TAG, "==> Forward command, speed=%s", payload);
    } else if (strstr(topic, "back")) {
        ESP_LOGI(TAG, "==> Back command, speed=%s", payload);
    } else if (strstr(topic, "stop")) {
        ESP_LOGI(TAG, "==> Stop command");
    } else {
        ESP_LOGI(TAG, "==> Unknown control topic");
    }
}

// ----------------------
// MQTT event handler
// ----------------------
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            // Subscribe to control topic
            esp_mqtt_client_subscribe(mqtt_client, TOPIC_CONTROL_SUB, 0);
            ESP_LOGI(TAG, "Subscribed to: %s", TOPIC_CONTROL_SUB);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT subscribed, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_DATA: {
            // event->topic and event->data are NOT null-terminated
            char topic[event->topic_len + 1];
            char data[event->data_len + 1];

            memcpy(topic, event->topic, event->topic_len);
            topic[event->topic_len] = '\0';

            memcpy(data, event->data, event->data_len);
            data[event->data_len] = '\0';

            ESP_LOGI(TAG, "MQTT RX | topic=%s data=%s", topic, data);

            // Handle control commands (any under seaguard/control/)
            if (strncmp(topic, "seaguard/control/", strlen("seaguard/control/")) == 0) {
                handle_control_command(topic, data);
            }
            break;
        }

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error");
            break;

        default:
            break;
    }
}

// ----------------------
// MQTT start
// ----------------------
static void mqtt_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        // Optional client id:
        // .credentials.client_id = "seaguard-esp32-1",
        // Optional username/password if you add auth on broker:
        // .credentials.username = "user",
        // .credentials.authentication.password = "pass",
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

// ----------------------
// Publish sensor JSON loop
// ----------------------
static void sensor_publish_task(void *pv)
{
    // Wait for Wi-Fi first
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    while (1) {
        // Example dummy sensor values (replace with real sensors)
        float temp = 25.2f;
        float humidity = 60.0f;
        float battery = 12.4f;
        float lat = 23.1f;
        float lon = 58.4f;

        long long ts_ms = (long long)esp_timer_get_time() / 1000LL;

        // Build JSON payload
        char payload[256];
        snprintf(payload, sizeof(payload),
                 "{"
                 "\"temp\":%.2f,"
                 "\"humidity\":%.2f,"
                 "\"battery\":%.2f,"
                 "\"gps\":{\"lat\":%.6f,\"lon\":%.6f},"
                 "\"timestamp\":%lld"
                 "}",
                 temp, humidity, battery, lat, lon, ts_ms);

        if (mqtt_client) {
            int msg_id = esp_mqtt_client_publish(mqtt_client, TOPIC_SENSORS, payload, 0, 0, 0);
            ESP_LOGI(TAG, "Published sensors msg_id=%d payload=%s", msg_id, payload);
        } else {
            ESP_LOGW(TAG, "MQTT not ready yet");
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // every 2 seconds
    }
}

// ----------------------
// app_main
// ----------------------
/*void app_main(void)
{

    mqtt_start();

    xTaskCreate(sensor_publish_task, "sensor_publish_task", 4096, NULL, 5, NULL);
}
*/