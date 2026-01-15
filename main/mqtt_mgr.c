#include "mqtt_mgr.h"

#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"

#define TOPIC_MAX_LEN 128

static const char *TAG = "MQTT_MGR";

static esp_mqtt_client_handle_t s_mqtt_client;
static bool s_connected;
static mqtt_control_cb_t s_control_cb;
static char s_control_prefix[TOPIC_MAX_LEN];
static char s_control_sub[TOPIC_MAX_LEN];

static bool topic_is_control(const char *topic)
{
    if (!topic || s_control_prefix[0] == '\0') {
        return false;
    }
    size_t prefix_len = strlen(s_control_prefix);
    if (strncmp(topic, s_control_prefix, prefix_len) != 0) {
        return false;
    }
    return true;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED: {
            s_connected = true;
            int msg_id = esp_mqtt_client_subscribe(s_mqtt_client, s_control_sub, 0);
            ESP_LOGI(TAG, "MQTT connected, subscribed %s msg_id=%d", s_control_sub, msg_id);
            break;
        }
        case MQTT_EVENT_DISCONNECTED:
            s_connected = false;
            ESP_LOGW(TAG, "MQTT disconnected");
            break;
        case MQTT_EVENT_DATA: {
            if (event->topic_len <= 0 || event->data_len <= 0) {
                break;
            }

            char *topic = calloc(1, event->topic_len + 1);
            char *data = calloc(1, event->data_len + 1);
            if (!topic || !data) {
                free(topic);
                free(data);
                break;
            }

            memcpy(topic, event->topic, event->topic_len);
            memcpy(data, event->data, event->data_len);

            if (topic_is_control(topic)) {
                cJSON *root = cJSON_ParseWithLength(data, event->data_len);
                if (!root) {
                    ESP_LOGW(TAG, "Bad control JSON");
                } else if (s_control_cb) {
                    s_control_cb(topic, data);
                    cJSON_Delete(root);
                } else {
                    cJSON_Delete(root);
                }
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

void mqtt_set_control_callback(mqtt_control_cb_t cb)
{
    s_control_cb = cb;
}

void mqtt_init_start(const char *broker_uri, const char *boat_id)
{
    if (!broker_uri || !boat_id) {
        ESP_LOGE(TAG, "MQTT init missing broker or boat id");
        return;
    }
    if (s_mqtt_client) {
        return;
    }

    int written = snprintf(s_control_prefix, sizeof(s_control_prefix),
                           "seaguard/%s/control/", boat_id);
    if (written < 0 || written >= (int)sizeof(s_control_prefix)) {
        ESP_LOGE(TAG, "Control prefix too long");
        return;
    }
    written = snprintf(s_control_sub, sizeof(s_control_sub), "%s#", s_control_prefix);
    if (written < 0 || written >= (int)sizeof(s_control_sub)) {
        ESP_LOGE(TAG, "Control sub too long");
        return;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri,
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_mqtt_client) {
        ESP_LOGE(TAG, "MQTT init failed");
        return;
    }

    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
}

bool mqtt_is_connected(void)
{
    return s_connected;
}

bool mqtt_publish(const char *topic, const char *payload)
{
    if (!topic || !payload || !s_mqtt_client || !s_connected) {
        return false;
    }
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, 0, 0);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "Publish failed");
        return false;
    }
    return true;
}
