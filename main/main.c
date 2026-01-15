#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_clk.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "soc/soc_caps.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "onewire_bus.h"
#include "ds18b20.h"
#include "dht.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#if SOC_TEMP_SENSOR_SUPPORTED
#include "driver/temperature_sensor.h"
#endif
#include "esp_rom_sys.h"
#include "soc/soc_caps.h"
#include "esp_clk.h"
#include "driver/i2c.h"
#include "esp_err.h"

#if SOC_TEMP_SENSOR_SUPPORTED
#include "driver/temperature_sensor.h"
#endif

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// Set Wi-Fi and MQTT here.
#define WIFI_SSID           "HUAWEI-B2368-759429_NOMAP"
#define WIFI_PASS           "92275499ax8f2002"
#define MQTT_BROKER_URI     "mqtt://192.168.1.54:1883"
#define BOAT_ID             "boat1"

#define WIFI_MAXIMUM_RETRY  5
#define TOPIC_MAX_LEN       128

#define WIFI_CONNECTED_BIT  BIT0
#define MQTT_CONNECTED_BIT  BIT1
#define WIFI_FAIL_BIT       BIT2

#define SENSORS_PUBLISH_MS  2000
#define GPS_PUBLISH_MS      1000
#define STATUS_PUBLISH_MS   5000

#define ULTRASONIC_TIMEOUT_US   30000
#define ULTRASONIC_TRIGGER_US   10

#define ULTRASONIC_TASK_MS  300
#define MPU_TASK_MS         150
#define SYSTEM_TASK_MS      2000
#define MQTT_PUBLISH_MS     1000
// Task Parameters
#define TASK_STACK_SIZE     4096
#define DS18B20_TASK_PRIORITY   5
#define DHT11_TASK_PRIORITY     5
#define MOTOR_TASK_PRIORITY     4
#define WIFI_TASK_PRIORITY      6

// HC-SR04 pins.
#define ULTRASONIC_TRIG_GPIO    GPIO_NUM_25
#define ULTRASONIC_ECHO_GPIO    GPIO_NUM_26

// I2C for MPU6050.
#define I2C_PORT            I2C_NUM_0
#define I2C_SDA_GPIO        GPIO_NUM_21
#define I2C_SCL_GPIO        GPIO_NUM_22
#define I2C_FREQ_HZ         400000

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

#define MPU6050_ADDR            0x68
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_ACCEL_XOUT  0x3B
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C


// DS18B20 Configuration (OneWire Protocol)
#define DS18B20_GPIO        GPIO_NUM_4
#define DS18B20_RESOLUTION  DS18B20_RESOLUTION_12B

#define DHT11_GPIO          GPIO_NUM_5

// Motor Control Pins
#define MOTOR1_PIN          GPIO_NUM_18
#define MOTOR2_PIN          GPIO_NUM_19

// Log Tags
static const char *TAG_DS18B20 = "DS18B20_TASK";
static const char *TAG_DHT11 = "DHT11_TASK";
static const char *TAG_ULTRASONIC = "ULTRASONIC_TASK";
static const char *TAG_MPU6050 = "MPU6050_TASK";
static const char *TAG_MOTOR = "MOTOR_TASK";
static const char *TAG_WIFI = "WIFI_TASK";

static const char *TAG_MAIN = "MAIN"; 

static const char *TAG = "APP";

static char s_topic_sensors[TOPIC_MAX_LEN];
static char s_topic_gps[TOPIC_MAX_LEN];
static char s_topic_status[TOPIC_MAX_LEN];
static char s_topic_ack[TOPIC_MAX_LEN];
static char s_topic_control_prefix[TOPIC_MAX_LEN];
static char s_topic_control_sub[TOPIC_MAX_LEN];


// ============================================================================
// GLOBAL VARIABLES & HANDLES
// ============================================================================


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

// Shared Sensor Data (protected by mutex)
typedef struct {
    struct 
    {
         float temp;
         bool valid;
    } ds18b20;

    struct {
        float temp;
        float humidity;
        bool valid;
    } dht11;

    int64_t timestamp_ms;

    
    struct {
        float ultrasonic_cm;
        bool ultrasonic_ok;
    } ultrasonic;

    struct {
        float ax, ay, az;
        float gx, gy, gz;
        float temp_c;
        bool ok;
    } mpu6050;

    struct {
        int wifi_rssi;
        char ssid[33];
        char ip[16];
        char mac[18];
        uint32_t uptime_ms;
        uint32_t free_heap;
        uint32_t min_free_heap;
        uint32_t cpu_hz;
        int reset_reason;
        int wifi_channel;
        bool wifi_connected;
        bool mqtt_connected;
        float vbat;
        float cpu_temp;
        int chip_model;
        int chip_revision;
        int chip_cores;
    } system;
} sensor_data_t;

static EventGroupHandle_t s_event_group;
static int s_retry_num;
static esp_mqtt_client_handle_t s_mqtt_client;
static bool s_mqtt_started;
static bool s_wifi_connected;
static bool s_mqtt_connected;
static esp_netif_t *s_wifi_netif;
static bool s_mpu_ready;
static int64_t s_mpu_last_init_ms;

static sensor_data_t g_sensor_data = {0};

// Sensor Handles
static onewire_bus_handle_t onewire_bus = NULL;
static ds18b20_device_handle_t ds18b20_device = NULL;

// Synchronization
static SemaphoreHandle_t xSensorMutex = NULL;

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


static void log_heap_status(const char *stage)
{
    uint32_t free_heap = esp_get_free_heap_size();
    size_t free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "[%s] Free heap=%u bytes (8-bit=%u bytes)", stage, (unsigned)free_heap, (unsigned)free_8bit);
}

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

        sensor_data_t snapshot = {0};
        bool snapshot_ok = false;
        if (xSensorMutex && xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            snapshot = g_sensor_data;
            xSemaphoreGive(xSensorMutex);
            snapshot_ok = true;
        }
        if (!snapshot_ok) {
            vTaskDelay(delay);
            continue;
        }
        snapshot.timestamp_ms = now_ms();

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
            s_mqtt_connected = true;
            xEventGroupSetBits(s_event_group, MQTT_CONNECTED_BIT);
            int msg_id = esp_mqtt_client_subscribe(s_mqtt_client, s_topic_control_sub, 0);
            ESP_LOGI(TAG, "Subscribed to %s msg_id=%d", s_topic_control_sub, msg_id);
            break;
        }
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            s_mqtt_connected = false;
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

    if (!s_event_group) {
        ESP_LOGE(TAG_WIFI, "Event group is NULL in wifi_event_handler");
        return;
    }

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
        s_wifi_connected = false;
        xEventGroupClearBits(s_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "Wi-Fi disconnected");
        if (xSensorMutex && xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_sensor_data.system.wifi_connected = false;
            xSemaphoreGive(xSensorMutex);
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        s_wifi_connected = true;
        xEventGroupSetBits(s_event_group, WIFI_CONNECTED_BIT);
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi got IP " IPSTR, IP2STR(&event->ip_info.ip));
        if (xSensorMutex && xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_sensor_data.system.wifi_connected = true;
            snprintf(g_sensor_data.system.ip, sizeof(g_sensor_data.system.ip), IPSTR, IP2STR(&event->ip_info.ip));
            xSemaphoreGive(xSensorMutex);
        }
        return;
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_wifi_netif = esp_netif_create_default_wifi_sta();

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
// ============================================================================
// DS18B20 INITIALIZATION
// ============================================================================
esp_err_t init_ds18b20(void)
{
    ESP_LOGI(TAG_DS18B20, "Initializing DS18B20 sensor...");
    
    // Note: We don't clean up handles on reinit as the bus should persist
    // Only initialize if not already done
    if (onewire_bus == NULL) {
        // Configure OneWire bus
        onewire_bus_config_t bus_config = {
            .bus_gpio_num = DS18B20_GPIO,
        };
        
        onewire_bus_rmt_config_t rmt_config = {
            .max_rx_bytes = 10,
        };
        
        // Create OneWire bus
        esp_err_t ret = onewire_new_bus_rmt(&bus_config, &rmt_config, &onewire_bus);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_DS18B20, "Failed to create OneWire bus: %s", esp_err_to_name(ret));
            onewire_bus = NULL;
            return ret;
        }
    }
    
    esp_err_t ret = ESP_OK;
    
    // Search for DS18B20 devices
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_device;
    
    ret = onewire_new_device_iter(onewire_bus, &iter);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_DS18B20, "Failed to create device iterator: %s", esp_err_to_name(ret));
        return ret;
    }
    
    esp_err_t search_result = onewire_device_iter_get_next(iter, &next_device);
    
    if (search_result == ESP_OK) {
        // Reset device handle if reinitializing
        ds18b20_device = NULL;
        
        ds18b20_config_t ds_cfg = {};
        ret = ds18b20_new_device(&next_device, &ds_cfg, &ds18b20_device);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_DS18B20, "Failed to create DS18B20 device: %s", esp_err_to_name(ret));
            onewire_del_device_iter(iter);
            return ret;
        }
        
        ESP_LOGI(TAG_DS18B20, "Found DS18B20 device, address: %016llX", 
                 next_device.address);
        
        // Set resolution
        ret = ds18b20_set_resolution(ds18b20_device, DS18B20_RESOLUTION);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG_DS18B20, "Failed to set resolution: %s", esp_err_to_name(ret));
            // Continue anyway, default resolution will be used
        }
    } else {
        ESP_LOGE(TAG_DS18B20, "No DS18B20 device found on bus! Check connections:");
        ESP_LOGE(TAG_DS18B20, "  - VCC to 3.3V");
        ESP_LOGE(TAG_DS18B20, "  - GND to GND");
        ESP_LOGE(TAG_DS18B20, "  - DATA to GPIO %d with 4.7kΩ pull-up resistor", DS18B20_GPIO);
        onewire_del_device_iter(iter);
        ds18b20_device = NULL;
        return ESP_FAIL;
    }
    
    onewire_del_device_iter(iter);
    ESP_LOGI(TAG_DS18B20, "DS18B20 initialized successfully");
    
    return ESP_OK;

}


// ============================================================================
// TASK 1: DS18B20 TEMPERATURE READING
// ============================================================================

void ds18b20_task(void *pvParameters)
{
    float temperature;
    bool sensor_available = (ds18b20_device != NULL);
    
    ESP_LOGI(TAG_DS18B20, "DS18B20 task started");
    
    if (!sensor_available) {
        ESP_LOGE(TAG_DS18B20, "DS18B20 device not initialized! Task will retry initialization...");
    }
    
    while (1) {
        // Check if sensor is available
        if (!sensor_available) {
            // Try to reinitialize
            ESP_LOGI(TAG_DS18B20, "Attempting to reinitialize DS18B20...");
            if (init_ds18b20() == ESP_OK) {
                sensor_available = true;
                ESP_LOGI(TAG_DS18B20, "DS18B20 reinitialized successfully");
            } else {
                ESP_LOGW(TAG_DS18B20, "Reinitialization failed, will retry in 5 seconds");
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
        }
        
        // Trigger temperature conversion
        esp_err_t err = ds18b20_trigger_temperature_conversion(ds18b20_device);
        
        if (err == ESP_OK) {
            // Wait for conversion (depends on resolution, 12-bit = 750ms)
            vTaskDelay(pdMS_TO_TICKS(800));
            
            // Read temperature
            err = ds18b20_get_temperature(ds18b20_device, &temperature);
            
            if (err == ESP_OK) {
                // Update shared data with mutex protection
                if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    g_sensor_data.ds18b20.temp = temperature;
                    g_sensor_data.ds18b20.valid = true;
                    xSemaphoreGive(xSensorMutex);
                }
                
                ESP_LOGI(TAG_DS18B20, "Temperature: %.2f °C", temperature);
            } else {
                ESP_LOGW(TAG_DS18B20, "Failed to read temperature: %s", 
                         esp_err_to_name(err));
                
                if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    g_sensor_data.ds18b20.valid = false;
                    xSemaphoreGive(xSensorMutex);
                }
                
                // Sensor may be disconnected
                sensor_available = false;
            }
        } else {
            ESP_LOGW(TAG_DS18B20, "Failed to trigger conversion: %s", 
                     esp_err_to_name(err));
            
            // Mark sensor as unavailable and try reinit next cycle
            sensor_available = false;
            
            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                g_sensor_data.ds18b20.valid = false;
                xSemaphoreGive(xSensorMutex);
            }
        }
        
        // Wait before next reading
        vTaskDelay(pdMS_TO_TICKS(DS18B20_READ_INTERVAL));
    }
}

// ============================================================================
// TASK 2: DHT11 TEMPERATURE & HUMIDITY READING
// ============================================================================

void dht11_task(void *pvParameters)
{
    float temperature;
    float humidity;
    
    ESP_LOGI(TAG_DHT11, "DHT11 task started");
    
    while (1) {
        // Read DHT11 sensor
                
        esp_err_t err = dht_read_float_data(DHT_TYPE_DHT11, DHT11_GPIO, 
                                            &humidity, &temperature);
        
        if (err == ESP_OK) {
            float temp_c = temperature;
            float hum_percent = humidity;
            
            // Update shared data with mutex protection
            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                g_sensor_data.dht11.temp = temp_c;
                g_sensor_data.dht11.humidity = hum_percent;
                g_sensor_data.dht11.valid = true;
                xSemaphoreGive(xSensorMutex);
            }
            
            ESP_LOGI(TAG_DHT11, "Temperature: %.1f °C, Humidity: %.1f %%", 
                     temp_c, hum_percent);
        } else {
            ESP_LOGW(TAG_DHT11, "Failed to read DHT11: %s", esp_err_to_name(err));
            
            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                g_sensor_data.dht11.valid = false;
                xSemaphoreGive(xSensorMutex);
            }
        }
        
        // Wait before next reading (DHT11 requires minimum 2 seconds between reads)
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
    return i2c_master_write_to_device(I2C_PORT, MPU6050_ADDR, data, sizeof(data), pdMS_TO_TICKS(100));
}

static esp_err_t mpu6050_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, MPU6050_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
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

static void ultrasonic_task(void *arg)
{
    (void)arg;
    const TickType_t delay = pdMS_TO_TICKS(ULTRASONIC_TASK_MS);

    for (;;) {
        float cm = 0.0f;
        bool ok = ultrasonic_read_cm(&cm);

        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_sensor_data.ultrasonic.ultrasonic_cm = cm;
            g_sensor_data.ultrasonic.ultrasonic_ok = ok;
            xSemaphoreGive(xSensorMutex);
        }

        if (!ok) {
            ESP_LOGW(TAG, "Ultrasonic timeout");
        }

        vTaskDelay(delay);
    }
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

        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_sensor_data.mpu6050.ax = ax;
            g_sensor_data.mpu6050.ay = ay;
            g_sensor_data.mpu6050.az = az;
            g_sensor_data.mpu6050.gx = gx;
            g_sensor_data.mpu6050.gy = gy;
            g_sensor_data.mpu6050.gz = gz;
            g_sensor_data.mpu6050.temp_c = temp_c;
            g_sensor_data.mpu6050.ok = ok;
            xSemaphoreGive(xSensorMutex);
        }

        if (!ok) {
            ESP_LOGW(TAG, "MPU6050 read failed");
        }

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
        bool wifi_connected = s_wifi_connected;
        bool mqtt_connected = s_mqtt_connected;
        int rssi = -127;
        int channel = -1;
        char ssid[33] = {0};
        char ip[16] = "0.0.0.0";

        if (wifi_connected) {
            wifi_ap_record_t ap_info;
            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                rssi = ap_info.rssi;
                channel = ap_info.primary;
                strlcpy(ssid, (char *)ap_info.ssid, sizeof(ssid));
            }

            if (s_wifi_netif) {
                esp_netif_ip_info_t ip_info;
                if (esp_netif_get_ip_info(s_wifi_netif, &ip_info) == ESP_OK) {
                    snprintf(ip, sizeof(ip), IPSTR, IP2STR(&ip_info.ip));
                }
            }
        }

        uint32_t uptime = (uint32_t)now_ms();
        uint32_t free_heap = esp_get_free_heap_size();
        uint32_t min_free_heap = esp_get_minimum_free_heap_size();
        uint32_t cpu_hz = esp_clk_cpu_freq();
        int reset_reason = (int)esp_reset_reason();
        float vbat = read_vbat();
        float cpu_temp = read_cpu_temp();

        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            g_sensor_data.system.wifi_connected = wifi_connected;
            g_sensor_data.system.mqtt_connected = mqtt_connected;
            g_sensor_data.system.wifi_rssi = rssi;
            g_sensor_data.system.wifi_channel = channel;
            strlcpy(g_sensor_data.system.ssid, ssid, sizeof(g_sensor_data.system.ssid));
            strlcpy(g_sensor_data.system.ip, ip, sizeof(g_sensor_data.system.ip));
            g_sensor_data.system.uptime_ms = uptime;
            g_sensor_data.system.free_heap = free_heap;
            g_sensor_data.system.min_free_heap = min_free_heap;
            g_sensor_data.system.cpu_hz = cpu_hz;
            g_sensor_data.system.reset_reason = reset_reason;
            g_sensor_data.system.vbat = vbat;
            g_sensor_data.system.cpu_temp = cpu_temp;
            xSemaphoreGive(xSensorMutex);
        }

        vTaskDelay(delay);
    }
}


static void init_static_system_info(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_sensor_data.system.chip_model = (int)chip_info.model;
        g_sensor_data.system.chip_revision = (int)chip_info.revision;
        g_sensor_data.system.chip_cores = (int)chip_info.cores;
        snprintf(g_sensor_data.system.mac, sizeof(g_sensor_data.system.mac),
                 "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        xSemaphoreGive(xSensorMutex);
    }
}

void app_main(void)
{
    log_heap_status("startup");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_event_group = xEventGroupCreate();
    if (!s_event_group) {
        ESP_LOGE(TAG_MAIN, "Event group create failed (heap=%u)", (unsigned)esp_get_free_heap_size());
        return;
    }

    /* Create mutex for protecting shared sensor data */
    xSensorMutex = xSemaphoreCreateMutex();
    if (xSensorMutex == NULL) {
        ESP_LOGE(TAG_MAIN, "Failed to create sensor mutex");
        return;
    }

    
    if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(&g_sensor_data, 0, sizeof(g_sensor_data));
        g_sensor_data.system.vbat = -1.0f;
        g_sensor_data.system.cpu_temp = -1000.0f;
        g_sensor_data.system.wifi_channel = -1;
        strlcpy(g_sensor_data.system.ip, "0.0.0.0", sizeof(g_sensor_data.system.ip));
        xSemaphoreGive(xSensorMutex);
    }

    init_topics();

    log_heap_status("pre-wifi");
    wifi_init_sta();
    log_heap_status("post-wifi");

    
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
        init_static_system_info();

    log_heap_status("post-sensor-tasks");
    xTaskCreate(mqtt_start_task, "mqtt_start_task", 4096, NULL, 5, NULL);

    xTaskCreate(ds18b20_task, "ds18b20_task", TASK_STACK_SIZE, NULL, DS18B20_TASK_PRIORITY, NULL);
    xTaskCreate(dht11_task, "dht11_task", TASK_STACK_SIZE, NULL, DHT11_TASK_PRIORITY, NULL);
    xTaskCreate(ultrasonic_task, "ultrasonic_task", TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIORITY, NULL);
    xTaskCreate(mpu6050_task, "mpu6050_task", TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIORITY, NULL);
    xTaskCreate(system_task, "system_task", TASK_STACK_SIZE, NULL, WIFI_TASK_PRIORITY, NULL);

    xTaskCreate(publish_sensors_task, "publish_sensors_task", 4096, NULL, 5, NULL);
    xTaskCreate(publish_gps_task, "publish_gps_task", 4096, NULL, 5, NULL);
    xTaskCreate(publish_status_task, "publish_status_task", 4096, NULL, 5, NULL);
}
