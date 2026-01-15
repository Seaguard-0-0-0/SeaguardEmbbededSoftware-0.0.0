#include "wifi_mgr.h"
#include "config.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

static const char *TAG = "WIFI_MGR";

static const int WIFI_CONNECTED_BIT = BIT0;

static EventGroupHandle_t s_event_group;
static bool s_initialized;
static bool s_connected;
static int s_rssi = -127;
static int s_channel = -1;
static char s_ssid[33];
static char s_ip[16] = "0.0.0.0";
static int s_retry_num;
static esp_netif_t *s_wifi_netif;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
        }
        s_connected = false;
        s_rssi = -127;
        s_channel = -1;
        strlcpy(s_ip, "0.0.0.0", sizeof(s_ip));
        memset(s_ssid, 0, sizeof(s_ssid));
        if (s_event_group) {
            xEventGroupClearBits(s_event_group, WIFI_CONNECTED_BIT);
        }
        ESP_LOGW(TAG, "Wi-Fi disconnected");
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_retry_num = 0;
        s_connected = true;
        if (event) {
            snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&event->ip_info.ip));
        }

        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            s_rssi = ap_info.rssi;
            s_channel = ap_info.primary;
            strlcpy(s_ssid, (const char *)ap_info.ssid, sizeof(s_ssid));
        }

        if (s_event_group) {
            xEventGroupSetBits(s_event_group, WIFI_CONNECTED_BIT);
        }
        ESP_LOGI(TAG, "Wi-Fi got IP %s", s_ip);
        return;
    }
}

void wifi_init_sta(void)
{
    if (s_initialized) {
        return;
    }

    s_event_group = xEventGroupCreate();
    if (!s_event_group) {
        ESP_LOGE(TAG, "Event group create failed");
        return;
    }

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

    s_initialized = true;
    ESP_LOGI(TAG, "Wi-Fi init done");
}

bool wifi_is_connected(void)
{
    return s_connected;
}

int wifi_get_rssi(void)
{
    if (s_connected) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            s_rssi = ap_info.rssi;
            s_channel = ap_info.primary;
            strlcpy(s_ssid, (const char *)ap_info.ssid, sizeof(s_ssid));
        }
    }
    return s_rssi;
}

int wifi_get_channel(void)
{
    if (s_connected) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            s_channel = ap_info.primary;
        }
    }
    return s_channel;
}

void wifi_get_ip(char *out, size_t len)
{
    if (!out || len == 0) {
        return;
    }
    strlcpy(out, s_ip, len);
}

void wifi_get_ssid(char *out, size_t len)
{
    if (!out || len == 0) {
        return;
    }
    strlcpy(out, s_ssid, len);
}
