/*
 * ESP32 Multi-Task Sensor and Motor Control System
 * Framework: ESP-IDF with FreeRTOS
 * 
 * Features:
 * - Task 1: DS18B20 Temperature Sensor Reading
 * - Task 2: DHT11 Temperature & Humidity Sensor Reading
 * - Task 3: Dual DC Motor Control
 * 
 * All tasks run concurrently using FreeRTOS
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "onewire_bus.h"
#include "ds18b20.h"
#include "dht.h"
#include "mqtt.c"

// ============================================================================
// CONFIGURATION & PIN DEFINITIONS
// ============================================================================

// WiFi Configuration
#define WIFI_SSID               "YOUR_WIFI_SSID"
#define WIFI_PASSWORD           "YOUR_WIFI_PASSWORD"
#define WIFI_MAXIMUM_RETRY      5
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAIL_BIT           BIT1

// DS18B20 Configuration (OneWire Protocol)
#define DS18B20_GPIO        GPIO_NUM_4
#define DS18B20_RESOLUTION  DS18B20_RESOLUTION_12B

// DHT11 Configuration
#define DHT11_GPIO          GPIO_NUM_5

// Motor Control Pins
#define MOTOR1_PIN          GPIO_NUM_18
#define MOTOR2_PIN          GPIO_NUM_19

// Task Parameters
#define TASK_STACK_SIZE     4096
#define DS18B20_TASK_PRIORITY   5
#define DHT11_TASK_PRIORITY     5
#define MOTOR_TASK_PRIORITY     4
#define WIFI_TASK_PRIORITY      6


// Timing (milliseconds)
#define DS18B20_READ_INTERVAL   2000
#define DHT11_READ_INTERVAL     2500
#define MOTOR_UPDATE_INTERVAL   1000
#define WIFI_RECONNECT_INTERVAL 5000

// Log Tags
static const char *TAG_DS18B20 = "DS18B20_TASK";
static const char *TAG_DHT11 = "DHT11_TASK";
static const char *TAG_MOTOR = "MOTOR_TASK";
static const char *TAG_WIFI = "WIFI_TASK";

static const char *TAG_MAIN = "MAIN";

// ============================================================================
// GLOBAL VARIABLES & HANDLES
// ============================================================================

// Sensor Handles
static onewire_bus_handle_t onewire_bus = NULL;
static ds18b20_device_handle_t ds18b20_device = NULL;

// Synchronization
static SemaphoreHandle_t xSensorMutex = NULL;

// Shared Sensor Data (protected by mutex)
typedef struct {
    float ds18b20_temp;
    float dht11_temp;
    float dht11_humidity;
    bool ds18b20_valid;
    bool dht11_valid;
    bool wifi_connected;
    char ip_address[16];
} sensor_data_t;

static sensor_data_t g_sensor_data = {0};

// Motor States
static bool motor1_state = false;
static bool motor2_state = false;

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
// MOTOR GPIO INITIALIZATION
// ============================================================================

void init_motors(void)
{
    ESP_LOGI(TAG_MOTOR, "Initializing motor control pins...");
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR1_PIN) | (1ULL << MOTOR2_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    gpio_config(&io_conf);
    
    // Initialize motors to OFF state
    gpio_set_level(MOTOR1_PIN, 0);
    gpio_set_level(MOTOR2_PIN, 0);
    
    ESP_LOGI(TAG_MOTOR, "Motor pins initialized (GPIO %d, %d)", 
             MOTOR1_PIN, MOTOR2_PIN);
}

// ============================================================================
// TASK 1: DS18B20 TEMPERATURE READING
// ============================================================================

void task_ds18b20(void *pvParameters)
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
                    g_sensor_data.ds18b20_temp = temperature;
                    g_sensor_data.ds18b20_valid = true;
                    xSemaphoreGive(xSensorMutex);
                }
                
                ESP_LOGI(TAG_DS18B20, "Temperature: %.2f °C", temperature);
            } else {
                ESP_LOGW(TAG_DS18B20, "Failed to read temperature: %s", 
                         esp_err_to_name(err));
                
                if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    g_sensor_data.ds18b20_valid = false;
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
                g_sensor_data.ds18b20_valid = false;
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

void task_dht11(void *pvParameters)
{
    float temperature;
    float humidity;
    
    ESP_LOGI(TAG_DHT11, "DHT11 task started");
    
    while (1) {
        // Read DHT11 sensor
                
        esp_err_t err = dht_read_float_data(DHT_TYPE_DHT11, DHT11_GPIO, 
                                            &humidity, &temperature);
        
        if (err == ESP_OK) {
            float temp_c = temperature / 10.0;
            float hum_percent = humidity / 10.0;
            
            // Update shared data with mutex protection
            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                g_sensor_data.dht11_temp = temp_c;
                g_sensor_data.dht11_humidity = hum_percent;
                g_sensor_data.dht11_valid = true;
                xSemaphoreGive(xSensorMutex);
            }
            
            ESP_LOGI(TAG_DHT11, "Temperature: %.1f °C, Humidity: %.1f %%", 
                     temp_c, hum_percent);
        } else {
            ESP_LOGW(TAG_DHT11, "Failed to read DHT11: %s", esp_err_to_name(err));
            
            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                g_sensor_data.dht11_valid = false;
                xSemaphoreGive(xSensorMutex);
            }
        }
        
        // Wait before next reading (DHT11 requires minimum 2 seconds between reads)
        vTaskDelay(pdMS_TO_TICKS(DHT11_READ_INTERVAL));
    }
}

// ============================================================================
// TASK 3: MOTOR CONTROL
// ============================================================================

void task_motor_control(void *pvParameters)
{
    uint32_t cycle_count = 0;
    
    ESP_LOGI(TAG_MOTOR, "Motor control task started");
    
    while (1) {
        sensor_data_t local_data;
        
        // Read sensor data with mutex protection
        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            local_data = g_sensor_data;
            xSemaphoreGive(xSensorMutex);
        }
        
        // Motor Control Logic Example:
        // Motor 1: ON if DS18B20 temperature > 25°C
        // Motor 2: ON if DHT11 humidity > 60%
        
        bool new_motor1_state = false;
        bool new_motor2_state = false;
        
        if (local_data.ds18b20_valid && local_data.ds18b20_temp > 25.0) {
            new_motor1_state = true;
        }
        
        if (local_data.dht11_valid && local_data.dht11_humidity > 60.0) {
            new_motor2_state = true;
        }
        
        // Update motor states if changed
        if (new_motor1_state != motor1_state) {
            motor1_state = new_motor1_state;
            gpio_set_level(MOTOR1_PIN, motor1_state ? 1 : 0);
            ESP_LOGI(TAG_MOTOR, "Motor 1: %s", motor1_state ? "ON" : "OFF");
        }
        
        if (new_motor2_state != motor2_state) {
            motor2_state = new_motor2_state;
            gpio_set_level(MOTOR2_PIN, motor2_state ? 1 : 0);
            ESP_LOGI(TAG_MOTOR, "Motor 2: %s", motor2_state ? "ON" : "OFF");
        }
        
        // Periodic status report every 5 seconds
        if (cycle_count % 5 == 0) {
            ESP_LOGI(TAG_MOTOR, "Status - M1: %s, M2: %s | DS18B20: %.1f°C | DHT11: %.1f°C, %.1f%%",
                     motor1_state ? "ON" : "OFF",
                     motor2_state ? "ON" : "OFF",
                     local_data.ds18b20_valid ? local_data.ds18b20_temp : 0.0,
                     local_data.dht11_valid ? local_data.dht11_temp : 0.0,
                     local_data.dht11_valid ? local_data.dht11_humidity : 0.0);
        }
        
        cycle_count++;
        
        // Wait before next control cycle
        vTaskDelay(pdMS_TO_TICKS(MOTOR_UPDATE_INTERVAL));
    }
}


// ============================================================================
// WIFI EVENT HANDLER
// ============================================================================

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG_WIFI, "WiFi started, attempting to connect...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG_WIFI, "Retry to connect to AP (attempt %d/%d)", 
                     s_retry_num, WIFI_MAXIMUM_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGW(TAG_WIFI, "Failed to connect to AP after %d attempts", 
                     WIFI_MAXIMUM_RETRY);
        }
        
        // Update connection status
        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            g_sensor_data.wifi_connected = false;
            strcpy(g_sensor_data.ip_address, "0.0.0.0");
            xSemaphoreGive(xSensorMutex);
        }
        
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_WIFI, "Got IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        
        // Update connection status
        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            g_sensor_data.wifi_connected = true;
            snprintf(g_sensor_data.ip_address, sizeof(g_sensor_data.ip_address),
                     IPSTR, IP2STR(&event->ip_info.ip));
            xSemaphoreGive(xSensorMutex);
        }
    }
}



// ============================================================================
// WIFI INITIALIZATION
// ============================================================================

esp_err_t init_wifi(void)
{
    ESP_LOGI(TAG_WIFI, "Initializing WiFi...");
    
    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create default WiFi station
    esp_netif_create_default_wifi_sta();
    
    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Create WiFi event group
    s_wifi_event_group = xEventGroupCreate();
    
    // Register event handlers
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
    
    // Configure WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG_WIFI, "WiFi initialization complete. Connecting to SSID: %s", WIFI_SSID);
    
    return ESP_OK;
}


// ============================================================================
// TASK 4: WIFI CONNECTION MANAGEMENT
// ============================================================================

void task_wifi_manager(void *pvParameters)
{
    ESP_LOGI(TAG_WIFI, "WiFi manager task started");
    
    while (1) {
        // Wait for connection or failure
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                                WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                                pdFALSE,
                                                pdFALSE,
                                                pdMS_TO_TICKS(WIFI_RECONNECT_INTERVAL));
        
        if (bits & WIFI_CONNECTED_BIT) {
            // WiFi connected successfully
            sensor_data_t local_data;
            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                local_data = g_sensor_data;
                xSemaphoreGive(xSensorMutex);
            }
            
            ESP_LOGI(TAG_WIFI, "Connected to AP | IP: %s", local_data.ip_address);
            
            // Clear the fail bit if it was set
            xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
            
        } else if (bits & WIFI_FAIL_BIT) {
            // Failed to connect, attempt reconnection
            ESP_LOGW(TAG_WIFI, "Connection failed, attempting to reconnect...");
            
            // Reset retry counter and try again
            s_retry_num = 0;
            xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
            
            // Attempt to reconnect
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                ESP_LOGE(TAG_WIFI, "WiFi reconnect failed: %s", esp_err_to_name(err));
            }
            
        } else {
            // Timeout - check connection status
            wifi_ap_record_t ap_info;
            esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
            
            if (err == ESP_OK) {
                // Connected and have AP info
                ESP_LOGD(TAG_WIFI, "WiFi connected - RSSI: %d dBm", ap_info.rssi);
            } else if (err == ESP_ERR_WIFI_NOT_CONNECT) {
                // Not connected, try to reconnect
                ESP_LOGW(TAG_WIFI, "WiFi disconnected, attempting reconnection...");
                s_retry_num = 0;
                esp_wifi_connect();
            }
        }
        
        // Check WiFi status periodically
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// MAIN APPLICATION
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "===========================================");
    ESP_LOGI(TAG_MAIN, "ESP32 Multi-Task Sensor & Motor Control");
    ESP_LOGI(TAG_MAIN, "===========================================");
    
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create mutex for sensor data protection
    xSensorMutex = xSemaphoreCreateMutex();
    if (xSensorMutex == NULL) {
        ESP_LOGE(TAG_MAIN, "Failed to create mutex!");
        return;
    }
    
    // Initialize WiFi
    ESP_LOGI(TAG_MAIN, "Initializing WiFi...");
    init_wifi();
    
    // Initialize DS18B20 sensor
    ESP_LOGI(TAG_MAIN, "Initializing DS18B20...");
    ret = init_ds18b20();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_MAIN, "DS18B20 initialization failed - sensor will be unavailable");
        ESP_LOGW(TAG_MAIN, "Task will attempt to reinitialize periodically");
        ESP_LOGW(TAG_MAIN, "Please check hardware connections:");
        ESP_LOGW(TAG_MAIN, "  - Sensor connected to GPIO %d", DS18B20_GPIO);
        ESP_LOGW(TAG_MAIN, "  - 4.7kΩ pull-up resistor between DATA and VCC");
        ESP_LOGW(TAG_MAIN, "  - VCC to 3.3V, GND to GND");
    }
    
    // Initialize motor control pins
    init_motors();
    
    ESP_LOGI(TAG_MAIN, "Creating FreeRTOS tasks...");
    
    // Create Task 1: DS18B20 Temperature Reading
    // Task will handle missing sensor and attempt reinitialization
    BaseType_t task_created = xTaskCreate(
        task_ds18b20,
        "DS18B20_Task",
        TASK_STACK_SIZE,
        NULL,
        DS18B20_TASK_PRIORITY,
        NULL
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create DS18B20 task!");
    } else {
        ESP_LOGI(TAG_MAIN, "✓ DS18B20 task created");
    }
    
    // Create Task 2: DHT11 Temperature & Humidity Reading
    task_created = xTaskCreate(
        task_dht11,
        "DHT11_Task",
        TASK_STACK_SIZE,
        NULL,
        DHT11_TASK_PRIORITY,
        NULL
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create DHT11 task!");
    } else {
        ESP_LOGI(TAG_MAIN, "✓ DHT11 task created");
    }
    
    // Create Task 3: Motor Control
    task_created = xTaskCreate(
        task_motor_control,
        "Motor_Task",
        TASK_STACK_SIZE,
        NULL,
        MOTOR_TASK_PRIORITY,
        NULL
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create motor control task!");
    } else {
        ESP_LOGI(TAG_MAIN, "✓ Motor control task created");
    }
    
    // Create Task 4: WiFi Manager
    task_created = xTaskCreate(
        task_wifi_manager,
        "WiFi_Task",
        TASK_STACK_SIZE,
        NULL,
        WIFI_TASK_PRIORITY,
        NULL
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create WiFi manager task!");
    } else {
        ESP_LOGI(TAG_MAIN, "✓ WiFi manager task created");
            mqtt_start();
            xTaskCreate(sensor_publish_task, "sensor_publish_task", 4096, NULL, 5, NULL);
    }
    
    ESP_LOGI(TAG_MAIN, "===========================================");
    ESP_LOGI(TAG_MAIN, "All tasks running - system operational");
    ESP_LOGI(TAG_MAIN, "===========================================");
    
    // Main task can delete itself as other tasks are running
    // Or implement additional monitoring/management logic here
}

// ============================================================================
// END OF FILE
// ============================================================================