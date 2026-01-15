#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float temp;
    bool valid;
} ds18b20_data_t;

typedef struct {
    float temp;
    float humidity;
    bool valid;
} dht11_data_t;

typedef struct {
    float ultrasonic_cm;
    bool ultrasonic_ok;
} ultrasonic_data_t;

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float temp_c;
    bool ok;
} mpu6050_data_t;

typedef struct {
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
} sensor_system_t;

typedef struct {
    int64_t timestamp_ms;
    ds18b20_data_t ds18b20;
    dht11_data_t dht11;
    ultrasonic_data_t ultrasonic;
    mpu6050_data_t mpu6050;
    sensor_system_t system;
} sensor_data_t;

extern sensor_data_t g_sensor_data;

void sensor_data_init(void);
void sensor_data_lock(void);
void sensor_data_unlock(void);

void sensor_data_set_timestamp(int64_t ts_ms);
void sensor_data_set_ds18b20(float temp, bool valid);
void sensor_data_set_dht11(float temp, float humidity, bool valid);
void sensor_data_set_ultrasonic(float cm, bool ok);
void sensor_data_set_mpu(float ax, float ay, float az,
                         float gx, float gy, float gz,
                         float temp_c, bool ok);
void sensor_data_set_system(const sensor_system_t *sys);
void sensor_data_set_chip_info(int model, int revision, int cores);
void sensor_data_set_mac(const char *mac);

void sensor_data_get_copy(sensor_data_t *out);

#endif // SENSOR_DATA_H
