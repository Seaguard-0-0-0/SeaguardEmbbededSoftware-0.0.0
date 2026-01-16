#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

// Set Wi-Fi and MQTT here.
#define WIFI_SSID           "HUAWEI-B2368-759429_NOMAP"
#define WIFI_PASS           "92275499ax8f2002"
#define MQTT_BROKER_URI     "mqtt://192.168.1.54:1883"
#define BOAT_ID             "boat1"

// RC input + motor output GPIOs (adjust to your wiring).
// Hardware notes:
// - FS-iA6B PWM may be 5V -> ESP32 needs level shifter/divider.
// - Common ground between receiver, ESP32, and ESCs is required.
// - U01 motors require ESC signal input; ESP32 outputs PWM to ESC.
#define RC_CH1_GPIO         GPIO_NUM_32
#define RC_CH2_GPIO         GPIO_NUM_33
#define RC_CH5_GPIO         GPIO_NUM_14
#define MOTOR_LEFT_GPIO     GPIO_NUM_18
#define MOTOR_RIGHT_GPIO    GPIO_NUM_19

#endif // CONFIG_H
