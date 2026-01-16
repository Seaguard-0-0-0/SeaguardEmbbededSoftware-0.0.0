#ifndef MOTOR_OUT_H
#define MOTOR_OUT_H

#include <stdbool.h>
#include <stdint.h>

#define ESC_NEUTRAL_US 1500
#define ESC_MIN_US     1000
#define ESC_MAX_US     2000

extern volatile bool g_esp_override;

void motor_out_init(void);
void motor_out_start(void);
void motor_out_set_us(uint16_t left_us, uint16_t right_us);
void motor_out_stop(void);

#endif // MOTOR_OUT_H
