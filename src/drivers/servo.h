/**
 * @file servo.h
 * @brief Dual PWM Servo driver for RP2040 (360° continuous rotation)
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdbool.h>

bool servo_init(void);
void servo_set_angle(uint8_t angle);
void servo_set_pulse_us(uint16_t pulse_us);
uint8_t servo_get_angle(void);
void servo_stop(void);           // Send neutral pulse (stopped for 360° servo)
void servo_enable(void);
void servo_disable(void);
bool servo_is_enabled(void);

#endif // SERVO_H
