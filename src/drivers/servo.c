/**
 * @file servo.c
 * @brief Dual PWM Servo driver for RP2040 (angle-based position servos)
 *
 * Drives two servos on SERVO_PIN and SERVO_PIN_2 in sync.
 * Set angle 0-180 and servo holds that position.
 */

#include "servo.h"
#include "config.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

// Servo 1 (SERVO_PIN)
static uint slice1;
static uint channel1;

// Servo 2 (SERVO_PIN_2)
static uint slice2;
static uint channel2;

static uint16_t wrap_value;
static uint8_t current_angle = 0;
static bool enabled = false;

bool servo_init(void) {
    // --- Servo 1 ---
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    slice1 = pwm_gpio_to_slice_num(SERVO_PIN);
    channel1 = pwm_gpio_to_channel(SERVO_PIN);

    // --- Servo 2 ---
    gpio_set_function(SERVO_PIN_2, GPIO_FUNC_PWM);
    slice2 = pwm_gpio_to_slice_num(SERVO_PIN_2);
    channel2 = pwm_gpio_to_channel(SERVO_PIN_2);

    // Configure PWM for 50Hz (20ms period) with 1us resolution
    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint32_t divider = clock_freq / (SERVO_PWM_FREQ * 20000);
    wrap_value = 20000 - 1;

    // Servo 1 PWM
    pwm_set_clkdiv(slice1, (float)divider);
    pwm_set_wrap(slice1, wrap_value);
    pwm_set_chan_level(slice1, channel1, 0);
    pwm_set_enabled(slice1, true);

    // Servo 2 PWM
    pwm_set_clkdiv(slice2, (float)divider);
    pwm_set_wrap(slice2, wrap_value);
    pwm_set_chan_level(slice2, channel2, 0);
    pwm_set_enabled(slice2, true);

    // Start at closed angle
    servo_set_angle(SERVO_CLOSED_ANGLE);

    return true;
}

void servo_set_pulse_us(uint16_t pulse_us) {
    if (pulse_us < SERVO_MIN_PULSE_US) pulse_us = SERVO_MIN_PULSE_US;
    if (pulse_us > SERVO_MAX_PULSE_US) pulse_us = SERVO_MAX_PULSE_US;

    if (enabled) {
        pwm_set_chan_level(slice1, channel1, pulse_us);
        pwm_set_chan_level(slice2, channel2, pulse_us);
    }
}

void servo_set_angle(uint8_t angle) {
    if (angle > 180) angle = 180;
    current_angle = angle;

    // Map angle 0-180 to pulse width range
    uint16_t pulse_us = SERVO_MIN_PULSE_US +
        ((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;

    servo_set_pulse_us(pulse_us);
}

uint8_t servo_get_angle(void) {
    return current_angle;
}

void servo_stop(void) {
    // Send neutral pulse to both servos
    if (enabled) {
        pwm_set_chan_level(slice1, channel1, SERVO_NEUTRAL_US);
        pwm_set_chan_level(slice2, channel2, SERVO_NEUTRAL_US);
    }
}

void servo_enable(void) {
    enabled = true;
    // Re-apply current angle
    servo_set_angle(current_angle);
}

void servo_disable(void) {
    enabled = false;
    pwm_set_chan_level(slice1, channel1, 0);
    pwm_set_chan_level(slice2, channel2, 0);
}

bool servo_is_enabled(void) {
    return enabled;
}
