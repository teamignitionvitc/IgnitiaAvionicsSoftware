/**
 * @file servo.c
 * @brief PWM Servo driver implementation for RP2040
 */

#include "servo.h"
#include "config.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

static uint slice_num;
static uint channel;
static uint16_t wrap_value;
static uint8_t current_angle = 0;
static bool enabled = false;

bool servo_init(void) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    
    slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    channel = pwm_gpio_to_channel(SERVO_PIN);
    
    // Configure PWM for 50Hz (20ms period)
    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint32_t divider = clock_freq / (SERVO_PWM_FREQ * 20000);
    wrap_value = 20000 - 1; // 20ms period with 1us resolution
    
    pwm_set_clkdiv(slice_num, (float)divider);
    pwm_set_wrap(slice_num, wrap_value);
    
    // Start with servo disabled
    pwm_set_chan_level(slice_num, channel, 0);
    pwm_set_enabled(slice_num, true);
    
    servo_set_angle(SERVO_CLOSED_ANGLE);
    
    return true;
}

void servo_set_angle(uint8_t angle) {
    if (angle > 180) angle = 180;
    current_angle = angle;
    
    // Map angle to pulse width
    uint16_t pulse_us = SERVO_MIN_PULSE_US + 
        ((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;
    
    servo_set_pulse_us(pulse_us);
}

void servo_set_pulse_us(uint16_t pulse_us) {
    if (pulse_us < SERVO_MIN_PULSE_US) pulse_us = SERVO_MIN_PULSE_US;
    if (pulse_us > SERVO_MAX_PULSE_US) pulse_us = SERVO_MAX_PULSE_US;
    
    if (enabled) {
        pwm_set_chan_level(slice_num, channel, pulse_us);
    }
}

uint8_t servo_get_angle(void) {
    return current_angle;
}

void servo_enable(void) {
    enabled = true;
    uint16_t pulse_us = SERVO_MIN_PULSE_US + 
        ((uint32_t)current_angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;
    pwm_set_chan_level(slice_num, channel, pulse_us);
}

void servo_disable(void) {
    enabled = false;
    pwm_set_chan_level(slice_num, channel, 0);
}

bool servo_is_enabled(void) {
    return enabled;
}
