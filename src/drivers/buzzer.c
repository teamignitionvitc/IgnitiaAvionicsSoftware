/**
 * @file buzzer.c
 * @brief Passive buzzer driver using PWM for RP2040
 *
 * Drives a passive buzzer with a square wave at the desired frequency.
 * Used only during initialization and when LANDED.
 */

#include "buzzer.h"
#include "config.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

static uint slice_num;
static uint channel;

void buzzer_init(void) {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    channel = pwm_gpio_to_channel(BUZZER_PIN);

    // Start with buzzer off
    pwm_set_enabled(slice_num, false);
}

void buzzer_tone(uint16_t freq_hz, uint16_t duration_ms) {
    if (freq_hz == 0) {
        buzzer_off();
        return;
    }

    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint32_t wrap = clock_freq / freq_hz - 1;

    // Use clock divider if wrap is too large
    float divider = 1.0f;
    while (wrap > 65535) {
        divider *= 2.0f;
        wrap /= 2;
    }

    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, (uint16_t)wrap);
    pwm_set_chan_level(slice_num, channel, (uint16_t)(wrap / 2));  // 50% duty
    pwm_set_enabled(slice_num, true);

    if (duration_ms > 0) {
        sleep_ms(duration_ms);
        buzzer_off();
    }
}

void buzzer_off(void) {
    pwm_set_enabled(slice_num, false);
    gpio_put(BUZZER_PIN, 0);
}

void buzzer_startup_melody(void) {
    // Short ascending melody to indicate init complete
    buzzer_tone(1000, 100);
    sleep_ms(50);
    buzzer_tone(1500, 100);
    sleep_ms(50);
    buzzer_tone(2000, 150);
}

void buzzer_landing_tone(void) {
    // Repeating beep to help locate the CanSat after landing
    buzzer_tone(BUZZER_FREQ_LANDED, BUZZER_BEEP_MS);
    sleep_ms(500);
    buzzer_tone(BUZZER_FREQ_LANDED, BUZZER_BEEP_MS);
}
