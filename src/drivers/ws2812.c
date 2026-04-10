/**
 * @file ws2812.c
 * @brief WS2812B RGB LED driver using bit-bang for RP2040-Zero
 *
 * The RP2040-Zero has a single WS2812B on GP16.
 * Uses precise cycle-counted delays with interrupts disabled.
 */

#include "ws2812.h"
#include "config.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

// WS2812B timing at 125 MHz (8ns per cycle)
// T0H = 400ns = 50 cycles, T0L = 850ns = 106 cycles
// T1H = 800ns = 100 cycles, T1L = 450ns = 56 cycles

static inline void delay_short(void) {
    // ~400ns at 125 MHz
    __asm volatile(
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
    );
}

static inline void delay_long(void) {
    // ~800ns at 125 MHz
    __asm volatile(
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
        "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
    );
}

void ws2812_init(void) {
    gpio_init(WS2812_PIN);
    gpio_set_dir(WS2812_PIN, GPIO_OUT);
    gpio_put(WS2812_PIN, 0);
    sleep_us(100);  // Reset
}

void ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    // WS2812B expects GRB order, MSB first
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;

    uint32_t save = save_and_disable_interrupts();

    for (int i = 23; i >= 0; i--) {
        if (grb & (1u << i)) {
            // Bit 1: long high, short low
            gpio_put(WS2812_PIN, 1);
            delay_long();
            gpio_put(WS2812_PIN, 0);
            delay_short();
        } else {
            // Bit 0: short high, long low
            gpio_put(WS2812_PIN, 1);
            delay_short();
            gpio_put(WS2812_PIN, 0);
            delay_long();
        }
    }

    restore_interrupts(save);
    sleep_us(60);  // Reset pulse
}

void ws2812_off(void) {
    ws2812_set_rgb(0, 0, 0);
}

void ws2812_green(void) {
    ws2812_set_rgb(0, 30, 0);   // Dim green (saves power, not blinding)
}

void ws2812_red(void) {
    ws2812_set_rgb(30, 0, 0);
}

void ws2812_yellow(void) {
    ws2812_set_rgb(30, 20, 0);
}

void ws2812_blue(void) {
    ws2812_set_rgb(0, 0, 30);
}

void ws2812_white(void) {
    ws2812_set_rgb(20, 20, 20);
}

void ws2812_blink_error(uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        ws2812_red();
        sleep_ms(200);
        ws2812_off();
        sleep_ms(200);
    }
    sleep_ms(600);  // Pause between blink groups
}
