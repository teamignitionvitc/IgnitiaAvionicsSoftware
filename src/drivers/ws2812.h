/**
 * @file ws2812.h
 * @brief WS2812B RGB LED driver for RP2040-Zero built-in LED
 */

#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>
#include "config.h"

void ws2812_init(void);
void ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b);
void ws2812_off(void);

// Convenience color functions
void ws2812_green(void);        // Running OK
void ws2812_red(void);          // Error
void ws2812_yellow(void);       // Warning
void ws2812_blue(void);         // Initializing
void ws2812_white(void);        // Deploying

// Error indication: blink red N times, then pause
void ws2812_blink_error(uint8_t count);

#endif // WS2812_H
