/**
 * @file buzzer.h
 * @brief Passive buzzer driver (PWM-driven) for RP2040
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

void buzzer_init(void);
void buzzer_tone(uint16_t freq_hz, uint16_t duration_ms);
void buzzer_off(void);
void buzzer_startup_melody(void);
void buzzer_landing_tone(void);

#endif // BUZZER_H
