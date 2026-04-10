// guva_hw837.c
// Driver for GUVA HW-837 UV sensor
// Simple analog sensor, reads via ADC

#include "guva_hw837.h"
#include <stdio.h>
#include "hardware/adc.h"

#define GUV_HW837_ADC_CHANNEL 0  // Change as per your wiring

void guva_hw837_init(void) {
    adc_init();
    adc_gpio_init(26 + GUV_HW837_ADC_CHANNEL); // GPIO26 = ADC0
}

float guva_hw837_read_uv(void) {
    adc_select_input(GUV_HW837_ADC_CHANNEL);
    uint16_t raw = adc_read();
    // Convert raw ADC value to voltage (3.3V ref, 12-bit ADC)
    float voltage = (raw * 3.3f) / 4095.0f;
    // Convert voltage to UV intensity (user to calibrate)
    // Example: return voltage as-is, or apply calibration curve
    return voltage;
}
