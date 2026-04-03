/**
 * @file bme280.h
 * @brief BME280 Environmental sensor driver for RP2040
 */

#ifndef BME280_H
#define BME280_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float temperature;
    float pressure;
    float humidity;
    float altitude;
} BME280_Data;

bool bme280_init(void);
bool bme280_is_connected(void);
bool bme280_read(BME280_Data *data);
bool bme280_read_pressure(float *pressure);
void bme280_set_ground_pressure(float pressure_pa);
float bme280_get_ground_pressure(void);
float bme280_calculate_altitude(float pressure);
bool bme280_calibrate_ground(uint16_t samples);
void bme280_reset(void);

#endif // BME280_H
