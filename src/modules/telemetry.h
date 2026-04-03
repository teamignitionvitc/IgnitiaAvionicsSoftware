/**
 * @file telemetry.h
 * @brief Telemetry module - USB and NRF Radio
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "mpu6050.h"
#include "bme280.h"
#include "neo_m8m.h"
#include "flight_state.h"
#include "nrf_radio.h"
#include <stdint.h>
#include <stdbool.h>

void telemetry_init(void);
void telemetry_send_packet(const FlightStateContext *flight, const BME280_Data *env,
                           const MPU6050_Data *imu, const GPS_Data *gps);
void telemetry_send_gps(const GPS_Data *gps);
void telemetry_send_status(const char *message);
void telemetry_send_debug(const char *format, ...);
void telemetry_process(void);

#endif // TELEMETRY_H
