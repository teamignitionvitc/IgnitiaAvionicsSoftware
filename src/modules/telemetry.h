/**
 * @file telemetry.h
 * @brief Telemetry module - USB and NRF Radio
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "flight_data.h"
#include "nrf_radio.h"
#include <stdint.h>
#include <stdbool.h>

void telemetry_init(void);
void telemetry_send(const SensorData *data);
void telemetry_send_status(const char *message);
void telemetry_send_debug(const char *format, ...);
void telemetry_process(void);
uint16_t telemetry_get_sequence(void);

#endif // TELEMETRY_H
