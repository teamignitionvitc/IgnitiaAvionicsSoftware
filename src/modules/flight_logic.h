/**
 * @file flight_logic.h
 * @brief Flight detection and deployment decision logic.
 */

#ifndef FLIGHT_LOGIC_H
#define FLIGHT_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include "flight_data.h"

void flight_logic_init(void);
void handle_flight_logic(SensorData *data);

bool flight_logic_should_deploy(void);
void flight_logic_mark_deployed(void);

bool flight_logic_freefall_detected(void);
bool flight_logic_apogee_detected(void);
float flight_logic_velocity_mps(void);
uint32_t flight_logic_drop_time_ms(void);

#endif // FLIGHT_LOGIC_H
