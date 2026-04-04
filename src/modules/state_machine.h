/**
 * @file state_machine.h
 * @brief Flight state controller and transition guards.
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>
#include "flight_data.h"

typedef enum {
    STATE_IDLE = 0,
    STATE_ARMED,
    STATE_FREEFALL,
    STATE_APOGEE,
    STATE_DEPLOYED,
    STATE_LANDED
} FlightState;

void state_init(void);
void state_update(SensorData *data);
FlightState get_state(void);

bool state_request_transition(FlightState next_state);
const char *state_name(FlightState state);
uint32_t state_entry_time_ms(void);
uint32_t state_time_in_state_ms(uint32_t now_ms);

#endif // STATE_MACHINE_H
