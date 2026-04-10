/**
 * @file flight_state.h
 * @brief Flight state machine for CanSat
 *
 * States: INIT -> ARMED -> APOGEE -> DEPLOYED -> LANDED
 */

#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    FlightState current_state;
    FlightState previous_state;
    uint32_t state_entry_time;
    uint32_t landing_confirm_start; // Time when landing conditions first met
    float baseline_altitude;        // Ground altitude reference
    float max_altitude;             // Maximum recorded altitude (apogee)
    float current_altitude;
    float current_velocity;         // For telemetry only
    float current_accel;            // For telemetry only
    bool deployed;
    bool altitude_baseline_valid;
    uint8_t apogee_confirm_count;   // Consecutive altitude-drop readings for apogee
} FlightStateContext;

void flight_state_init(FlightStateContext *ctx);
void flight_state_update(FlightStateContext *ctx, float altitude, float velocity, float accel_magnitude);
FlightState flight_state_get(const FlightStateContext *ctx);
const char* flight_state_name(FlightState state);
bool flight_state_is_armed(const FlightStateContext *ctx);
bool flight_state_should_deploy(const FlightStateContext *ctx);
void flight_state_arm(FlightStateContext *ctx);
void flight_state_mark_deployed(FlightStateContext *ctx);
uint32_t flight_state_get_flight_time(const FlightStateContext *ctx);

#endif // FLIGHT_STATE_H
