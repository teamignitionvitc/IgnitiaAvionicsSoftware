/**
 * @file flight_state.h
 * @brief Flight state machine for CanSat (Drone Drop Mode)
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
    uint32_t drop_time;             // Time when drop was detected
    uint32_t baseline_init_time;    // Time when altitude baseline was initialized
    float arm_altitude;             // Altitude when armed
    float baseline_altitude;        // Ground/baseline altitude for launch detection
    float launch_altitude;          // Altitude where launch/ascent was confirmed
    float drop_altitude;            // Altitude at drop
    float max_altitude;             // Maximum recorded altitude
    float current_altitude;
    float current_velocity;
    float prev_velocity;
    float current_accel;
    bool deployed;
    bool altitude_baseline_valid;
    uint8_t launch_confirm_count;   // Consecutive ascent readings
    uint8_t apogee_confirm_count;   // Consecutive negative velocity readings
    uint8_t drop_confirm_count;     // Consecutive freefall readings
} FlightStateContext;

void flight_state_init(FlightStateContext *ctx);
void flight_state_update(FlightStateContext *ctx, float altitude, float velocity, float accel_magnitude);
FlightState flight_state_get(const FlightStateContext *ctx);
const char* flight_state_name(FlightState state);
bool flight_state_is_armed(const FlightStateContext *ctx);
bool flight_state_should_deploy(const FlightStateContext *ctx);
void flight_state_arm(FlightStateContext *ctx);
void flight_state_disarm(FlightStateContext *ctx);
void flight_state_mark_deployed(FlightStateContext *ctx);
uint32_t flight_state_get_flight_time(const FlightStateContext *ctx);

#endif // FLIGHT_STATE_H
