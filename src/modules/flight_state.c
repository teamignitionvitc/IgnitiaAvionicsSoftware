/**
 * @file flight_state.c
 * @brief Flight state machine — altitude-only transitions
 *
 * State flow:
 *   INIT -> ARMED       (automatic after calibration)
 *   ARMED -> APOGEE     (altitude drops from peak by APOGEE_DROP_THRESHOLD)
 *   APOGEE -> DEPLOYED  (deployment_trigger() called)
 *   DEPLOYED -> LANDED   (altitude near baseline for LANDING_CONFIRM_TIME_MS)
 */

#include "flight_state.h"
#include "config.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

static const char* state_names[] = {
    "INIT",
    "ARMED",
    "APOGEE",
    "DEPLOYED",
    "LANDED",
    "ERROR"
};

static void change_state(FlightStateContext *ctx, FlightState new_state) {
    if (ctx->current_state == new_state) return;

    printf("[STATE] %s -> %s\r\n",
           state_names[ctx->current_state], state_names[new_state]);

    ctx->previous_state = ctx->current_state;
    ctx->current_state = new_state;
    ctx->state_entry_time = to_ms_since_boot(get_absolute_time());
}

void flight_state_init(FlightStateContext *ctx) {
    ctx->current_state = STATE_INIT;
    ctx->previous_state = STATE_INIT;
    ctx->state_entry_time = to_ms_since_boot(get_absolute_time());
    ctx->landing_confirm_start = 0;
    ctx->baseline_altitude = 0.0f;
    ctx->max_altitude = 0.0f;
    ctx->current_altitude = 0.0f;
    ctx->current_velocity = 0.0f;
    ctx->current_accel = 0.0f;
    ctx->deployed = false;
    ctx->altitude_baseline_valid = false;
    ctx->apogee_confirm_count = 0;
}

void flight_state_update(FlightStateContext *ctx, float altitude, float velocity, float accel_magnitude) {
    ctx->current_altitude = altitude;
    ctx->current_velocity = velocity;
    ctx->current_accel = accel_magnitude;

    uint32_t now = to_ms_since_boot(get_absolute_time());
    float alt_above_baseline = altitude - ctx->baseline_altitude;

    switch (ctx->current_state) {
        case STATE_INIT:
            // INIT -> ARMED is done externally by calling flight_state_arm()
            break;

        case STATE_ARMED:
            // Track max altitude
            if (altitude > ctx->max_altitude) {
                ctx->max_altitude = altitude;
                ctx->apogee_confirm_count = 0;
            }

            // Detect apogee: altitude dropped from peak by threshold
            if (ctx->max_altitude > ctx->baseline_altitude + 1.0f) {
                // Only check for apogee if we've actually ascended at least 1m
                float drop = ctx->max_altitude - altitude;
                if (drop >= APOGEE_DROP_THRESHOLD) {
                    ctx->apogee_confirm_count++;
                    if (ctx->apogee_confirm_count >= APOGEE_CONFIRM_COUNT) {
                        change_state(ctx, STATE_APOGEE);
                    }
                } else {
                    ctx->apogee_confirm_count = 0;
                }
            }
            break;

        case STATE_APOGEE:
            // Apogee state — deployment is triggered externally by main loop
            // checking flight_state_should_deploy().
            // Transition to DEPLOYED is done by flight_state_mark_deployed().
            break;

        case STATE_DEPLOYED:
            // Check for landing: altitude close to baseline for sustained period
            if (alt_above_baseline < LANDING_ALT_THRESHOLD) {
                if (ctx->landing_confirm_start == 0) {
                    ctx->landing_confirm_start = now;
                } else if ((now - ctx->landing_confirm_start) >= LANDING_CONFIRM_TIME_MS) {
                    change_state(ctx, STATE_LANDED);
                }
            } else {
                ctx->landing_confirm_start = 0;  // Reset timer
            }
            break;

        case STATE_LANDED:
            // Terminal state — stay here
            break;

        case STATE_ERROR:
            break;
    }
}

FlightState flight_state_get(const FlightStateContext *ctx) {
    return ctx->current_state;
}

const char* flight_state_name(FlightState state) {
    if (state <= STATE_ERROR) {
        return state_names[state];
    }
    return "UNKNOWN";
}

bool flight_state_is_armed(const FlightStateContext *ctx) {
    return ctx->current_state >= STATE_ARMED && ctx->current_state < STATE_ERROR;
}

bool flight_state_should_deploy(const FlightStateContext *ctx) {
    if (ctx->deployed) return false;

    if (ctx->current_state == STATE_APOGEE) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        uint32_t time_in_apogee = now - ctx->state_entry_time;

        // Deploy after DEPLOY_DELAY_MS
        if (time_in_apogee >= DEPLOY_DELAY_MS) {
            return true;
        }

        // Safety: force deploy after DEPLOYMENT_SAFETY_TIME
        if (time_in_apogee >= DEPLOYMENT_SAFETY_TIME) {
            return true;
        }
    }

    return false;
}

void flight_state_arm(FlightStateContext *ctx) {
    if (ctx->current_state == STATE_INIT) {
        change_state(ctx, STATE_ARMED);
        printf("[STATE] System ARMED — monitoring altitude for apogee\r\n");
    }
}

void flight_state_mark_deployed(FlightStateContext *ctx) {
    ctx->deployed = true;
    if (ctx->current_state == STATE_APOGEE) {
        change_state(ctx, STATE_DEPLOYED);
    }
}

uint32_t flight_state_get_flight_time(const FlightStateContext *ctx) {
    return to_ms_since_boot(get_absolute_time()) - ctx->state_entry_time;
}
