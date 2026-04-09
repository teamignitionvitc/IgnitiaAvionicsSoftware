/**
 * @file flight_state.c
 * @brief Flight state machine for Drone Drop mode
 * 
 * States: INIT -> IDLE -> ARMED -> FREEFALL -> DEPLOYED -> LANDED
 * 
 * DRONE DROP PROFILE:
 *   1. IDLE: Cansat on ground, waiting for arm command or altitude rise
 *   2. ARMED: Attached to drone, ascending/hovering - waiting for drop
 *   3. FREEFALL: Dropped! Detected via low acceleration (<0.5g) or rapid descent
 *   4. DEPLOYED: Parachute released, descending slowly
 *   5. LANDED: On ground, mission complete
 * 
 * Drop detection: acceleration < 0.5g (freefall) AND/OR velocity < -2m/s
 * Deploy: Shortly after drop detection when confirmed descending
 */

#include "flight_state.h"
#include "filters.h"
#include "pico/stdlib.h"
#include <string.h>
#include <math.h>

static const char* state_names[] = {
    "INIT", "IDLE", "ARMED", "FREEFALL", "DEPLOYED", "LANDED", "ERROR"
};

void flight_state_init(FlightStateContext *ctx) {
    memset(ctx, 0, sizeof(FlightStateContext));
    ctx->current_state = STATE_INIT;
    ctx->previous_state = STATE_INIT;
    ctx->state_entry_time = to_ms_since_boot(get_absolute_time());
    ctx->baseline_init_time = ctx->state_entry_time;
}

static void transition_to(FlightStateContext *ctx, FlightState new_state) {
    ctx->previous_state = ctx->current_state;
    ctx->current_state = new_state;
    ctx->state_entry_time = to_ms_since_boot(get_absolute_time());
}

#define ACCEL_FILTER_ALPHA 0.2f
static float filtered_accel = 1.0f;

void flight_state_update(FlightStateContext *ctx, float altitude, float velocity, float accel_magnitude) {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // Initialize and refresh altitude baseline while in non-flight states.
    if (!ctx->altitude_baseline_valid) {
        ctx->baseline_altitude = altitude;
        ctx->altitude_baseline_valid = true;
        ctx->baseline_init_time = now;
    }

    if (ctx->current_state == STATE_INIT || ctx->current_state == STATE_IDLE) {
        // Track baseline only during startup settle window and only while nearly still.
        if ((now - ctx->baseline_init_time) < BASELINE_SETTLE_TIME_MS &&
            fabsf(velocity) < BARO_IDLE_VEL_THRESHOLD) {
            ctx->baseline_altitude = low_pass_filter(ctx->baseline_altitude, altitude, BASELINE_TRACK_ALPHA);
        }
    }

    ctx->current_altitude = altitude;
    ctx->prev_velocity = ctx->current_velocity;
    ctx->current_velocity = velocity;
    
    // Ignore acceleration for drop detection (per user request)
    filtered_accel = 1.0f;
    ctx->current_accel = filtered_accel;
    
    if (altitude > ctx->max_altitude) {
        ctx->max_altitude = altitude;
    }
    
    switch (ctx->current_state) {
        case STATE_INIT:
            transition_to(ctx, STATE_IDLE);
            break;
            
        case STATE_IDLE:
            // Auto-arm when altitude rises significantly (drone taking off with cansat)
            // This detects the drone ascending with the cansat attached
            if ((altitude - ctx->baseline_altitude) > LAUNCH_ALTITUDE_RISE &&
                velocity > LAUNCH_VELOCITY_THRESHOLD) {
                if (ctx->launch_confirm_count < 255) ctx->launch_confirm_count++;
            } else if (ctx->launch_confirm_count > 0) {
                ctx->launch_confirm_count--;
            }

            if (ctx->launch_confirm_count >= LAUNCH_CONFIRMATION_COUNT) {
                ctx->arm_altitude = ctx->baseline_altitude;
                ctx->launch_altitude = altitude;
                ctx->max_altitude = altitude;
                ctx->drop_confirm_count = 0;
                ctx->apogee_confirm_count = 0;
                transition_to(ctx, STATE_ARMED);
            }
            break;
            
        case STATE_ARMED:
            // DRONE DROP DETECTION:
            // When drone releases cansat, we experience freefall (very low acceleration)
            // MUST be at altitude before accepting freefall detection to avoid false positives
            
            // Deploy when altitude drops below max altitude (apogee) and descending
            if (altitude < ctx->max_altitude && velocity < -0.5f) {
                ctx->drop_time = now;
                ctx->drop_altitude = altitude;
                transition_to(ctx, STATE_FREEFALL);
            }

            // Trigger freefall state when drop is confirmed
            if (ctx->drop_confirm_count >= DROP_CONFIRMATION_COUNT) {
                ctx->drop_time = now;
                ctx->drop_altitude = altitude;
                transition_to(ctx, STATE_FREEFALL);
            }
            break;
            
        case STATE_FREEFALL:
            // Brief state - deploy parachute quickly
            // Deployment is handled by flight_state_should_deploy() + deployment module
            if (ctx->deployed) {
                transition_to(ctx, STATE_DEPLOYED);
            }
            break;
            
        case STATE_DEPLOYED:
            // Descending with parachute - detect landing
            // Must be low altitude AND slow velocity for sustained period
            if (altitude < LANDING_ALTITUDE_THRESHOLD && 
                fabsf(velocity) < LANDING_VELOCITY_THRESHOLD) {
                if ((now - ctx->state_entry_time) > LANDING_CONFIRMATION_TIME) {
                    transition_to(ctx, STATE_LANDED);
                }
            }
            break;
            
        case STATE_LANDED:
            // Final state - mission complete
            break;
            
        case STATE_ERROR:
            // Stay in error state
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
    return ctx->current_state >= STATE_ARMED && ctx->current_state < STATE_LANDED;
}

bool flight_state_should_deploy(const FlightStateContext *ctx) {
    if (ctx->deployed) return false;
    
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Deploy in FREEFALL state after delay
    if (ctx->current_state == STATE_FREEFALL) {
        uint32_t time_since_drop = now - ctx->drop_time;
        
        // Deploy after delay, ensure we're actually descending
        if (time_since_drop >= DEPLOY_DELAY_MS && 
            ctx->current_velocity < DEPLOY_VELOCITY_THRESHOLD) {
            return true;
        }
        
        // Safety: force deploy after longer delay regardless
        if (time_since_drop >= DEPLOYMENT_SAFETY_TIME) {
            return true;
        }
    }
    
    // Safety: if armed, was at altitude, now below safety threshold while descending fast
    // This catches cases where freefall detection failed but we're clearly falling
    if (ctx->current_state == STATE_ARMED) {
        // Only trigger if we actually reached flight altitude (max_altitude > ARM_ALTITUDE_MIN)
        // AND we're now below safety threshold AND descending rapidly
        if (ctx->max_altitude > ARM_ALTITUDE_MIN &&
            ctx->current_altitude < DEPLOYMENT_SAFETY_ALT && 
            ctx->current_velocity < -2.0f) {
            return true;
        }
    }
    
    return false;
}

void flight_state_arm(FlightStateContext *ctx) {
    if (ctx->current_state == STATE_IDLE) {
        if (!ctx->altitude_baseline_valid) {
            ctx->baseline_altitude = ctx->current_altitude;
            ctx->altitude_baseline_valid = true;
            ctx->baseline_init_time = to_ms_since_boot(get_absolute_time());
        }
        ctx->arm_altitude = ctx->current_altitude;
        ctx->launch_altitude = ctx->current_altitude;
        ctx->max_altitude = ctx->current_altitude;
        ctx->deployed = false;
        ctx->launch_confirm_count = 0;
        ctx->drop_confirm_count = 0;
        transition_to(ctx, STATE_ARMED);
    }
}

void flight_state_disarm(FlightStateContext *ctx) {
    if (ctx->current_state == STATE_ARMED) {
        ctx->launch_confirm_count = 0;
        ctx->drop_confirm_count = 0;
        transition_to(ctx, STATE_IDLE);
    }
}

void flight_state_mark_deployed(FlightStateContext *ctx) {
    ctx->deployed = true;
}

uint32_t flight_state_get_flight_time(const FlightStateContext *ctx) {
    if (ctx->drop_time == 0) return 0;
    return to_ms_since_boot(get_absolute_time()) - ctx->drop_time;
}
