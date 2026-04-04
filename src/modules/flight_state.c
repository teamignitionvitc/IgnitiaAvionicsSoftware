/**
 * @file flight_state.c
 * @brief Flight state machine for Drone Drop mode
 * 
 * States: INIT -> IDLE -> ARMED -> FREEFALL -> DEPLOYED -> LANDED
 * 
 * Drop detection: acceleration < 0.3g (freefall) OR velocity < -2m/s
 * Deploy: Shortly after drop detection when confirmed descending
 */

#include "flight_state.h"
#include "pico/stdlib.h"
#include <string.h>
#include <math.h>

static const char* state_names[] = {
    "IDLE", "ARMED", "FREEFALL", "APOGEE", "DEPLOYED", "LANDED"
};

void flight_state_init(FlightStateContext *ctx) {
    memset(ctx, 0, sizeof(FlightStateContext));
    ctx->current_state = STATE_IDLE;
    ctx->previous_state = STATE_IDLE;
    ctx->state_entry_time = to_ms_since_boot(get_absolute_time());
}

static void transition_to(FlightStateContext *ctx, FlightState new_state) {
    ctx->previous_state = ctx->current_state;
    ctx->current_state = new_state;
    ctx->state_entry_time = to_ms_since_boot(get_absolute_time());
}

void flight_state_update(FlightStateContext *ctx, float altitude, float velocity, float accel_magnitude) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    ctx->current_altitude = altitude;
    ctx->current_velocity = velocity;
    ctx->current_accel = accel_magnitude;
    
    if (altitude > ctx->max_altitude) {
        ctx->max_altitude = altitude;
    }
    
    switch (ctx->current_state) {
        case STATE_IDLE:
            // Wait for arm command - nothing to do here
            break;
            
        case STATE_ARMED:
            // Detect drop: freefall (low g) or rapid descent
            // Freefall: accel magnitude close to 0 (not 1g)
            if (accel_magnitude < DROP_ACCEL_THRESHOLD) {
                ctx->drop_confirm_count++;
            } else if (velocity < DROP_VELOCITY_THRESHOLD) {
                ctx->drop_confirm_count++;
            } else {
                // Reset if not in freefall
                if (ctx->drop_confirm_count > 0) ctx->drop_confirm_count--;
            }
            
            // Also check altitude drop from armed altitude
            if ((ctx->arm_altitude - altitude) > DROP_ALTITUDE_CHANGE) {
                ctx->drop_confirm_count += 2;
            }
            
            // Confirm drop
            if (ctx->drop_confirm_count >= DROP_CONFIRMATION_COUNT) {
                ctx->drop_time = now;
                ctx->drop_altitude = altitude;
                transition_to(ctx, STATE_FREEFALL);
            }
            break;
            
        case STATE_FREEFALL:
            // Brief state - deploy parachute quickly
            // Wait for deploy delay then transition
            if (ctx->deployed) {
                transition_to(ctx, STATE_DEPLOYED);
            }
            break;
            
        case STATE_DEPLOYED:
            // Descending with parachute - detect landing
            if (altitude < LANDING_ALTITUDE_THRESHOLD && 
                fabsf(velocity) < LANDING_VELOCITY_THRESHOLD) {
                // Low and slow - might be landed
                if ((now - ctx->state_entry_time) > LANDING_CONFIRMATION_TIME) {
                    transition_to(ctx, STATE_LANDED);
                }
            }
            break;
            
        case STATE_LANDED:
            // Final state - nothing to do
            break;
            
        case STATE_APOGEE:
            // Apogee detection - transition to deployed when parachute activates
            break;
    }
}

FlightState flight_state_get(const FlightStateContext *ctx) {
    return ctx->current_state;
}

const char* flight_state_name(FlightState state) {
    if (state <= STATE_LANDED) {
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
    
    // Safety: if armed and below safety altitude while descending
    if (ctx->current_state == STATE_ARMED) {
        if (ctx->current_altitude < DEPLOYMENT_SAFETY_ALT && 
            ctx->current_velocity < -1.0f) {
            return true;
        }
    }
    
    return false;
}

void flight_state_arm(FlightStateContext *ctx) {
    if (ctx->current_state == STATE_IDLE) {
        ctx->arm_altitude = ctx->current_altitude;
        ctx->max_altitude = ctx->current_altitude;
        ctx->deployed = false;
        ctx->drop_confirm_count = 0;
        transition_to(ctx, STATE_ARMED);
    }
}

void flight_state_disarm(FlightStateContext *ctx) {
    if (ctx->current_state == STATE_ARMED) {
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
