/**
 * @file flight_logic.c
 * @brief Core detection algorithms: freefall, velocity, apogee, deployment.
 */

#include "flight_logic.h"
#include "state_machine.h"
#include "filters.h"

// CRITICAL FIX (Issue 1): Use fixed-point thresholds for deterministic timing
#define FREEFALL_ACCEL_THRESHOLD_G      0.3f
#define FREEFALL_VELOCITY_THRESHOLD_CMPS -200  // cm/s (-2.0 m/s) - fixed-point
#define FREEFALL_CONFIRMATION_COUNT     3u

// CRITICAL FIX (Issue 3): Apogee detection with hysteresis to prevent false triggers
#define APOGEE_POSITIVE_THRESHOLD_CMPS  50    // cm/s (0.5 m/s) - must be clearly ascending
#define APOGEE_NEGATIVE_THRESHOLD_CMPS  -20   // cm/s (-0.2 m/s) - must be clearly descending
#define APOGEE_CONFIRMATION_COUNT       3u    // Require 3 consecutive negative readings

#define DEPLOY_TIMEOUT_MS               30000u
#define DEPLOY_MIN_ALTITUDE_M           30.0f  // Requirement 5.3: low-altitude fail-safe

static float velocity_prev = 0.0f;
static int16_t velocity_prev_cmps = 0;  // Track previous velocity in cm/s for apogee detection
static bool was_ascending = false;  // Track if we were ascending (for apogee hysteresis)

static bool freefall_detected = false;
static bool apogee_detected = false;
static bool deploy_requested = false;
static bool deployed = false;

static uint8_t freefall_confirm_count = 0;
static uint8_t apogee_confirm_count = 0;  // CRITICAL FIX: Track consecutive negative readings
static uint32_t drop_time = 0;

void flight_logic_init(void) {
    velocity_prev = 0.0f;
    velocity_prev_cmps = 0;
    was_ascending = false;

    freefall_detected = false;
    apogee_detected = false;
    deploy_requested = false;
    deployed = false;

    freefall_confirm_count = 0;
    apogee_confirm_count = 0;  // CRITICAL FIX: Reset apogee confirmation counter
    drop_time = 0;
}

void handle_flight_logic(SensorData *data) {
    if (!data) {
        return;
    }

    FlightState state = get_state();
    
    // CRITICAL FIX (Issue 1): Use fixed-point velocity for deterministic timing
    int16_t velocity_current_cmps = data->velocity_cmps;

    // A. Freefall Detection with 3-reading confirmation counter.
    // Requirements: 2.1, 2.2, 2.5, 2.6
    // IMPROVED: Use fixed-point velocity for deterministic timing
    // Detect freefall when: accel_mag < 0.3g AND velocity < -200 cm/s (-2.0 m/s) for 3 consecutive readings
    if (!freefall_detected && state == STATE_ARMED) {
        // Check both acceleration and velocity criteria (fixed-point)
        if (data->accel_mag_g < FREEFALL_ACCEL_THRESHOLD_G && 
            velocity_current_cmps < FREEFALL_VELOCITY_THRESHOLD_CMPS) {
            // Increment confirmation counter
            freefall_confirm_count++;
            
            // Require 3 consecutive confirmations for debouncing
            if (freefall_confirm_count >= FREEFALL_CONFIRMATION_COUNT) {
                freefall_detected = true;
                drop_time = data->timestamp_ms;
            }
        } else {
            // Reset counter on normal acceleration or velocity
            freefall_confirm_count = 0;
        }
    }

    // B. Apogee Detection with HYSTERESIS (CRITICAL FIX - Issue 3 + Issue 1)
    // Requirements: 4.1, 4.3, 4.4
    // IMPROVED: Use hysteresis + fixed-point to prevent false triggers from noise
    // - prev_velocity > 50 cm/s (0.5 m/s) - must be clearly ascending
    // - current_velocity < -20 cm/s (-0.2 m/s) - must be clearly descending
    // - Require 3 consecutive negative readings for confirmation
    // - Use fixed-point velocity for deterministic timing
    if (!apogee_detected && state == STATE_FREEFALL) {
        // Track if we were ascending (above positive threshold)
        if (velocity_current_cmps > APOGEE_POSITIVE_THRESHOLD_CMPS) {
            was_ascending = true;
        }
        
        // Check for zero-crossing: was ascending, now descending
        if (was_ascending && velocity_current_cmps < APOGEE_NEGATIVE_THRESHOLD_CMPS) {
            // Increment confirmation counter
            apogee_confirm_count++;
            
            // Require 3 consecutive confirmations to prevent false triggers
            if (apogee_confirm_count >= APOGEE_CONFIRMATION_COUNT) {
                apogee_detected = true;
            }
        } else if (velocity_current_cmps >= APOGEE_NEGATIVE_THRESHOLD_CMPS) {
            // Reset counter if velocity goes back above negative threshold
            apogee_confirm_count = 0;
        }
        
        // Update previous velocity for next iteration
        velocity_prev_cmps = velocity_current_cmps;
    }

    // C. State Transitions
    if (state == STATE_ARMED && freefall_detected) {
        state_request_transition(STATE_FREEFALL);
    }

    if (state == STATE_FREEFALL && apogee_detected) {
        state_request_transition(STATE_APOGEE);
    }

    // D. Deployment Logic: apogee OR timeout OR low-altitude fail-safe.
    // Requirements: 5.1, 5.2, 5.3
    // IMPROVED: Use fixed-point velocity for deterministic timing
    // Three triggers:
    // 1. Primary: apogee detected
    // 2. Fail-safe 1: time since drop > 30 seconds
    // 3. Fail-safe 2: altitude < 30m while descending (velocity < 0)
    if (!deployed && (state == STATE_FREEFALL || state == STATE_APOGEE)) {
        bool deploy_timeout = (drop_time > 0) && ((data->timestamp_ms - drop_time) >= DEPLOY_TIMEOUT_MS);
        bool deploy_low_altitude = (data->filtered_altitude_m < DEPLOY_MIN_ALTITUDE_M) && (velocity_current_cmps < 0);

        if (apogee_detected || deploy_timeout || deploy_low_altitude) {
            deploy_requested = true;
        }
    }
}

bool flight_logic_should_deploy(void) {
    return deploy_requested && !deployed;
}

void flight_logic_mark_deployed(void) {
    deployed = true;
    deploy_requested = false;
}

bool flight_logic_freefall_detected(void) {
    return freefall_detected;
}

bool flight_logic_apogee_detected(void) {
    return apogee_detected;
}

float flight_logic_velocity_mps(void) {
    return velocity_prev;
}

uint32_t flight_logic_drop_time_ms(void) {
    return drop_time;
}
