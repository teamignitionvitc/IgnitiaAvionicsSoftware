/**
 * @file state_machine.c
 * @brief Central flight state machine with guarded transitions.
 */

#include "state_machine.h"
#include "config.h"

#include <math.h>

static FlightState current_state = STATE_IDLE;
static FlightState previous_state = STATE_IDLE;
static FlightState requested_state = STATE_IDLE;

static bool has_request = false;
static uint32_t entry_time = 0;
static uint32_t landed_candidate_start = 0;

static bool transition_allowed(FlightState from, FlightState to, SensorData *data) {
    if (from == to) {
        return true;
    }

    // Allow disarm (transition to IDLE) from any state
    if (to == STATE_IDLE) {
        return true;
    }

    // Enforce strict state progression with guard conditions
    switch (from) {
        case STATE_IDLE:
            // IDLE → ARMED: Altitude > 50m AND sensors healthy
            if (to == STATE_ARMED) {
                if (!data) return false;
                return (data->filtered_altitude_m > ARM_ALTITUDE_MIN) && 
                       (!data->sensor_fault);
            }
            return false;

        case STATE_ARMED:
            // ARMED → FREEFALL: Freefall detected (handled by flight_logic)
            if (to == STATE_FREEFALL) {
                return true;
            }
            return false;

        case STATE_FREEFALL:
            // FREEFALL → APOGEE: Velocity crosses zero (handled by flight_logic)
            if (to == STATE_APOGEE) {
                return true;
            }
            return false;

        case STATE_APOGEE:
            // APOGEE → DEPLOYED: Deployment triggered (handled by deployment module)
            if (to == STATE_DEPLOYED) {
                return true;
            }
            return false;

        case STATE_DEPLOYED:
            // DEPLOYED → LANDED: Landing detected (handled in state_update)
            if (to == STATE_LANDED) {
                return true;
            }
            return false;

        case STATE_LANDED:
            // LANDED → IDLE: Allow reset
            if (to == STATE_IDLE) {
                return true;
            }
            return false;

        default:
            return false;
    }
}

static void apply_transition(FlightState next, uint32_t now_ms) {
    previous_state = current_state;
    current_state = next;
    entry_time = now_ms;

    if (current_state != STATE_DEPLOYED) {
        landed_candidate_start = 0;
    }
}

void state_init(void) {
    current_state = STATE_IDLE;
    previous_state = STATE_IDLE;
    requested_state = STATE_IDLE;
    has_request = false;
    entry_time = 0;
    landed_candidate_start = 0;
}

bool state_request_transition(FlightState next_state) {
    // Pre-validate transition without sensor data (will be validated again in state_update)
    // Allow the request if it's a valid state progression
    if (next_state == STATE_IDLE) {
        // Always allow disarm request
        requested_state = next_state;
        has_request = true;
        return true;
    }

    // For IDLE → ARMED, we need sensor data to validate, so accept the request
    // and validate in state_update when we have the data
    if (current_state == STATE_IDLE && next_state == STATE_ARMED) {
        requested_state = next_state;
        has_request = true;
        return true;
    }

    // For other transitions, check basic progression validity
    bool valid_progression = false;
    switch (current_state) {
        case STATE_ARMED:
            valid_progression = (next_state == STATE_FREEFALL);
            break;
        case STATE_FREEFALL:
            valid_progression = (next_state == STATE_APOGEE);
            break;
        case STATE_APOGEE:
            valid_progression = (next_state == STATE_DEPLOYED);
            break;
        case STATE_DEPLOYED:
            valid_progression = (next_state == STATE_LANDED);
            break;
        case STATE_LANDED:
            valid_progression = (next_state == STATE_IDLE);
            break;
        default:
            valid_progression = false;
    }

    if (valid_progression) {
        requested_state = next_state;
        has_request = true;
        return true;
    }

    return false;
}

void state_update(SensorData *data) {
    uint32_t now_ms = data ? data->timestamp_ms : 0;

    // Process pending transition request with guard condition validation
    if (has_request) {
        if (transition_allowed(current_state, requested_state, data)) {
            apply_transition(requested_state, now_ms);
        }
        has_request = false;
    }

    if (!data) {
        return;
    }

    // Automatic landing detection when in DEPLOYED state
    if (current_state == STATE_DEPLOYED) {
        bool low_altitude = data->filtered_altitude_m < LANDING_ALTITUDE_THRESHOLD;
        bool low_speed = fabsf(data->fused_velocity_mps) < LANDING_VELOCITY_THRESHOLD;

        if (low_altitude && low_speed) {
            if (landed_candidate_start == 0) {
                landed_candidate_start = now_ms;
            }

            // Require continuous satisfaction for 5 seconds
            if ((now_ms - landed_candidate_start) >= LANDING_CONFIRMATION_TIME) {
                apply_transition(STATE_LANDED, now_ms);
            }
        } else {
            // Reset timer if conditions not met
            landed_candidate_start = 0;
        }
    }
}

FlightState get_state(void) {
    return current_state;
}

const char *state_name(FlightState state) {
    switch (state) {
        case STATE_IDLE: return "IDLE";
        case STATE_ARMED: return "ARMED";
        case STATE_FREEFALL: return "FREEFALL";
        case STATE_APOGEE: return "APOGEE";
        case STATE_DEPLOYED: return "DEPLOYED";
        case STATE_LANDED: return "LANDED";
        default: return "UNKNOWN";
    }
}

uint32_t state_entry_time_ms(void) {
    return entry_time;
}

uint32_t state_time_in_state_ms(uint32_t now_ms) {
    return now_ms - entry_time;
}
