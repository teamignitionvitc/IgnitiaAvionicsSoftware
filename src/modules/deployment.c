/**
 * @file deployment.c
 * @brief Parachute deployment implementation with safety interlocks
 */

#include "deployment.h"
#include "servo.h"
#include "config.h"
#include "state_machine.h"
#include "pico/stdlib.h"

static DeploymentState state = DEPLOY_IDLE;
static uint32_t deploy_time = 0;
static bool test_mode = false;

void deployment_init(void) {
    servo_init();
    servo_set_angle(SERVO_CLOSED_ANGLE);
    servo_enable();
    state = DEPLOY_IDLE;
    test_mode = false;
}

void deployment_arm(void) {
    if (state == DEPLOY_IDLE) {
        servo_set_angle(SERVO_CLOSED_ANGLE);
        servo_enable();
        state = DEPLOY_READY;
    }
}

void deployment_disarm(void) {
    if (state == DEPLOY_READY) {
        state = DEPLOY_IDLE;
    }
}

bool deployment_trigger(float altitude_m) {
    // Safety interlock 1: Must be armed (DEPLOY_READY state)
    if (state != DEPLOY_READY) {
        return false;
    }
    
    // Safety interlock 2: Must not be in IDLE state (check flight state)
    FlightState flight_state = get_state();
    if (flight_state == STATE_IDLE) {
        return false;
    }
    
    // Safety interlock 3: Altitude must be > 30m (unless in test mode)
    if (!test_mode && altitude_m <= DEPLOY_ALTITUDE_MIN) {
        return false;
    }
    
    // All safety checks passed - proceed with deployment
    state = DEPLOY_TRIGGERED;
    deploy_time = to_ms_since_boot(get_absolute_time());
    
    // Actuate servo to release parachute
    servo_set_angle(SERVO_OPEN_ANGLE);
    
    // Wait for servo to move
    sleep_ms(500);
    
    // Verify deployment (in real system, check continuity or sensor)
    if (servo_get_angle() == SERVO_OPEN_ANGLE) {
        state = DEPLOY_COMPLETE;
        return true;
    }
    
    state = DEPLOY_FAILED;
    return false;
}

bool deployment_is_armed(void) {
    return state == DEPLOY_READY;
}

DeploymentState deployment_get_state(void) {
    return state;
}

bool deployment_is_deployed(void) {
    return state == DEPLOY_COMPLETE;
}

bool deployment_test(void) {
    // Test mode bypasses altitude checks but still requires armed state
    if (state != DEPLOY_READY) {
        return false;
    }
    
    // Enable test mode temporarily
    test_mode = true;
    
    // Test servo movement
    servo_enable();
    servo_set_angle(SERVO_OPEN_ANGLE);
    sleep_ms(1000);
    servo_set_angle(SERVO_CLOSED_ANGLE);
    sleep_ms(500);
    
    // Disable test mode
    test_mode = false;
    
    return true;
}

void deployment_reset(void) {
    servo_set_angle(SERVO_CLOSED_ANGLE);
    state = DEPLOY_IDLE;
    deploy_time = 0;
    test_mode = false;
}
