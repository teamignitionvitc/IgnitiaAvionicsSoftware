/**
 * @file deployment.c
 * @brief Parachute deployment implementation
 */

#include "deployment.h"
#include "servo.h"
#include "config.h"
#include "pico/stdlib.h"

static DeploymentState state = DEPLOY_IDLE;
static uint32_t deploy_time = 0;

void deployment_init(void) {
    servo_init();
    servo_set_angle(SERVO_CLOSED_ANGLE);
    servo_enable();
    state = DEPLOY_IDLE;
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

bool deployment_trigger(void) {
    if (state != DEPLOY_READY) return false;
    
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

DeploymentState deployment_get_state(void) {
    return state;
}

bool deployment_is_deployed(void) {
    return state == DEPLOY_COMPLETE;
}

void deployment_test(void) {
    // Test servo movement without changing state
    servo_enable();
    servo_set_angle(SERVO_OPEN_ANGLE);
    sleep_ms(1000);
    servo_set_angle(SERVO_CLOSED_ANGLE);
    sleep_ms(500);
}

void deployment_reset(void) {
    servo_set_angle(SERVO_CLOSED_ANGLE);
    state = DEPLOY_IDLE;
    deploy_time = 0;
}
