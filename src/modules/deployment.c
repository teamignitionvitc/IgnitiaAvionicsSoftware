/**
 * @file deployment.c
 * @brief Parachute deployment using dual position servos (MG90S)
 *
 * Init:   Servos go to SERVO_CLOSED_ANGLE (locked)
 * Deploy: Servos go to SERVO_OPEN_ANGLE (released) at apogee
 */

#include "deployment.h"
#include "servo.h"
#include "config.h"
#include "pico/stdlib.h"
#include <stdio.h>

static DeploymentState state = DEPLOY_IDLE;
static uint32_t deploy_time = 0;

void deployment_init(void) {
    servo_init();
    servo_enable();
    // Lock both servos at closed angle
    servo_set_angle(SERVO_CLOSED_ANGLE);
    printf("[DEPLOY] Servos locked at %d deg on GP%d & GP%d\r\n",
           SERVO_CLOSED_ANGLE, SERVO_PIN, SERVO_PIN_2);
    state = DEPLOY_IDLE;
}

void deployment_arm(void) {
    if (state == DEPLOY_IDLE) {
        servo_enable();
        servo_set_angle(SERVO_CLOSED_ANGLE);  // Ensure locked
        state = DEPLOY_READY;
    }
}

void deployment_disarm(void) {
    if (state == DEPLOY_READY) {
        state = DEPLOY_IDLE;
    }
}

bool deployment_trigger(void) {
    // Allow deployment from READY or IDLE state (emergency fallback)
    if (state != DEPLOY_READY && state != DEPLOY_IDLE) {
        return false;
    }

    state = DEPLOY_TRIGGERED;
    deploy_time = to_ms_since_boot(get_absolute_time());

    // Open servos to release parachute
    servo_enable();
    servo_set_angle(SERVO_OPEN_ANGLE);
    printf("[DEPLOY] Servos opened to %d deg\r\n", SERVO_OPEN_ANGLE);

    state = DEPLOY_COMPLETE;
    return true;
}

DeploymentState deployment_get_state(void) {
    return state;
}

bool deployment_is_deployed(void) {
    return state == DEPLOY_COMPLETE;
}

void deployment_test(void) {
    // Test: open then close
    printf("[DEPLOY] Test: opening servos...\r\n");
    servo_enable();
    servo_set_angle(SERVO_OPEN_ANGLE);
    sleep_ms(1000);
    servo_set_angle(SERVO_CLOSED_ANGLE);
    sleep_ms(500);
    printf("[DEPLOY] Test: done\r\n");
}

void deployment_reset(void) {
    servo_set_angle(SERVO_CLOSED_ANGLE);  // Lock
    state = DEPLOY_IDLE;
    deploy_time = 0;
}
