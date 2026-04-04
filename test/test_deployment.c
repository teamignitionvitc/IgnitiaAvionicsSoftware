/**
 * @file test_deployment.c
 * @brief Unit tests for deployment controller with safety interlocks
 */

#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include "../src/modules/deployment.h"
#include "../src/modules/state_machine.h"

// Mock servo functions
static uint8_t mock_servo_angle = 0;
static bool mock_servo_enabled = false;

bool servo_init(void) {
    mock_servo_angle = 0;
    mock_servo_enabled = false;
    return true;
}

void servo_set_angle(uint8_t angle) {
    mock_servo_angle = angle;
}

uint8_t servo_get_angle(void) {
    return mock_servo_angle;
}

void servo_enable(void) {
    mock_servo_enabled = true;
}

void servo_disable(void) {
    mock_servo_enabled = false;
}

bool servo_is_enabled(void) {
    return mock_servo_enabled;
}

void servo_set_pulse_us(uint16_t pulse_us) {
    // Mock implementation
}

// Mock time functions
uint32_t mock_time_ms = 0;

typedef struct {
    uint32_t _private_us_since_boot;
} absolute_time_t;

absolute_time_t get_absolute_time(void) {
    absolute_time_t t;
    t._private_us_since_boot = mock_time_ms * 1000;
    return t;
}

uint32_t to_ms_since_boot(absolute_time_t t) {
    return t._private_us_since_boot / 1000;
}

void sleep_ms(uint32_t ms) {
    mock_time_ms += ms;
}

// Test helper functions
void reset_test_state(void) {
    mock_servo_angle = 0;
    mock_servo_enabled = false;
    mock_time_ms = 0;
    deployment_reset();
    state_init();
}

void test_deployment_init(void) {
    printf("Test: deployment_init\n");
    
    deployment_init();
    
    assert(mock_servo_angle == 0);  // Servo should be at closed position
    assert(mock_servo_enabled == true);  // Servo should be enabled
    assert(deployment_get_state() == DEPLOY_IDLE);
    assert(!deployment_is_armed());
    assert(!deployment_is_deployed());
    
    printf("  PASS\n");
}

void test_deployment_arm_disarm(void) {
    printf("Test: deployment_arm_disarm\n");
    
    reset_test_state();
    deployment_init();
    
    // Test arming
    deployment_arm();
    assert(deployment_is_armed());
    assert(deployment_get_state() == DEPLOY_READY);
    assert(mock_servo_angle == 0);  // Should still be closed
    
    // Test disarming
    deployment_disarm();
    assert(!deployment_is_armed());
    assert(deployment_get_state() == DEPLOY_IDLE);
    
    printf("  PASS\n");
}

void test_deployment_trigger_not_armed(void) {
    printf("Test: deployment_trigger when not armed\n");
    
    reset_test_state();
    deployment_init();
    
    // Try to trigger without arming
    bool result = deployment_trigger(100.0f);  // 100m altitude
    
    assert(result == false);  // Should fail
    assert(mock_servo_angle == 0);  // Servo should not move
    assert(!deployment_is_deployed());
    
    printf("  PASS\n");
}

void test_deployment_trigger_idle_state(void) {
    printf("Test: deployment_trigger in IDLE state\n");
    
    reset_test_state();
    deployment_init();
    deployment_arm();
    
    // State machine is in IDLE, should reject deployment
    assert(get_state() == STATE_IDLE);
    
    bool result = deployment_trigger(100.0f);
    
    assert(result == false);  // Should fail due to IDLE state
    assert(!deployment_is_deployed());
    
    printf("  PASS\n");
}

void test_deployment_trigger_low_altitude(void) {
    printf("Test: deployment_trigger below minimum altitude\n");
    
    reset_test_state();
    deployment_init();
    deployment_arm();
    
    // Transition to ARMED state
    state_request_transition(STATE_ARMED);
    assert(get_state() == STATE_ARMED);
    
    // Try to deploy at 20m (below 30m minimum)
    bool result = deployment_trigger(20.0f);
    
    assert(result == false);  // Should fail due to low altitude
    assert(!deployment_is_deployed());
    assert(mock_servo_angle == 0);  // Servo should not move
    
    printf("  PASS\n");
}

void test_deployment_trigger_success(void) {
    printf("Test: deployment_trigger successful deployment\n");
    
    reset_test_state();
    deployment_init();
    deployment_arm();
    
    // Transition to ARMED state
    state_request_transition(STATE_ARMED);
    assert(get_state() == STATE_ARMED);
    
    // Deploy at safe altitude (50m)
    bool result = deployment_trigger(50.0f);
    
    assert(result == true);  // Should succeed
    assert(deployment_is_deployed());
    assert(mock_servo_angle == 90);  // Servo should be at open position
    assert(deployment_get_state() == DEPLOY_COMPLETE);
    
    printf("  PASS\n");
}

void test_deployment_trigger_already_deployed(void) {
    printf("Test: deployment_trigger when already deployed\n");
    
    reset_test_state();
    deployment_init();
    deployment_arm();
    
    // Transition to ARMED state and deploy
    state_request_transition(STATE_ARMED);
    deployment_trigger(50.0f);
    assert(deployment_is_deployed());
    
    // Try to deploy again
    bool result = deployment_trigger(50.0f);
    
    assert(result == false);  // Should fail - already deployed
    
    printf("  PASS\n");
}

void test_deployment_test_mode(void) {
    printf("Test: deployment_test bypasses altitude check\n");
    
    reset_test_state();
    deployment_init();
    deployment_arm();
    
    // Test mode should work even without proper state
    bool result = deployment_test();
    
    assert(result == true);  // Should succeed
    // Servo should have moved to open and back to closed
    assert(mock_servo_angle == 0);  // Should be back at closed position
    assert(!deployment_is_deployed());  // Test mode doesn't mark as deployed
    
    printf("  PASS\n");
}

void test_deployment_test_mode_not_armed(void) {
    printf("Test: deployment_test requires armed state\n");
    
    reset_test_state();
    deployment_init();
    
    // Try test mode without arming
    bool result = deployment_test();
    
    assert(result == false);  // Should fail - not armed
    
    printf("  PASS\n");
}

void test_deployment_is_armed(void) {
    printf("Test: deployment_is_armed query function\n");
    
    reset_test_state();
    deployment_init();
    
    assert(!deployment_is_armed());
    
    deployment_arm();
    assert(deployment_is_armed());
    
    deployment_disarm();
    assert(!deployment_is_armed());
    
    printf("  PASS\n");
}

void test_deployment_is_deployed(void) {
    printf("Test: deployment_is_deployed query function\n");
    
    reset_test_state();
    deployment_init();
    deployment_arm();
    
    assert(!deployment_is_deployed());
    
    state_request_transition(STATE_ARMED);
    deployment_trigger(50.0f);
    
    assert(deployment_is_deployed());
    
    printf("  PASS\n");
}

int main(void) {
    printf("\n=== Deployment Controller Tests ===\n\n");
    
    test_deployment_init();
    test_deployment_arm_disarm();
    test_deployment_trigger_not_armed();
    test_deployment_trigger_idle_state();
    test_deployment_trigger_low_altitude();
    test_deployment_trigger_success();
    test_deployment_trigger_already_deployed();
    test_deployment_test_mode();
    test_deployment_test_mode_not_armed();
    test_deployment_is_armed();
    test_deployment_is_deployed();
    
    printf("\n=== All Deployment Tests Passed ===\n\n");
    
    return 0;
}
