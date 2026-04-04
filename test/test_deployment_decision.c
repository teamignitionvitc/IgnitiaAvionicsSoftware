/**
 * @file test_deployment_decision.c
 * @brief Unit tests for deployment decision logic in flight_logic.c
 * Tests Requirements 5.1, 5.2, 5.3
 */

#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include "../src/modules/flight_logic.h"
#include "../src/modules/state_machine.h"

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
    mock_time_ms = 0;
    flight_logic_init();
    state_init();
}

SensorData create_test_data(float altitude, float velocity, float accel_mag) {
    SensorData data = {0};
    data.timestamp_ms = mock_time_ms;
    data.dt_s = 0.02f;  // 50 Hz
    data.filtered_altitude_m = altitude;
    data.fused_velocity_mps = velocity;
    data.accel_mag_g = accel_mag;
    return data;
}

void test_deployment_trigger_apogee(void) {
    printf("Test: Deployment triggered by apogee detection (Requirement 5.1)\n");
    
    reset_test_state();
    
    // Transition to ARMED state
    state_request_transition(STATE_ARMED);
    
    // Simulate freefall detection (3 consecutive readings)
    for (int i = 0; i < 3; i++) {
        SensorData data = create_test_data(100.0f, -5.0f, 0.2f);
        handle_flight_logic(&data);
        mock_time_ms += 20;
    }
    
    assert(get_state() == STATE_FREEFALL);
    assert(flight_logic_freefall_detected());
    
    // Simulate ascending to apogee
    SensorData data1 = create_test_data(150.0f, 5.0f, 0.2f);
    handle_flight_logic(&data1);
    mock_time_ms += 20;
    
    // Simulate apogee (velocity crosses zero)
    SensorData data2 = create_test_data(150.0f, -0.5f, 0.2f);
    handle_flight_logic(&data2);
    
    assert(flight_logic_apogee_detected());
    assert(flight_logic_should_deploy());
    
    printf("  PASS\n");
}

void test_deployment_trigger_timeout(void) {
    printf("Test: Deployment triggered by 30s timeout (Requirement 5.2)\n");
    
    reset_test_state();
    
    // Transition to ARMED state
    state_request_transition(STATE_ARMED);
    
    // Simulate freefall detection
    for (int i = 0; i < 3; i++) {
        SensorData data = create_test_data(100.0f, -5.0f, 0.2f);
        handle_flight_logic(&data);
        mock_time_ms += 20;
    }
    
    assert(get_state() == STATE_FREEFALL);
    uint32_t drop_time = flight_logic_drop_time_ms();
    
    // Advance time to just before timeout (29 seconds)
    mock_time_ms = drop_time + 29000;
    SensorData data1 = create_test_data(100.0f, -5.0f, 0.2f);
    handle_flight_logic(&data1);
    
    assert(!flight_logic_should_deploy());  // Should not deploy yet
    
    // Advance time past timeout (31 seconds)
    mock_time_ms = drop_time + 31000;
    SensorData data2 = create_test_data(100.0f, -5.0f, 0.2f);
    handle_flight_logic(&data2);
    
    assert(flight_logic_should_deploy());  // Should deploy due to timeout
    
    printf("  PASS\n");
}

void test_deployment_trigger_low_altitude(void) {
    printf("Test: Deployment triggered by low altitude fail-safe (Requirement 5.3)\n");
    
    reset_test_state();
    
    // Transition to ARMED state
    state_request_transition(STATE_ARMED);
    
    // Simulate freefall detection
    for (int i = 0; i < 3; i++) {
        SensorData data = create_test_data(100.0f, -5.0f, 0.2f);
        handle_flight_logic(&data);
        mock_time_ms += 20;
    }
    
    assert(get_state() == STATE_FREEFALL);
    
    // Simulate descending to 50m (above threshold)
    SensorData data1 = create_test_data(50.0f, -10.0f, 0.2f);
    handle_flight_logic(&data1);
    mock_time_ms += 20;
    
    assert(!flight_logic_should_deploy());  // Should not deploy yet
    
    // Simulate descending to 25m (below 30m threshold) while descending
    SensorData data2 = create_test_data(25.0f, -10.0f, 0.2f);
    handle_flight_logic(&data2);
    
    assert(flight_logic_should_deploy());  // Should deploy due to low altitude
    
    printf("  PASS\n");
}

void test_deployment_no_trigger_low_altitude_ascending(void) {
    printf("Test: No deployment when altitude < 30m but ascending (velocity > 0)\n");
    
    reset_test_state();
    
    // Transition to ARMED state
    state_request_transition(STATE_ARMED);
    
    // Simulate freefall detection
    for (int i = 0; i < 3; i++) {
        SensorData data = create_test_data(100.0f, -5.0f, 0.2f);
        handle_flight_logic(&data);
        mock_time_ms += 20;
    }
    
    assert(get_state() == STATE_FREEFALL);
    
    // Simulate low altitude but ascending (shouldn't trigger)
    SensorData data = create_test_data(25.0f, 5.0f, 0.2f);
    handle_flight_logic(&data);
    
    assert(!flight_logic_should_deploy());  // Should NOT deploy (ascending)
    
    printf("  PASS\n");
}

void test_deployment_no_trigger_before_freefall(void) {
    printf("Test: No deployment before freefall state\n");
    
    reset_test_state();
    
    // Stay in ARMED state
    state_request_transition(STATE_ARMED);
    
    // Try various conditions that would trigger deployment in FREEFALL
    SensorData data = create_test_data(25.0f, -10.0f, 1.0f);
    handle_flight_logic(&data);
    
    assert(!flight_logic_should_deploy());  // Should NOT deploy (not in FREEFALL)
    
    printf("  PASS\n");
}

void test_deployment_mark_deployed(void) {
    printf("Test: flight_logic_mark_deployed prevents double deployment\n");
    
    reset_test_state();
    
    // Transition to ARMED state
    state_request_transition(STATE_ARMED);
    
    // Simulate freefall detection
    for (int i = 0; i < 3; i++) {
        SensorData data = create_test_data(100.0f, -5.0f, 0.2f);
        handle_flight_logic(&data);
        mock_time_ms += 20;
    }
    
    // Trigger deployment via low altitude
    SensorData data = create_test_data(25.0f, -10.0f, 0.2f);
    handle_flight_logic(&data);
    
    assert(flight_logic_should_deploy());
    
    // Mark as deployed
    flight_logic_mark_deployed();
    
    assert(!flight_logic_should_deploy());  // Should not deploy again
    
    // Try to trigger again
    SensorData data2 = create_test_data(20.0f, -10.0f, 0.2f);
    handle_flight_logic(&data2);
    
    assert(!flight_logic_should_deploy());  // Still should not deploy
    
    printf("  PASS\n");
}

int main(void) {
    printf("\n=== Deployment Decision Logic Tests ===\n\n");
    
    test_deployment_trigger_apogee();
    test_deployment_trigger_timeout();
    test_deployment_trigger_low_altitude();
    test_deployment_no_trigger_low_altitude_ascending();
    test_deployment_no_trigger_before_freefall();
    test_deployment_mark_deployed();
    
    printf("\n=== All Deployment Decision Tests Passed ===\n\n");
    
    return 0;
}
