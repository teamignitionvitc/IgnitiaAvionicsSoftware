/**
 * @file test_state_transitions.c
 * @brief Comprehensive tests for state transition guard conditions
 * 
 * Validates Requirements: 1.2, 1.3, 1.4, 1.5, 1.6, 1.8, 1.9, 1.10
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Include the state machine module
#include "../src/modules/state_machine.h"
#include "../src/modules/flight_data.h"
#include "../src/config.h"

// Simple test framework
static int tests_run = 0;
static int tests_passed = 0;

#define TEST_ASSERT(cond, msg) do { \
    tests_run++; \
    if (cond) { \
        tests_passed++; \
        printf("  ✓ PASS: %s\n", msg); \
    } else { \
        printf("  ✗ FAIL: %s\n", msg); \
    } \
} while(0)

// Helper function to create sensor data
static SensorData create_sensor_data(float altitude, float velocity, bool sensor_fault) {
    SensorData data = {0};
    data.timestamp_ms = 1000;
    data.dt_s = 0.02f;
    data.filtered_altitude_m = altitude;
    data.fused_velocity_mps = velocity;
    data.sensor_fault = sensor_fault;
    data.imu_ok = true;
    data.bme_ok = true;
    return data;
}

/**
 * Test 1: IDLE → ARMED transition guard
 * Requirement 1.2: altitude > 50m AND sensors healthy
 */
void test_idle_to_armed_guard(void) {
    printf("\n=== Test 1: IDLE → ARMED Guard Conditions ===\n");
    
    // Test 1.1: Valid transition (altitude > 50m, sensors healthy)
    state_init();
    SensorData data = create_sensor_data(60.0f, 0.0f, false);
    bool result = state_request_transition(STATE_ARMED);
    state_update(&data);
    TEST_ASSERT(result && get_state() == STATE_ARMED, 
                "IDLE → ARMED allowed when altitude > 50m and sensors healthy");
    
    // Test 1.2: Reject when altitude too low
    state_init();
    data = create_sensor_data(40.0f, 0.0f, false);
    result = state_request_transition(STATE_ARMED);
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_IDLE, 
                "IDLE → ARMED rejected when altitude < 50m");
    
    // Test 1.3: Reject when sensor fault
    state_init();
    data = create_sensor_data(60.0f, 0.0f, true);
    result = state_request_transition(STATE_ARMED);
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_IDLE, 
                "IDLE → ARMED rejected when sensor fault present");
    
    // Test 1.4: Reject when both conditions fail
    state_init();
    data = create_sensor_data(40.0f, 0.0f, true);
    result = state_request_transition(STATE_ARMED);
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_IDLE, 
                "IDLE → ARMED rejected when altitude low AND sensor fault");
    
    // Test 1.5: Boundary condition - exactly 50m
    state_init();
    data = create_sensor_data(50.0f, 0.0f, false);
    result = state_request_transition(STATE_ARMED);
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_IDLE, 
                "IDLE → ARMED rejected at boundary (altitude = 50m, needs > 50m)");
    
    // Test 1.6: Just above boundary
    state_init();
    data = create_sensor_data(50.1f, 0.0f, false);
    result = state_request_transition(STATE_ARMED);
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_ARMED, 
                "IDLE → ARMED allowed just above boundary (50.1m)");
}

/**
 * Test 2: ARMED → FREEFALL transition guard
 * Requirement 1.3: freefall detected (3 consecutive readings)
 * Note: Freefall detection is handled by flight_logic module
 */
void test_armed_to_freefall_guard(void) {
    printf("\n=== Test 2: ARMED → FREEFALL Guard Conditions ===\n");
    
    // Test 2.1: Valid transition (freefall detected by flight_logic)
    state_init();
    SensorData data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    
    // Simulate freefall detection by flight_logic
    bool result = state_request_transition(STATE_FREEFALL);
    state_update(&data);
    TEST_ASSERT(result && get_state() == STATE_FREEFALL, 
                "ARMED → FREEFALL allowed when freefall detected");
    
    // Test 2.2: Cannot skip to other states
    state_init();
    data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    
    result = state_request_transition(STATE_APOGEE);
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_ARMED, 
                "ARMED → APOGEE rejected (cannot skip FREEFALL)");
}

/**
 * Test 3: FREEFALL → APOGEE transition guard
 * Requirement 1.4: velocity crosses zero
 * Note: Zero-crossing detection is handled by flight_logic module
 */
void test_freefall_to_apogee_guard(void) {
    printf("\n=== Test 3: FREEFALL → APOGEE Guard Conditions ===\n");
    
    // Test 3.1: Valid transition (velocity zero-crossing detected)
    state_init();
    SensorData data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    state_request_transition(STATE_FREEFALL);
    state_update(&data);
    
    // Simulate apogee detection by flight_logic
    bool result = state_request_transition(STATE_APOGEE);
    state_update(&data);
    TEST_ASSERT(result && get_state() == STATE_APOGEE, 
                "FREEFALL → APOGEE allowed when velocity crosses zero");
    
    // Test 3.2: Cannot skip to other states
    state_init();
    data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    state_request_transition(STATE_FREEFALL);
    state_update(&data);
    
    result = state_request_transition(STATE_DEPLOYED);
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_FREEFALL, 
                "FREEFALL → DEPLOYED rejected (cannot skip APOGEE)");
}

/**
 * Test 4: APOGEE → DEPLOYED transition guard
 * Requirement 1.5: deployment triggered
 * Note: Deployment trigger is handled by deployment module
 */
void test_apogee_to_deployed_guard(void) {
    printf("\n=== Test 4: APOGEE → DEPLOYED Guard Conditions ===\n");
    
    // Test 4.1: Valid transition (deployment triggered)
    state_init();
    SensorData data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    state_request_transition(STATE_FREEFALL);
    state_update(&data);
    state_request_transition(STATE_APOGEE);
    state_update(&data);
    
    // Simulate deployment trigger
    bool result = state_request_transition(STATE_DEPLOYED);
    state_update(&data);
    TEST_ASSERT(result && get_state() == STATE_DEPLOYED, 
                "APOGEE → DEPLOYED allowed when deployment triggered");
    
    // Test 4.2: Cannot skip to other states
    state_init();
    data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    state_request_transition(STATE_FREEFALL);
    state_update(&data);
    state_request_transition(STATE_APOGEE);
    state_update(&data);
    
    result = state_request_transition(STATE_LANDED);
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_APOGEE, 
                "APOGEE → LANDED rejected (cannot skip DEPLOYED)");
}

/**
 * Test 5: DEPLOYED → LANDED transition guard
 * Requirement 1.6: velocity < 0.5 m/s AND altitude < 20m for 5 consecutive seconds
 */
void test_deployed_to_landed_guard(void) {
    printf("\n=== Test 5: DEPLOYED → LANDED Guard Conditions ===\n");
    
    // Setup: Get to DEPLOYED state
    state_init();
    SensorData data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    state_request_transition(STATE_FREEFALL);
    state_update(&data);
    state_request_transition(STATE_APOGEE);
    state_update(&data);
    state_request_transition(STATE_DEPLOYED);
    state_update(&data);
    
    // Test 5.1: Conditions not met - high velocity
    data = create_sensor_data(15.0f, 2.0f, false);
    data.timestamp_ms = 1000;
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_DEPLOYED, 
                "DEPLOYED → LANDED not triggered when velocity > 0.5 m/s");
    
    // Test 5.2: Conditions not met - high altitude
    data = create_sensor_data(25.0f, 0.3f, false);
    data.timestamp_ms = 2000;
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_DEPLOYED, 
                "DEPLOYED → LANDED not triggered when altitude > 20m");
    
    // Test 5.3: Conditions met but not for 5 seconds
    data = create_sensor_data(15.0f, 0.3f, false);
    data.timestamp_ms = 3000;
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_DEPLOYED, 
                "DEPLOYED → LANDED not triggered before 5 seconds");
    
    // Test 5.4: Conditions met for 5 seconds - should transition
    data.timestamp_ms = 8000;  // 5 seconds after first meeting conditions
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_LANDED, 
                "DEPLOYED → LANDED triggered after 5 seconds of stable conditions");
    
    // Test 5.5: Timer reset when conditions not met
    state_init();
    data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    state_request_transition(STATE_FREEFALL);
    state_update(&data);
    state_request_transition(STATE_APOGEE);
    state_update(&data);
    state_request_transition(STATE_DEPLOYED);
    state_update(&data);
    
    // Meet conditions
    data = create_sensor_data(15.0f, 0.3f, false);
    data.timestamp_ms = 1000;
    state_update(&data);
    
    // Break conditions (high velocity)
    data = create_sensor_data(15.0f, 2.0f, false);
    data.timestamp_ms = 3000;
    state_update(&data);
    
    // Meet conditions again
    data = create_sensor_data(15.0f, 0.3f, false);
    data.timestamp_ms = 4000;
    state_update(&data);
    
    // Should not transition yet (timer was reset)
    data.timestamp_ms = 6000;  // Only 2 seconds since reset
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_DEPLOYED, 
                "DEPLOYED → LANDED timer resets when conditions break");
}

/**
 * Test 6: Disarm from any state (Any state → IDLE)
 * Requirement 1.10: disarm allowed from any state
 */
void test_disarm_from_any_state(void) {
    printf("\n=== Test 6: Disarm from Any State (→ IDLE) ===\n");
    
    // Test 6.1: Disarm from ARMED
    state_init();
    SensorData data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    bool result = state_request_transition(STATE_IDLE);
    state_update(&data);
    TEST_ASSERT(result && get_state() == STATE_IDLE, 
                "ARMED → IDLE allowed (disarm)");
    
    // Test 6.2: Disarm from FREEFALL
    state_init();
    data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    state_request_transition(STATE_FREEFALL);
    state_update(&data);
    result = state_request_transition(STATE_IDLE);
    state_update(&data);
    TEST_ASSERT(result && get_state() == STATE_IDLE, 
                "FREEFALL → IDLE allowed (disarm)");
    
    // Test 6.3: Disarm from APOGEE
    state_init();
    data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    state_request_transition(STATE_FREEFALL);
    state_update(&data);
    state_request_transition(STATE_APOGEE);
    state_update(&data);
    result = state_request_transition(STATE_IDLE);
    state_update(&data);
    TEST_ASSERT(result && get_state() == STATE_IDLE, 
                "APOGEE → IDLE allowed (disarm)");
    
    // Test 6.4: Disarm from DEPLOYED
    state_init();
    data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    state_request_transition(STATE_FREEFALL);
    state_update(&data);
    state_request_transition(STATE_APOGEE);
    state_update(&data);
    state_request_transition(STATE_DEPLOYED);
    state_update(&data);
    result = state_request_transition(STATE_IDLE);
    state_update(&data);
    TEST_ASSERT(result && get_state() == STATE_IDLE, 
                "DEPLOYED → IDLE allowed (disarm)");
    
    // Test 6.5: Disarm from LANDED
    state_init();
    data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    state_request_transition(STATE_FREEFALL);
    state_update(&data);
    state_request_transition(STATE_APOGEE);
    state_update(&data);
    state_request_transition(STATE_DEPLOYED);
    state_update(&data);
    // Fast-forward to landing
    data = create_sensor_data(15.0f, 0.3f, false);
    data.timestamp_ms = 1000;
    state_update(&data);
    data.timestamp_ms = 6100;
    state_update(&data);
    result = state_request_transition(STATE_IDLE);
    state_update(&data);
    TEST_ASSERT(result && get_state() == STATE_IDLE, 
                "LANDED → IDLE allowed (disarm)");
}

/**
 * Test 7: No state skipping (Requirement 1.8)
 */
void test_no_state_skipping(void) {
    printf("\n=== Test 7: No State Skipping ===\n");
    
    // Test 7.1: Cannot skip from IDLE to FREEFALL
    state_init();
    SensorData data = create_sensor_data(60.0f, 0.0f, false);
    bool result = state_request_transition(STATE_FREEFALL);
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_IDLE, 
                "IDLE → FREEFALL rejected (must go through ARMED)");
    
    // Test 7.2: Cannot skip from ARMED to APOGEE
    state_init();
    data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    result = state_request_transition(STATE_APOGEE);
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_ARMED, 
                "ARMED → APOGEE rejected (must go through FREEFALL)");
    
    // Test 7.3: Cannot skip from FREEFALL to DEPLOYED
    state_init();
    data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    state_request_transition(STATE_FREEFALL);
    state_update(&data);
    result = state_request_transition(STATE_DEPLOYED);
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_FREEFALL, 
                "FREEFALL → DEPLOYED rejected (must go through APOGEE)");
    
    // Test 7.4: Cannot skip from APOGEE to LANDED
    state_init();
    data = create_sensor_data(60.0f, 0.0f, false);
    state_request_transition(STATE_ARMED);
    state_update(&data);
    state_request_transition(STATE_FREEFALL);
    state_update(&data);
    state_request_transition(STATE_APOGEE);
    state_update(&data);
    result = state_request_transition(STATE_LANDED);
    state_update(&data);
    TEST_ASSERT(get_state() == STATE_APOGEE, 
                "APOGEE → LANDED rejected (must go through DEPLOYED)");
}

/**
 * Test 8: State entry timestamp tracking (Requirement 1.7)
 */
void test_state_entry_timestamps(void) {
    printf("\n=== Test 8: State Entry Timestamp Tracking ===\n");
    
    // Test 8.1: Entry time recorded on transition
    state_init();
    SensorData data = create_sensor_data(60.0f, 0.0f, false);
    data.timestamp_ms = 1000;
    state_request_transition(STATE_ARMED);
    state_update(&data);
    uint32_t entry_time = state_entry_time_ms();
    TEST_ASSERT(entry_time == 1000, 
                "State entry time recorded on transition");
    
    // Test 8.2: Entry time updates on next transition
    data.timestamp_ms = 2000;
    state_request_transition(STATE_FREEFALL);
    state_update(&data);
    entry_time = state_entry_time_ms();
    TEST_ASSERT(entry_time == 2000, 
                "State entry time updates on new transition");
    
    // Test 8.3: Time in state calculation
    data.timestamp_ms = 5000;
    uint32_t time_in_state = state_time_in_state_ms(data.timestamp_ms);
    TEST_ASSERT(time_in_state == 3000, 
                "Time in state calculated correctly (5000 - 2000 = 3000)");
}

/**
 * Test 9: State name function
 */
void test_state_names(void) {
    printf("\n=== Test 9: State Name Function ===\n");
    
    TEST_ASSERT(strcmp(state_name(STATE_IDLE), "IDLE") == 0, 
                "STATE_IDLE name is 'IDLE'");
    TEST_ASSERT(strcmp(state_name(STATE_ARMED), "ARMED") == 0, 
                "STATE_ARMED name is 'ARMED'");
    TEST_ASSERT(strcmp(state_name(STATE_FREEFALL), "FREEFALL") == 0, 
                "STATE_FREEFALL name is 'FREEFALL'");
    TEST_ASSERT(strcmp(state_name(STATE_APOGEE), "APOGEE") == 0, 
                "STATE_APOGEE name is 'APOGEE'");
    TEST_ASSERT(strcmp(state_name(STATE_DEPLOYED), "DEPLOYED") == 0, 
                "STATE_DEPLOYED name is 'DEPLOYED'");
    TEST_ASSERT(strcmp(state_name(STATE_LANDED), "LANDED") == 0, 
                "STATE_LANDED name is 'LANDED'");
}

int main(void) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║  State Transition Guard Conditions Test Suite             ║\n");
    printf("║  Validates Requirements: 1.2, 1.3, 1.4, 1.5, 1.6,         ║\n");
    printf("║                          1.8, 1.9, 1.10                    ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    
    test_idle_to_armed_guard();
    test_armed_to_freefall_guard();
    test_freefall_to_apogee_guard();
    test_apogee_to_deployed_guard();
    test_deployed_to_landed_guard();
    test_disarm_from_any_state();
    test_no_state_skipping();
    test_state_entry_timestamps();
    test_state_names();
    
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║  Test Results: %3d/%3d tests passed                        ║\n", tests_passed, tests_run);
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    return (tests_passed == tests_run) ? 0 : 1;
}
