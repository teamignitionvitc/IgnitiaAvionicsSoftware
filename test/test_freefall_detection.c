/**
 * @file test_freefall_detection.c
 * @brief Unit tests for freefall detection algorithm
 * 
 * Tests Requirements: 2.1, 2.2, 2.5, 2.6
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

// Test framework
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

// Mock includes for testing
#include "../src/modules/flight_data.h"
#include "../src/modules/state_machine.h"

// Mock state machine state
static FlightState mock_state = STATE_IDLE;

// Override state machine functions
#undef get_state
#undef state_request_transition

FlightState get_state(void) {
    return mock_state;
}

bool state_request_transition(FlightState new_state) {
    mock_state = new_state;
    return true;
}

// Include filters for MovingAverage
#include "../src/utils/filters.c"

// Include flight_logic implementation
#include "../src/modules/flight_logic.c"

// Helper to create sensor data
SensorData create_test_data(float accel_mag, float velocity, uint32_t timestamp) {
    SensorData data = {0};
    data.timestamp_ms = timestamp;
    data.dt_s = 0.02f;  // 50 Hz
    data.accel_mag_g = accel_mag;
    data.fused_velocity_mps = velocity;
    data.velocity_cmps = (int16_t)(velocity * 100.0f);  // Convert m/s to cm/s
    data.imu_ok = true;
    data.bme_ok = true;
    return data;
}

void test_freefall_not_detected_single_reading(void) {
    printf("\n=== Test: Single reading below threshold should NOT trigger ===\n");
    
    flight_logic_init();
    mock_state = STATE_ARMED;
    
    // Single reading with low accel and negative velocity
    SensorData data = create_test_data(0.2f, -3.0f, 1000);
    handle_flight_logic(&data);
    
    TEST_ASSERT(!flight_logic_freefall_detected(), 
                "Single reading should not trigger freefall");
    TEST_ASSERT(mock_state == STATE_ARMED, 
                "Should remain in ARMED state");
}

void test_freefall_detected_three_consecutive(void) {
    printf("\n=== Test: Three consecutive readings should trigger ===\n");
    
    flight_logic_init();
    mock_state = STATE_ARMED;
    
    // First reading
    SensorData data1 = create_test_data(0.2f, -3.0f, 1000);
    handle_flight_logic(&data1);
    TEST_ASSERT(!flight_logic_freefall_detected(), 
                "First reading should not trigger");
    
    // Second reading
    SensorData data2 = create_test_data(0.25f, -2.5f, 1020);
    handle_flight_logic(&data2);
    TEST_ASSERT(!flight_logic_freefall_detected(), 
                "Second reading should not trigger");
    
    // Third reading - should trigger
    SensorData data3 = create_test_data(0.28f, -2.8f, 1040);
    handle_flight_logic(&data3);
    TEST_ASSERT(flight_logic_freefall_detected(), 
                "Third consecutive reading should trigger freefall");
    TEST_ASSERT(mock_state == STATE_FREEFALL, 
                "Should transition to FREEFALL state");
    TEST_ASSERT(flight_logic_drop_time_ms() == 1040, 
                "Drop time should be recorded");
}

void test_freefall_counter_resets_on_normal_accel(void) {
    printf("\n=== Test: Counter resets on normal acceleration ===\n");
    
    flight_logic_init();
    mock_state = STATE_ARMED;
    
    // Two readings below threshold
    SensorData data1 = create_test_data(0.2f, -3.0f, 1000);
    handle_flight_logic(&data1);
    
    SensorData data2 = create_test_data(0.25f, -2.5f, 1020);
    handle_flight_logic(&data2);
    
    // Normal acceleration - should reset counter
    SensorData data3 = create_test_data(1.0f, -2.0f, 1040);
    handle_flight_logic(&data3);
    TEST_ASSERT(!flight_logic_freefall_detected(), 
                "Counter should reset, no freefall");
    
    // Two more readings below threshold
    SensorData data4 = create_test_data(0.2f, -3.0f, 1060);
    handle_flight_logic(&data4);
    
    SensorData data5 = create_test_data(0.25f, -2.5f, 1080);
    handle_flight_logic(&data5);
    TEST_ASSERT(!flight_logic_freefall_detected(), 
                "Still need one more reading after reset");
}

void test_freefall_requires_velocity_condition(void) {
    printf("\n=== Test: Freefall requires velocity < -2.0 m/s ===\n");
    
    flight_logic_init();
    mock_state = STATE_ARMED;
    
    // Three readings with low accel but velocity not negative enough
    SensorData data1 = create_test_data(0.2f, -1.0f, 1000);
    handle_flight_logic(&data1);
    
    SensorData data2 = create_test_data(0.25f, -1.5f, 1020);
    handle_flight_logic(&data2);
    
    SensorData data3 = create_test_data(0.28f, -1.8f, 1040);
    handle_flight_logic(&data3);
    
    TEST_ASSERT(!flight_logic_freefall_detected(), 
                "Should not trigger without velocity < -2.0 m/s");
}

void test_freefall_requires_accel_condition(void) {
    printf("\n=== Test: Freefall requires accel < 0.3g ===\n");
    
    flight_logic_init();
    mock_state = STATE_ARMED;
    
    // Three readings with good velocity but accel too high
    SensorData data1 = create_test_data(0.5f, -3.0f, 1000);
    handle_flight_logic(&data1);
    
    SensorData data2 = create_test_data(0.6f, -2.5f, 1020);
    handle_flight_logic(&data2);
    
    SensorData data3 = create_test_data(0.4f, -2.8f, 1040);
    handle_flight_logic(&data3);
    
    TEST_ASSERT(!flight_logic_freefall_detected(), 
                "Should not trigger without accel < 0.3g");
}

void test_freefall_only_in_armed_state(void) {
    printf("\n=== Test: Freefall detection only in ARMED state ===\n");
    
    flight_logic_init();
    mock_state = STATE_IDLE;
    
    // Three readings that would trigger in ARMED state
    SensorData data1 = create_test_data(0.2f, -3.0f, 1000);
    handle_flight_logic(&data1);
    
    SensorData data2 = create_test_data(0.25f, -2.5f, 1020);
    handle_flight_logic(&data2);
    
    SensorData data3 = create_test_data(0.28f, -2.8f, 1040);
    handle_flight_logic(&data3);
    
    TEST_ASSERT(!flight_logic_freefall_detected(), 
                "Should not detect freefall in IDLE state");
}

void test_freefall_boundary_conditions(void) {
    printf("\n=== Test: Boundary conditions (exactly at thresholds) ===\n");
    
    // Test 1: Exactly at accel threshold (should NOT trigger)
    flight_logic_init();
    mock_state = STATE_ARMED;
    
    for (int i = 0; i < 3; i++) {
        SensorData data = create_test_data(0.3f, -3.0f, 1000 + i * 20);
        handle_flight_logic(&data);
    }
    TEST_ASSERT(!flight_logic_freefall_detected(), 
                "Accel exactly at 0.3g should NOT trigger");
    
    // Test 2: Exactly at velocity threshold (should NOT trigger)
    flight_logic_init();
    mock_state = STATE_ARMED;
    
    for (int i = 0; i < 3; i++) {
        SensorData data = create_test_data(0.2f, -2.0f, 1000 + i * 20);
        handle_flight_logic(&data);
    }
    TEST_ASSERT(!flight_logic_freefall_detected(), 
                "Velocity exactly at -2.0 m/s should NOT trigger");
    
    // Test 3: Just below both thresholds (should trigger)
    flight_logic_init();
    mock_state = STATE_ARMED;
    
    for (int i = 0; i < 3; i++) {
        SensorData data = create_test_data(0.29f, -2.01f, 1000 + i * 20);
        handle_flight_logic(&data);
    }
    TEST_ASSERT(flight_logic_freefall_detected(), 
                "Just below both thresholds should trigger");
}

int main(void) {
    printf("╔════════════════════════════════════════════════════╗\n");
    printf("║   Freefall Detection Unit Tests                   ║\n");
    printf("║   Requirements: 2.1, 2.2, 2.5, 2.6                 ║\n");
    printf("╚════════════════════════════════════════════════════╝\n");
    
    test_freefall_not_detected_single_reading();
    test_freefall_detected_three_consecutive();
    test_freefall_counter_resets_on_normal_accel();
    test_freefall_requires_velocity_condition();
    test_freefall_requires_accel_condition();
    test_freefall_only_in_armed_state();
    test_freefall_boundary_conditions();
    
    printf("\n╔════════════════════════════════════════════════════╗\n");
    printf("║   Results: %d/%d tests passed                      ║\n", tests_passed, tests_run);
    printf("╚════════════════════════════════════════════════════╝\n");
    
    return (tests_passed == tests_run) ? 0 : 1;
}
