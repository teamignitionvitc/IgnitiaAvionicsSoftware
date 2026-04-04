/**
 * @file test_velocity_calculation.c
 * @brief Unit tests for velocity calculation in sensor_fusion module
 */

#include <stdio.h>
#include <math.h>
#include "../src/modules/sensor_fusion.h"
#include "../src/modules/flight_data.h"

// Simple test framework
static int tests_run = 0;
static int tests_passed = 0;

#define TEST_ASSERT(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); } \
} while(0)

#define EPSILON 0.001f

void test_velocity_calculation_basic(void) {
    printf("Testing basic velocity calculation...\n");
    
    fusion_init();
    
    SensorData data = {0};
    data.timestamp_ms = 0;
    data.dt_s = 0.02f;  // 50 Hz
    data.baro_altitude_m = 100.0f;
    data.accel_z_g = 1.0f;
    
    // First call - initializes
    filter_data(&data);
    TEST_ASSERT(data.baro_velocity_mps == 0.0f, "Initial velocity is zero");
    
    // Second call - altitude increases by 1m
    data.timestamp_ms = 20;
    data.baro_altitude_m = 101.0f;
    filter_data(&data);
    
    // Expected velocity: (101 - 100) / 0.02 = 50 m/s (but filtered)
    // Due to low-pass filter, actual value will be less
    TEST_ASSERT(data.baro_velocity_mps > 0.0f, "Positive velocity for ascending");
    TEST_ASSERT(data.baro_velocity_mps < 100.0f, "Velocity within reasonable bounds");
}

void test_velocity_sanity_check_positive(void) {
    printf("\nTesting velocity sanity check (positive)...\n");
    
    fusion_init();
    
    SensorData data = {0};
    data.timestamp_ms = 0;
    data.dt_s = 0.02f;
    data.baro_altitude_m = 100.0f;
    data.accel_z_g = 1.0f;
    
    filter_data(&data);
    
    // Huge altitude jump that would cause velocity > 100 m/s
    data.timestamp_ms = 20;
    data.baro_altitude_m = 200.0f;  // 100m jump in 0.02s = 5000 m/s
    filter_data(&data);
    
    TEST_ASSERT(data.baro_velocity_mps <= 100.0f, "Velocity clamped to +100 m/s");
}

void test_velocity_sanity_check_negative(void) {
    printf("\nTesting velocity sanity check (negative)...\n");
    
    fusion_init();
    
    SensorData data = {0};
    data.timestamp_ms = 0;
    data.dt_s = 0.02f;
    data.baro_altitude_m = 200.0f;
    data.accel_z_g = 1.0f;
    
    filter_data(&data);
    
    // Huge altitude drop that would cause velocity < -100 m/s
    data.timestamp_ms = 20;
    data.baro_altitude_m = 100.0f;  // 100m drop in 0.02s = -5000 m/s
    filter_data(&data);
    
    TEST_ASSERT(data.baro_velocity_mps >= -100.0f, "Velocity clamped to -100 m/s");
}

void test_velocity_sign_correctness(void) {
    printf("\nTesting velocity sign correctness...\n");
    
    fusion_init();
    
    SensorData data = {0};
    data.timestamp_ms = 0;
    data.dt_s = 0.02f;
    data.baro_altitude_m = 100.0f;
    data.accel_z_g = 1.0f;
    
    filter_data(&data);
    
    // Ascending
    data.timestamp_ms = 20;
    data.baro_altitude_m = 101.0f;
    filter_data(&data);
    TEST_ASSERT(data.baro_velocity_mps > 0.0f, "Positive velocity when ascending");
    
    // Descending
    data.timestamp_ms = 40;
    data.baro_altitude_m = 100.0f;
    filter_data(&data);
    TEST_ASSERT(data.baro_velocity_mps < 0.0f, "Negative velocity when descending");
}

void test_fused_velocity_sanity_check(void) {
    printf("\nTesting fused velocity sanity check...\n");
    
    fusion_init();
    
    SensorData data = {0};
    data.timestamp_ms = 0;
    data.dt_s = 0.02f;
    data.baro_altitude_m = 100.0f;
    data.accel_z_g = 1.0f;
    data.baro_velocity_mps = 0.0f;
    
    fuse_sensors(&data);
    
    // Set extreme baro velocity
    data.baro_velocity_mps = 150.0f;  // Above limit
    data.accel_z_g = 10.0f;  // High acceleration
    fuse_sensors(&data);
    
    TEST_ASSERT(data.fused_velocity_mps <= 100.0f, "Fused velocity clamped to +100 m/s");
    TEST_ASSERT(data.fused_velocity_mps >= -100.0f, "Fused velocity clamped to -100 m/s");
}

void test_velocity_finiteness(void) {
    printf("\nTesting velocity finiteness...\n");
    
    fusion_init();
    
    SensorData data = {0};
    data.timestamp_ms = 0;
    data.dt_s = 0.0f;  // Zero dt could cause division by zero
    data.baro_altitude_m = 100.0f;
    data.accel_z_g = 1.0f;
    
    filter_data(&data);
    
    // Should handle zero dt gracefully (uses default 0.02)
    TEST_ASSERT(isfinite(data.baro_velocity_mps), "Velocity is finite with zero dt");
    
    fuse_sensors(&data);
    TEST_ASSERT(isfinite(data.fused_velocity_mps), "Fused velocity is finite");
}

void test_previous_altitude_storage(void) {
    printf("\nTesting previous altitude storage...\n");
    
    fusion_init();
    
    SensorData data = {0};
    data.timestamp_ms = 0;
    data.dt_s = 0.02f;
    data.baro_altitude_m = 100.0f;
    data.accel_z_g = 1.0f;
    
    filter_data(&data);
    float first_filtered = data.filtered_altitude_m;
    
    // Second call with same altitude
    data.timestamp_ms = 20;
    data.baro_altitude_m = 100.0f;
    filter_data(&data);
    
    // Velocity should be near zero since altitude didn't change
    TEST_ASSERT(fabs(data.baro_velocity_mps) < 1.0f, "Near-zero velocity for constant altitude");
}

int main(void) {
    printf("=== Velocity Calculation Tests ===\n\n");
    
    test_velocity_calculation_basic();
    test_velocity_sanity_check_positive();
    test_velocity_sanity_check_negative();
    test_velocity_sign_correctness();
    test_fused_velocity_sanity_check();
    test_velocity_finiteness();
    test_previous_altitude_storage();
    
    printf("\n=== Results: %d/%d tests passed ===\n", tests_passed, tests_run);
    
    return (tests_passed == tests_run) ? 0 : 1;
}
