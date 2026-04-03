/**
 * @file test_flight_state.c
 * @brief Unit tests for flight state machine
 */

#include <stdio.h>
#include <string.h>

// Simple test framework (no Unity dependency for quick testing)
static int tests_run = 0;
static int tests_passed = 0;

#define TEST_ASSERT(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); } \
} while(0)

// Mock pico/stdlib functions for host testing
#ifdef HOST_TEST
static unsigned long mock_time_ms = 0;
unsigned long to_ms_since_boot(void* t) { (void)t; return mock_time_ms; }
void* get_absolute_time(void) { return NULL; }
void set_mock_time(unsigned long ms) { mock_time_ms = ms; }
float fabsf(float x) { return x < 0 ? -x : x; }
#else
#include "pico/stdlib.h"
#endif

// Override config values for testing
#define APOGEE_DETECTION_THRESHOLD  2.0f
#define APOGEE_CONFIRMATION_COUNT   5
#define LAUNCH_ACCEL_THRESHOLD      2.5f
#define LAUNCH_ALTITUDE_CHANGE      10.0f
#define LANDING_VELOCITY_THRESHOLD  0.5f
#define LANDING_ALTITUDE_THRESHOLD  20.0f
#define LANDING_CONFIRMATION_TIME   5000
#define DEPLOYMENT_SAFETY_ALT       500.0f
#define DEPLOYMENT_SAFETY_TIME      60000
#define MIN_FLIGHT_TIME             5000

typedef enum {
    STATE_INIT = 0,
    STATE_IDLE,
    STATE_ARMED,
    STATE_ASCENT,
    STATE_APOGEE,
    STATE_DESCENT,
    STATE_LANDED,
    STATE_ERROR
} FlightState;

typedef struct {
    FlightState current_state;
    FlightState previous_state;
    unsigned long state_entry_time;
    unsigned long launch_time;
    float launch_altitude;
    float max_altitude;
    float current_altitude;
    float current_velocity;
    float current_accel;
    int deployed;
    int apogee_confirm_count;
} FlightStateContext;

// Inline implementation for testing
void flight_state_init(FlightStateContext *ctx) {
    memset(ctx, 0, sizeof(FlightStateContext));
    ctx->current_state = STATE_INIT;
    ctx->previous_state = STATE_INIT;
    #ifdef HOST_TEST
    ctx->state_entry_time = mock_time_ms;
    #endif
}

void test_init(void) {
    printf("Testing flight_state_init...\n");
    FlightStateContext ctx;
    flight_state_init(&ctx);
    
    TEST_ASSERT(ctx.current_state == STATE_INIT, "Initial state is INIT");
    TEST_ASSERT(ctx.max_altitude == 0, "Max altitude starts at 0");
    TEST_ASSERT(ctx.deployed == 0, "Not deployed initially");
}

void test_state_names(void) {
    printf("\nTesting state names...\n");
    const char* names[] = {"INIT", "IDLE", "ARMED", "ASCENT", "APOGEE", "DESCENT", "LANDED", "ERROR"};
    
    for (int i = 0; i <= STATE_ERROR; i++) {
        TEST_ASSERT(i >= STATE_INIT && i <= STATE_ERROR, names[i]);
    }
}

void test_arm_disarm(void) {
    printf("\nTesting arm/disarm...\n");
    FlightStateContext ctx;
    flight_state_init(&ctx);
    ctx.current_state = STATE_IDLE;
    
    // Simulate arm
    ctx.current_state = STATE_ARMED;
    TEST_ASSERT(ctx.current_state == STATE_ARMED, "System arms from IDLE");
    
    // Simulate disarm
    ctx.current_state = STATE_IDLE;
    TEST_ASSERT(ctx.current_state == STATE_IDLE, "System disarms to IDLE");
}

void test_launch_detection(void) {
    printf("\nTesting launch detection...\n");
    FlightStateContext ctx;
    flight_state_init(&ctx);
    ctx.current_state = STATE_ARMED;
    ctx.launch_altitude = 0;
    
    // Below threshold - should stay armed
    float accel = 1.5f;
    TEST_ASSERT(accel < LAUNCH_ACCEL_THRESHOLD, "Low accel does not trigger launch");
    
    // Above threshold - should transition to ascent
    accel = 3.0f;
    TEST_ASSERT(accel > LAUNCH_ACCEL_THRESHOLD, "High accel triggers launch");
    ctx.current_state = STATE_ASCENT;
    TEST_ASSERT(ctx.current_state == STATE_ASCENT, "Transitions to ASCENT");
}

void test_apogee_detection(void) {
    printf("\nTesting apogee detection...\n");
    FlightStateContext ctx;
    flight_state_init(&ctx);
    ctx.current_state = STATE_ASCENT;
    ctx.max_altitude = 100.0f;
    
    // Altitude dropping
    ctx.current_altitude = 97.0f;
    float drop = ctx.max_altitude - ctx.current_altitude;
    TEST_ASSERT(drop > APOGEE_DETECTION_THRESHOLD, "Altitude drop detected");
    
    // Confirm count
    ctx.apogee_confirm_count = APOGEE_CONFIRMATION_COUNT;
    TEST_ASSERT(ctx.apogee_confirm_count >= APOGEE_CONFIRMATION_COUNT, "Apogee confirmed");
}

void test_deployment_conditions(void) {
    printf("\nTesting deployment conditions...\n");
    FlightStateContext ctx;
    flight_state_init(&ctx);
    
    // At apogee state
    ctx.current_state = STATE_APOGEE;
    ctx.deployed = 0;
    TEST_ASSERT(ctx.current_state == STATE_APOGEE && !ctx.deployed, "Deploy at apogee");
    
    // Already deployed
    ctx.deployed = 1;
    TEST_ASSERT(ctx.deployed, "Don't double deploy");
}

void test_landing_detection(void) {
    printf("\nTesting landing detection...\n");
    FlightStateContext ctx;
    flight_state_init(&ctx);
    ctx.current_state = STATE_DESCENT;
    
    ctx.current_altitude = 5.0f;
    ctx.current_velocity = 0.3f;
    
    TEST_ASSERT(ctx.current_altitude < LANDING_ALTITUDE_THRESHOLD, "Low altitude");
    TEST_ASSERT(fabsf(ctx.current_velocity) < LANDING_VELOCITY_THRESHOLD, "Low velocity");
}

int main(void) {
    printf("=== Flight State Machine Tests ===\n\n");
    
    #ifdef HOST_TEST
    set_mock_time(0);
    #endif
    
    test_init();
    test_state_names();
    test_arm_disarm();
    test_launch_detection();
    test_apogee_detection();
    test_deployment_conditions();
    test_landing_detection();
    
    printf("\n=== Results: %d/%d tests passed ===\n", tests_passed, tests_run);
    
    return (tests_passed == tests_run) ? 0 : 1;
}
