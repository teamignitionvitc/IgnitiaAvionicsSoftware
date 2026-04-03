/**
 * @file test_filters.c
 * @brief Unit tests for digital filters
 */

#include <stdio.h>
#include <math.h>

static int tests_run = 0;
static int tests_passed = 0;

#define TEST_ASSERT(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); } \
} while(0)

#define EPSILON 0.001f

// Low-pass filter inline
float low_pass_filter(float prev, float current, float alpha) {
    return prev + alpha * (current - prev);
}

// Kalman filter
typedef struct {
    float x;
    float p;
    float q;
    float r;
} KalmanFilter1D;

void kalman_init(KalmanFilter1D *kf, float q, float r, float initial) {
    kf->x = initial;
    kf->p = 1.0f;
    kf->q = q;
    kf->r = r;
}

float kalman_update(KalmanFilter1D *kf, float measurement) {
    kf->p = kf->p + kf->q;
    float k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + k * (measurement - kf->x);
    kf->p = (1.0f - k) * kf->p;
    return kf->x;
}

// Moving average
typedef struct {
    float buffer[10];
    int size;
    int index;
    float sum;
} MovingAverage;

void moving_avg_init(MovingAverage *ma, int size) {
    ma->size = size;
    ma->index = 0;
    ma->sum = 0;
    for (int i = 0; i < 10; i++) ma->buffer[i] = 0;
}

float moving_avg_update(MovingAverage *ma, float value) {
    ma->sum -= ma->buffer[ma->index];
    ma->buffer[ma->index] = value;
    ma->sum += value;
    ma->index = (ma->index + 1) % ma->size;
    return ma->sum / ma->size;
}

void test_low_pass_filter(void) {
    printf("Testing low-pass filter...\n");
    
    float prev = 0.0f;
    float alpha = 0.5f;
    
    // Step response
    float result = low_pass_filter(prev, 1.0f, alpha);
    TEST_ASSERT(fabsf(result - 0.5f) < EPSILON, "Step response at 50%");
    
    result = low_pass_filter(result, 1.0f, alpha);
    TEST_ASSERT(fabsf(result - 0.75f) < EPSILON, "Step response at 75%");
    
    // No change
    result = low_pass_filter(1.0f, 1.0f, alpha);
    TEST_ASSERT(fabsf(result - 1.0f) < EPSILON, "Steady state");
    
    // Alpha = 1 passes through
    result = low_pass_filter(0.0f, 5.0f, 1.0f);
    TEST_ASSERT(fabsf(result - 5.0f) < EPSILON, "Alpha=1 passthrough");
    
    // Alpha = 0 holds previous
    result = low_pass_filter(3.0f, 10.0f, 0.0f);
    TEST_ASSERT(fabsf(result - 3.0f) < EPSILON, "Alpha=0 holds value");
}

void test_kalman_filter(void) {
    printf("\nTesting Kalman filter...\n");
    
    KalmanFilter1D kf;
    kalman_init(&kf, 0.1f, 1.0f, 0.0f);
    
    TEST_ASSERT(fabsf(kf.x - 0.0f) < EPSILON, "Initial state is 0");
    TEST_ASSERT(fabsf(kf.p - 1.0f) < EPSILON, "Initial covariance is 1");
    
    // Update with measurement
    float result = kalman_update(&kf, 10.0f);
    TEST_ASSERT(result > 0 && result < 10.0f, "Moves toward measurement");
    
    // Multiple updates converge
    for (int i = 0; i < 20; i++) {
        result = kalman_update(&kf, 10.0f);
    }
    TEST_ASSERT(fabsf(result - 10.0f) < 1.0f, "Converges to measurement");
    
    // Noisy measurements
    kalman_init(&kf, 0.01f, 10.0f, 50.0f);
    float measurements[] = {52, 48, 51, 49, 50, 53, 47, 50};
    for (int i = 0; i < 8; i++) {
        result = kalman_update(&kf, measurements[i]);
    }
    TEST_ASSERT(result > 45.0f && result < 55.0f, "Filters noise");
}

void test_moving_average(void) {
    printf("\nTesting moving average...\n");
    
    MovingAverage ma;
    moving_avg_init(&ma, 4);
    
    float result;
    result = moving_avg_update(&ma, 4.0f);
    result = moving_avg_update(&ma, 4.0f);
    result = moving_avg_update(&ma, 4.0f);
    result = moving_avg_update(&ma, 4.0f);
    TEST_ASSERT(fabsf(result - 4.0f) < EPSILON, "Constant input = constant output");
    
    moving_avg_init(&ma, 4);
    result = moving_avg_update(&ma, 1.0f);
    result = moving_avg_update(&ma, 2.0f);
    result = moving_avg_update(&ma, 3.0f);
    result = moving_avg_update(&ma, 4.0f);
    TEST_ASSERT(fabsf(result - 2.5f) < EPSILON, "Average of 1,2,3,4 = 2.5");
}

int main(void) {
    printf("=== Filter Tests ===\n\n");
    
    test_low_pass_filter();
    test_kalman_filter();
    test_moving_average();
    
    printf("\n=== Results: %d/%d tests passed ===\n", tests_passed, tests_run);
    
    return (tests_passed == tests_run) ? 0 : 1;
}
