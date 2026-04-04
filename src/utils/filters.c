/**
 * @file filters.c
 * @brief Digital filter implementations
 */

#include "filters.h"
#include <math.h>
#include <string.h>

float low_pass(float input, float prev, float alpha) {
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    return alpha * input + (1.0f - alpha) * prev;
}

bool spike_reject(float input, float prev, float max_delta, float *output) {
    if (!output || max_delta < 0.0f) {
        return false;
    }

    if (fabsf(input - prev) > max_delta) {
        *output = prev;
        return false;
    }

    *output = input;
    return true;
}

void moving_avg_init(MovingAverage *ma, float *buffer, uint16_t size) {
    if (!ma || !buffer || size == 0) return;

    ma->buffer = buffer;
    ma->size = size;
    ma->index = 0;
    ma->sum = 0;
    memset(buffer, 0, size * sizeof(float));
}

float moving_avg_update(MovingAverage *ma, float value) {
    if (!ma || !ma->buffer || ma->size == 0) return value;

    ma->sum -= ma->buffer[ma->index];
    ma->buffer[ma->index] = value;
    ma->sum += value;
    ma->index = (ma->index + 1) % ma->size;
    return ma->sum / ma->size;
}

float moving_avg_get(const MovingAverage *ma) {
    if (!ma || ma->size == 0) return 0.0f;
    return ma->sum / ma->size;
}

void moving_avg_reset(MovingAverage *ma) {
    if (!ma || !ma->buffer || ma->size == 0) return;

    ma->index = 0;
    ma->sum = 0;
    memset(ma->buffer, 0, ma->size * sizeof(float));
}

void kalman_init(KalmanFilter1D *kf, float q, float r, float initial) {
    if (!kf) return;

    kf->x = initial;
    kf->p = 1.0f;
    kf->q = q;
    kf->r = r;
}

float kalman_update(KalmanFilter1D *kf, float measurement) {
    if (!kf) return measurement;

    // Prediction
    kf->p = kf->p + kf->q;
    
    // Update
    float k = kf->p / (kf->p + kf->r);  // Kalman gain
    kf->x = kf->x + k * (measurement - kf->x);
    kf->p = (1.0f - k) * kf->p;
    
    return kf->x;
}

// CRITICAL FIX (Issue 1 + Small but Powerful): Clamping utility functions
float clamp_float(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

int16_t clamp_int16(int16_t value, int16_t min_val, int16_t max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}
