/**
 * @file filters.h
 * @brief Digital filter implementations
 */

#ifndef FILTERS_H
#define FILTERS_H

#include <stdint.h>
#include <stdbool.h>

float low_pass(float input, float prev, float alpha);
bool spike_reject(float input, float prev, float max_delta, float *output);

// Simple low-pass filter
static inline float low_pass_filter(float prev, float current, float alpha) {
    return low_pass(current, prev, alpha);
}

// High-pass filter (derivative)
static inline float high_pass_filter(float prev_out, float current, float prev_in, float alpha) {
    return alpha * (prev_out + current - prev_in);
}

// Moving average filter state
typedef struct {
    float *buffer;
    uint16_t size;
    uint16_t index;
    float sum;
} MovingAverage;

void moving_avg_init(MovingAverage *ma, float *buffer, uint16_t size);
float moving_avg_update(MovingAverage *ma, float value);
float moving_avg_get(const MovingAverage *ma);
void moving_avg_reset(MovingAverage *ma);

// 1D Kalman filter state
typedef struct {
    float x;    // State estimate
    float p;    // Estimate covariance
    float q;    // Process noise covariance
    float r;    // Measurement noise covariance
} KalmanFilter1D;

void kalman_init(KalmanFilter1D *kf, float q, float r, float initial);
float kalman_update(KalmanFilter1D *kf, float measurement);

// CRITICAL FIX (Issue 1 + Small but Powerful): Clamping utility functions
float clamp_float(float value, float min_val, float max_val);
int16_t clamp_int16(int16_t value, int16_t min_val, int16_t max_val);

#endif // FILTERS_H