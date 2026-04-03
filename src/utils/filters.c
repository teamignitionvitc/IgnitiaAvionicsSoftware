/**
 * @file filters.c
 * @brief Digital filter implementations
 */

#include "filters.h"
#include <string.h>

void moving_avg_init(MovingAverage *ma, float *buffer, uint16_t size) {
    ma->buffer = buffer;
    ma->size = size;
    ma->index = 0;
    ma->sum = 0;
    memset(buffer, 0, size * sizeof(float));
}

float moving_avg_update(MovingAverage *ma, float value) {
    ma->sum -= ma->buffer[ma->index];
    ma->buffer[ma->index] = value;
    ma->sum += value;
    ma->index = (ma->index + 1) % ma->size;
    return ma->sum / ma->size;
}

float moving_avg_get(const MovingAverage *ma) {
    return ma->sum / ma->size;
}

void moving_avg_reset(MovingAverage *ma) {
    ma->index = 0;
    ma->sum = 0;
    memset(ma->buffer, 0, ma->size * sizeof(float));
}

void kalman_init(KalmanFilter1D *kf, float q, float r, float initial) {
    kf->x = initial;
    kf->p = 1.0f;
    kf->q = q;
    kf->r = r;
}

float kalman_update(KalmanFilter1D *kf, float measurement) {
    // Prediction
    kf->p = kf->p + kf->q;
    
    // Update
    float k = kf->p / (kf->p + kf->r);  // Kalman gain
    kf->x = kf->x + k * (measurement - kf->x);
    kf->p = (1.0f - k) * kf->p;
    
    return kf->x;
}
