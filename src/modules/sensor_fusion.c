/**
 * @file sensor_fusion.c
 * @brief Sensor filtering and fusion implementation.
 */

#include "sensor_fusion.h"
#include "filters.h"
#include <math.h>

#define ALTITUDE_ALPHA              0.15f
#define ALTITUDE_SPIKE_MAX_DELTA    20.0f
#define MAX_REASONABLE_VELOCITY     100.0f

// IMPROVEMENT (Issue 2): Adaptive filtering based on noise detection
#define NOISE_VARIANCE_THRESHOLD    5.0f   // m² - variance threshold for high noise
#define NOISE_WINDOW_SIZE           10     // samples for variance calculation
#define ALPHA_NORMAL                0.7f   // 70% accel + 30% baro (normal conditions)
#define ALPHA_NOISY                 0.5f   // 50% accel + 50% baro (high noise)

static float prev_filtered_altitude = 0.0f;
static float prev_fused_velocity = 0.0f;
static bool initialized = false;

// Noise detection state
static float altitude_history[NOISE_WINDOW_SIZE] = {0};
static uint8_t altitude_history_index = 0;
static bool altitude_history_full = false;

void fusion_init(void) {
    prev_filtered_altitude = 0.0f;
    prev_fused_velocity = 0.0f;
    initialized = false;
    
    // Reset noise detection
    altitude_history_index = 0;
    altitude_history_full = false;
    for (uint8_t i = 0; i < NOISE_WINDOW_SIZE; i++) {
        altitude_history[i] = 0.0f;
    }
}

// IMPROVEMENT (Issue 2): Detect high noise conditions for adaptive filtering
static bool detect_high_noise(void) {
    if (!altitude_history_full) {
        return false;  // Not enough data yet
    }
    
    // Calculate mean
    float mean = 0.0f;
    for (uint8_t i = 0; i < NOISE_WINDOW_SIZE; i++) {
        mean += altitude_history[i];
    }
    mean /= NOISE_WINDOW_SIZE;
    
    // Calculate variance
    float variance = 0.0f;
    for (uint8_t i = 0; i < NOISE_WINDOW_SIZE; i++) {
        float diff = altitude_history[i] - mean;
        variance += diff * diff;
    }
    variance /= NOISE_WINDOW_SIZE;
    
    // High noise if variance exceeds threshold
    return (variance > NOISE_VARIANCE_THRESHOLD);
}

void filter_data(SensorData *data) {
    if (!data) {
        return;
    }

    if (!initialized) {
        prev_filtered_altitude = data->baro_altitude_m;
        data->filtered_altitude_m = data->baro_altitude_m;
        data->baro_velocity_mps = 0.0f;
        data->altitude_spike_rejected = false;
        initialized = true;
        return;
    }

    float altitude_checked = prev_filtered_altitude;
    bool accepted = spike_reject(data->baro_altitude_m,
                                 prev_filtered_altitude,
                                 ALTITUDE_SPIKE_MAX_DELTA,
                                 &altitude_checked);

    data->altitude_spike_rejected = !accepted;
    data->filtered_altitude_m = low_pass(altitude_checked, prev_filtered_altitude, ALTITUDE_ALPHA);
    
    // IMPROVEMENT (Small but Powerful): Clamp altitude to reasonable bounds [0, 5000m]
    data->filtered_altitude_m = clamp_float(data->filtered_altitude_m, 0.0f, 5000.0f);

    // IMPROVEMENT (Issue 2): Update altitude history for noise detection
    altitude_history[altitude_history_index] = data->filtered_altitude_m;
    altitude_history_index = (altitude_history_index + 1) % NOISE_WINDOW_SIZE;
    if (altitude_history_index == 0) {
        altitude_history_full = true;
    }

    float dt = data->dt_s > 0.0f ? data->dt_s : 0.02f;
    data->baro_velocity_mps = (data->filtered_altitude_m - prev_filtered_altitude) / dt;

    // Velocity sanity check: clamp to ±100 m/s and ensure finite
    // Requirements: 3.4, 3.5
    if (data->baro_velocity_mps > MAX_REASONABLE_VELOCITY) {
        data->baro_velocity_mps = MAX_REASONABLE_VELOCITY;
    } else if (data->baro_velocity_mps < -MAX_REASONABLE_VELOCITY) {
        data->baro_velocity_mps = -MAX_REASONABLE_VELOCITY;
    }
    
    // Ensure velocity is finite (not NaN or infinity)
    if (!isfinite(data->baro_velocity_mps)) {
        data->baro_velocity_mps = 0.0f;
    }

    prev_filtered_altitude = data->filtered_altitude_m;
}

void fuse_sensors(SensorData *data) {
    if (!data) {
        return;
    }

    float dt = data->dt_s > 0.0f ? data->dt_s : 0.02f;

    // IMPROVEMENT (Issue 2): Adaptive complementary filter based on noise detection
    // Normal: 70% accel + 30% baro (prevents drift accumulation)
    // High noise: 50% accel + 50% baro (more stable)
    bool high_noise = detect_high_noise();
    data->high_noise = high_noise;
    float alpha = high_noise ? ALPHA_NOISY : ALPHA_NORMAL;

    // Complementary velocity fusion with adaptive weighting
    float accel_mps2 = (data->accel_z_g - 1.0f) * 9.80665f;
    float accel_integrated_velocity = prev_fused_velocity + accel_mps2 * dt;

    data->fused_velocity_mps = alpha * accel_integrated_velocity + (1.0f - alpha) * data->baro_velocity_mps;
    
    // Velocity sanity check: clamp to ±100 m/s and ensure finite
    // Requirements: 3.4, 3.5
    if (data->fused_velocity_mps > MAX_REASONABLE_VELOCITY) {
        data->fused_velocity_mps = MAX_REASONABLE_VELOCITY;
    } else if (data->fused_velocity_mps < -MAX_REASONABLE_VELOCITY) {
        data->fused_velocity_mps = -MAX_REASONABLE_VELOCITY;
    }
    
    // Ensure velocity is finite (not NaN or infinity)
    if (!isfinite(data->fused_velocity_mps)) {
        data->fused_velocity_mps = 0.0f;
    }
    
    prev_fused_velocity = data->fused_velocity_mps;
    
    // CRITICAL FIX (Issue 1): Convert to fixed-point cm/s for deterministic timing
    // Range: ±327.68 m/s (±32768 cm/s) with 1 cm/s precision
    // Clamp to ±50 m/s (±5000 cm/s) for safety
    int32_t velocity_cmps_temp = (int32_t)(data->fused_velocity_mps * 100.0f);
    if (velocity_cmps_temp > 5000) {
        velocity_cmps_temp = 5000;
    } else if (velocity_cmps_temp < -5000) {
        velocity_cmps_temp = -5000;
    }
    data->velocity_cmps = (int16_t)velocity_cmps_temp;
}

void fusion_reset(void) {
    fusion_init();
}
