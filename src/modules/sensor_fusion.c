/**
 * @file sensor_fusion.c
 * @brief Sensor fusion implementation with Kalman filtering
 */

#include "sensor_fusion.h"
#include "config.h"
#include "filters.h"
#include "pico/stdlib.h"
#include <math.h>
#include <string.h>

void fusion_init(FusionState *state) {
    memset(state, 0, sizeof(FusionState));
    state->last_update = to_ms_since_boot(get_absolute_time());
}

void fusion_update(FusionState *state, const BME280_Data *baro, const MPU6050_Data *imu) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    float dt = (now - state->last_update) / 1000.0f;
    state->last_update = now;
    
    if (dt <= 0 || dt > 1.0f) {
        dt = 0.01f;
    }
    
    // Altitude from barometer
    if (baro) {
        float new_altitude = baro->altitude;
        state->altitude_raw = new_altitude;
        
        // Low-pass filter altitude
        state->altitude = low_pass_filter(state->altitude, new_altitude, ALTITUDE_FILTER_ALPHA);
        
        // Estimate velocity from altitude change
        float raw_velocity = (new_altitude - state->altitude) / dt;
        state->velocity_raw = raw_velocity;
        state->velocity = low_pass_filter(state->velocity, raw_velocity, VELOCITY_FILTER_ALPHA);
    }
    
    // Attitude from IMU (complementary filter)
    if (imu) {
        // Accelerometer-based angles
        float accel_roll = atan2f(imu->accel_y, imu->accel_z) * 180.0f / 3.14159f;
        float accel_pitch = atan2f(-imu->accel_x, 
            sqrtf(imu->accel_y * imu->accel_y + imu->accel_z * imu->accel_z)) * 180.0f / 3.14159f;
        
        // Gyroscope integration
        float gyro_roll = state->roll + imu->gyro_x * dt;
        float gyro_pitch = state->pitch + imu->gyro_y * dt;
        float gyro_yaw = state->yaw + imu->gyro_z * dt;
        
        // Complementary filter
        state->roll = COMPLEMENTARY_FILTER_ALPHA * gyro_roll + 
                     (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accel_roll;
        state->pitch = COMPLEMENTARY_FILTER_ALPHA * gyro_pitch + 
                      (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accel_pitch;
        state->yaw = gyro_yaw;  // No magnetometer correction
        
        // Normalize yaw
        while (state->yaw > 180.0f) state->yaw -= 360.0f;
        while (state->yaw < -180.0f) state->yaw += 360.0f;
    }
}

float fusion_get_altitude(const FusionState *state) {
    return state->altitude;
}

float fusion_get_velocity(const FusionState *state) {
    return state->velocity;
}

void fusion_get_attitude(const FusionState *state, float *roll, float *pitch, float *yaw) {
    if (roll) *roll = state->roll;
    if (pitch) *pitch = state->pitch;
    if (yaw) *yaw = state->yaw;
}

void fusion_reset(FusionState *state) {
    memset(state, 0, sizeof(FusionState));
    state->last_update = to_ms_since_boot(get_absolute_time());
}
