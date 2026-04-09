/**
 * @file sensor_fusion.h
 * @brief Sensor fusion for altitude and attitude estimation
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "mpu6050.h"
#include "bme280.h"
#include "filters.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float altitude;
    float velocity;
    float altitude_raw;
    float velocity_raw;
    float altitude_reference;
    float altitude_ref_accum;
    uint16_t altitude_ref_samples;
    bool altitude_zeroed;
    float roll, pitch, yaw;
    uint32_t last_update;
} FusionState;

void fusion_init(FusionState *state);
void fusion_update(FusionState *state, const BME280_Data *baro, const MPU6050_Data *imu);
float fusion_get_altitude(const FusionState *state);
float fusion_get_velocity(const FusionState *state);
void fusion_get_attitude(const FusionState *state, float *roll, float *pitch, float *yaw);
void fusion_reset(FusionState *state);

#endif // SENSOR_FUSION_H
