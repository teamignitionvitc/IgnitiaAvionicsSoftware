/**
 * @file mpu6050.h
 * @brief MPU6050 IMU driver for RP2040
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t accel_x_raw, accel_y_raw, accel_z_raw;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
    int16_t temp_raw;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float temperature;
    float accel_magnitude;
} MPU6050_Data;

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
} MPU6050_Calibration;

bool mpu6050_init(void);
bool mpu6050_is_connected(void);
bool mpu6050_read(MPU6050_Data *data);
bool mpu6050_calibrate(uint16_t samples);
void mpu6050_get_calibration(MPU6050_Calibration *cal);
void mpu6050_set_calibration(const MPU6050_Calibration *cal);
void mpu6050_reset(void);
void mpu6050_sleep(void);
void mpu6050_wake(void);

#endif // MPU6050_H
