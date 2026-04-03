/**
 * @file mpu6050.c
 * @brief MPU6050 IMU driver implementation for RP2040
 */

#include "mpu6050.h"
#include "config.h"
#include "hardware/i2c.h"
#include <math.h>
#include <string.h>

// MPU6050 Registers
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_FIFO_EN         0x23
#define MPU6050_REG_INT_ENABLE      0x38
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75

#define MPU6050_WHO_AM_I_VAL        0x68

static float accel_scale = 16384.0f;
static float gyro_scale = 131.0f;
static MPU6050_Calibration calibration = {0};

static bool write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false) == 2;
}

static bool read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    if (i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true) != 1) {
        return false;
    }
    return i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buf, len, false) == (int)len;
}

bool mpu6050_init(void) {
    uint8_t who_am_i;
    if (!read_regs(MPU6050_REG_WHO_AM_I, &who_am_i, 1) || who_am_i != MPU6050_WHO_AM_I_VAL) {
        return false;
    }
    
    if (!write_reg(MPU6050_REG_PWR_MGMT_1, 0x80)) return false;
    sleep_ms(100);
    
    if (!write_reg(MPU6050_REG_PWR_MGMT_1, 0x01)) return false;
    sleep_ms(10);
    
    uint8_t divider = (1000 / MPU6050_SAMPLE_RATE) - 1;
    if (!write_reg(MPU6050_REG_SMPLRT_DIV, divider)) return false;
    if (!write_reg(MPU6050_REG_CONFIG, 0x03)) return false;
    if (!write_reg(MPU6050_REG_GYRO_CONFIG, 0x00)) return false;
    if (!write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00)) return false;
    if (!write_reg(MPU6050_REG_FIFO_EN, 0x00)) return false;
    if (!write_reg(MPU6050_REG_INT_ENABLE, 0x01)) return false;
    
    return true;
}

bool mpu6050_is_connected(void) {
    uint8_t who_am_i;
    if (!read_regs(MPU6050_REG_WHO_AM_I, &who_am_i, 1)) return false;
    return who_am_i == MPU6050_WHO_AM_I_VAL;
}

bool mpu6050_read(MPU6050_Data *data) {
    if (!data) return false;
    
    uint8_t buf[14];
    if (!read_regs(MPU6050_REG_ACCEL_XOUT_H, buf, 14)) return false;
    
    data->accel_x_raw = (int16_t)((buf[0] << 8) | buf[1]) - calibration.accel_x;
    data->accel_y_raw = (int16_t)((buf[2] << 8) | buf[3]) - calibration.accel_y;
    data->accel_z_raw = (int16_t)((buf[4] << 8) | buf[5]) - calibration.accel_z;
    data->temp_raw    = (int16_t)((buf[6] << 8) | buf[7]);
    data->gyro_x_raw  = (int16_t)((buf[8] << 8) | buf[9]) - calibration.gyro_x;
    data->gyro_y_raw  = (int16_t)((buf[10] << 8) | buf[11]) - calibration.gyro_y;
    data->gyro_z_raw  = (int16_t)((buf[12] << 8) | buf[13]) - calibration.gyro_z;
    
    data->accel_x = data->accel_x_raw / accel_scale;
    data->accel_y = data->accel_y_raw / accel_scale;
    data->accel_z = data->accel_z_raw / accel_scale;
    data->gyro_x = data->gyro_x_raw / gyro_scale;
    data->gyro_y = data->gyro_y_raw / gyro_scale;
    data->gyro_z = data->gyro_z_raw / gyro_scale;
    data->temperature = (data->temp_raw / 340.0f) + 36.53f;
    
    data->accel_magnitude = sqrtf(
        data->accel_x * data->accel_x +
        data->accel_y * data->accel_y +
        data->accel_z * data->accel_z
    );
    
    return true;
}

bool mpu6050_calibrate(uint16_t samples) {
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    
    memset(&calibration, 0, sizeof(calibration));
    
    uint8_t buf[14];
    for (uint16_t i = 0; i < samples; i++) {
        if (!read_regs(MPU6050_REG_ACCEL_XOUT_H, buf, 14)) return false;
        
        ax_sum += (int16_t)((buf[0] << 8) | buf[1]);
        ay_sum += (int16_t)((buf[2] << 8) | buf[3]);
        az_sum += (int16_t)((buf[4] << 8) | buf[5]);
        gx_sum += (int16_t)((buf[8] << 8) | buf[9]);
        gy_sum += (int16_t)((buf[10] << 8) | buf[11]);
        gz_sum += (int16_t)((buf[12] << 8) | buf[13]);
        
        sleep_ms(10);
    }
    
    calibration.accel_x = ax_sum / samples;
    calibration.accel_y = ay_sum / samples;
    calibration.accel_z = (az_sum / samples) - (int16_t)accel_scale;
    calibration.gyro_x = gx_sum / samples;
    calibration.gyro_y = gy_sum / samples;
    calibration.gyro_z = gz_sum / samples;
    
    return true;
}

void mpu6050_get_calibration(MPU6050_Calibration *cal) {
    if (cal) *cal = calibration;
}

void mpu6050_set_calibration(const MPU6050_Calibration *cal) {
    if (cal) calibration = *cal;
}

void mpu6050_reset(void) {
    write_reg(MPU6050_REG_PWR_MGMT_1, 0x80);
    sleep_ms(100);
}

void mpu6050_sleep(void) {
    uint8_t val;
    read_regs(MPU6050_REG_PWR_MGMT_1, &val, 1);
    write_reg(MPU6050_REG_PWR_MGMT_1, val | 0x40);
}

void mpu6050_wake(void) {
    uint8_t val;
    read_regs(MPU6050_REG_PWR_MGMT_1, &val, 1);
    write_reg(MPU6050_REG_PWR_MGMT_1, val & ~0x40);
}
