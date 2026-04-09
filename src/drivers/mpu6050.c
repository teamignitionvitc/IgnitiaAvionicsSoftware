/**
 * @file mpu6050.c
 * @brief MPU6050 IMU driver implementation for RP2040
 */

#include "mpu6050.h"
#include "config.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <math.h>
#include <stdio.h>
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

// WHO_AM_I values for supported MPU variants
#define MPU6050_WHO_AM_I_VAL        0x68    // MPU6050
#define MPU6500_WHO_AM_I_VAL        0x70    // MPU6500
#define MPU9250_WHO_AM_I_VAL        0x71    // MPU9250
#define MPU9255_WHO_AM_I_VAL        0x73    // MPU9255
#define ICM20600_WHO_AM_I_VAL       0x11    // ICM20600
#define ICM20602_WHO_AM_I_VAL       0x12    // ICM20602
#define ICM20608_WHO_AM_I_VAL       0xAF    // ICM20608

#define MPU6050_ALT_ADDR            0x69
#define I2C_TIMEOUT_US              25000   // Increased for better tolerance
#define I2C_RETRY_COUNT             5       // More retries for reliability
#define I2C_INTER_OP_DELAY_US       100     // Delay between I2C operations

static float accel_scale = 16384.0f;
static float gyro_scale = 131.0f;
static MPU6050_Calibration calibration = {0};
static uint8_t mpu_addr = MPU6050_ADDR;
static uint8_t detected_who_am_i = 0;

// Sensor type names for debug output
static const char* get_mpu_type_name(uint8_t who_am_i) {
    switch (who_am_i) {
        case MPU6050_WHO_AM_I_VAL: return "MPU6050";
        case MPU6500_WHO_AM_I_VAL: return "MPU6500";
        case MPU9250_WHO_AM_I_VAL: return "MPU9250";
        case MPU9255_WHO_AM_I_VAL: return "MPU9255";
        case ICM20600_WHO_AM_I_VAL: return "ICM20600";
        case ICM20602_WHO_AM_I_VAL: return "ICM20602";
        case ICM20608_WHO_AM_I_VAL: return "ICM20608";
        default: return "Unknown";
    }
}

static bool is_supported_who_am_i(uint8_t who_am_i) {
    switch (who_am_i) {
        case MPU6050_WHO_AM_I_VAL:   // 0x68 - MPU6050
        case MPU6500_WHO_AM_I_VAL:   // 0x70 - MPU6500
        case MPU9250_WHO_AM_I_VAL:   // 0x71 - MPU9250
        case MPU9255_WHO_AM_I_VAL:   // 0x73 - MPU9255
        case ICM20600_WHO_AM_I_VAL:  // 0x11 - ICM20600
        case ICM20602_WHO_AM_I_VAL:  // 0x12 - ICM20602
        case ICM20608_WHO_AM_I_VAL:  // 0xAF - ICM20608
            return true;
        default:
            return false;
    }
}

static bool write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    for (int attempt = 0; attempt < I2C_RETRY_COUNT; attempt++) {
        if (i2c_write_timeout_us(I2C_PORT, mpu_addr, buf, 2, false, I2C_TIMEOUT_US) == 2) {
            sleep_us(I2C_INTER_OP_DELAY_US);
            return true;
        }
        sleep_us(200);  // Longer delay on retry
    }
    return false;
}

static bool read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    for (int attempt = 0; attempt < I2C_RETRY_COUNT; attempt++) {
        // Preferred path: repeated-start register read (matches MicroPython writeto_then_readfrom).
        int w_rc = i2c_write_timeout_us(I2C_PORT, mpu_addr, &reg, 1, true, I2C_TIMEOUT_US);
        if (w_rc == 1) {
            sleep_us(50);  // Small delay before read like MicroPython
            int r_rc = i2c_read_timeout_us(I2C_PORT, mpu_addr, buf, len, false, I2C_TIMEOUT_US);
            if (r_rc == (int)len) {
                sleep_us(I2C_INTER_OP_DELAY_US);
                return true;
            }
        }

        sleep_us(100);

        // Fallback path: STOP between register write and read.
        w_rc = i2c_write_timeout_us(I2C_PORT, mpu_addr, &reg, 1, false, I2C_TIMEOUT_US);
        if (w_rc == 1) {
            sleep_us(100);  // Longer delay for STOP-based path
            int r_rc = i2c_read_timeout_us(I2C_PORT, mpu_addr, buf, len, false, I2C_TIMEOUT_US);
            if (r_rc == (int)len) {
                sleep_us(I2C_INTER_OP_DELAY_US);
                return true;
            }
        }

        sleep_us(200);  // Longer delay between retry attempts
    }
    return false;
}

static bool read_word_reg(uint8_t reg, int16_t *value) {
    uint8_t buf[2];
    if (!value) return false;

    if (!read_regs(reg, buf, 2)) {
        return false;
    }

    *value = (int16_t)((buf[0] << 8) | buf[1]);
    return true;
}

static bool probe_addr(uint8_t addr, uint8_t *who_am_i) {
    uint8_t reg = MPU6050_REG_WHO_AM_I;
    int w_rc = 0;
    int r_rc = 0;

    // Give device time to stabilize before probing
    sleep_ms(5);

    for (int attempt = 0; attempt < I2C_RETRY_COUNT; attempt++) {
        // Preferred path: repeated-start (MicroPython style)
        w_rc = i2c_write_timeout_us(I2C_PORT, addr, &reg, 1, true, I2C_TIMEOUT_US);
        if (w_rc == 1) {
            sleep_us(50);  // Critical: delay before read like MicroPython
            r_rc = i2c_read_timeout_us(I2C_PORT, addr, who_am_i, 1, false, I2C_TIMEOUT_US);
            if (r_rc == 1) {
                return true;
            }
        }

        sleep_us(200);

        // Fallback path: STOP between write and read
        w_rc = i2c_write_timeout_us(I2C_PORT, addr, &reg, 1, false, I2C_TIMEOUT_US);
        if (w_rc == 1) {
            sleep_us(100);
            r_rc = i2c_read_timeout_us(I2C_PORT, addr, who_am_i, 1, false, I2C_TIMEOUT_US);
            if (r_rc == 1) {
                return true;
            }
        }

        sleep_us(200);
    }

#if DEBUG_ENABLE
    printf("MPU probe 0x%02X failed (w_rc=%d r_rc=%d)\r\n", addr, w_rc, r_rc);
#endif
    return false;
}

bool mpu6050_init(void) {
    uint8_t who_am_i = 0;
    const uint8_t addrs[] = {MPU6050_ADDR, MPU6050_ALT_ADDR};

    // Longer initial delay for device power-up stabilization
    sleep_ms(100);

#if DEBUG_ENABLE
    printf("MPU init: scanning for IMU devices...\r\n");
#endif

    // Some breakout boards strap AD0 high (0x69), others use 0x68.
    bool found = false;
    for (size_t i = 0; i < sizeof(addrs); i++) {
        for (int attempt = 0; attempt < 10; attempt++) {
            if (probe_addr(addrs[i], &who_am_i)) {
#if DEBUG_ENABLE
                printf("  Addr 0x%02X: WHO_AM_I=0x%02X (%s)\r\n", 
                       addrs[i], who_am_i, get_mpu_type_name(who_am_i));
#endif
                if (is_supported_who_am_i(who_am_i)) {
                    mpu_addr = addrs[i];
                    detected_who_am_i = who_am_i;
                    found = true;
                    break;
                }
            }
            sleep_ms(50);
        }
        if (found) break;
    }

    if (!found) {
#if DEBUG_ENABLE
        printf("MPU probe FAILED (final WHO_AM_I=0x%02X)\r\n", who_am_i);
        printf("\r\n");
        printf("=== MPU TROUBLESHOOTING ===\r\n");
        printf("Read WHO_AM_I: 0x%02X\r\n", who_am_i);
        printf("Expected:      0x68 (MPU6050), 0x70 (MPU6500), 0x71 (MPU9250)\r\n");
        printf("\r\n");
        if (who_am_i == 0x60) {
            printf(">>> 0x60 = BMP280/BME280 chip ID!\r\n");
            printf(">>> You are reading the BAROMETER, not the IMU!\r\n");
            printf(">>> Check wiring: MPU AD0 pin may be floating.\r\n");
        }
        printf("\r\n");
        printf("Checklist:\r\n");
        printf("  [ ] AD0 pin connected to GND (for 0x68) or 3.3V (for 0x69)\r\n");
        printf("  [ ] I2C pullups on SDA/SCL (4.7k to 3.3V typical)\r\n");
        printf("  [ ] MPU VCC/VIN connected to 3.3V\r\n");
        printf("  [ ] GND connected\r\n");
        printf("  [ ] Wiring: SDA=GP%d, SCL=GP%d\r\n", I2C_SDA_PIN, I2C_SCL_PIN);
        printf("  [ ] No I2C address conflicts\r\n");
        printf("===========================\r\n\r\n");
#endif
        return false;
    }

#if DEBUG_ENABLE
    printf("MPU detected: %s at 0x%02X (WHO_AM_I=0x%02X)\r\n", 
           get_mpu_type_name(detected_who_am_i), mpu_addr, detected_who_am_i);
#endif

    // Match the known-working MicroPython sequence: wake using 0x00.
    sleep_ms(10);  // Small delay before config writes
    if (!write_reg(MPU6050_REG_PWR_MGMT_1, 0x00)) {
#if DEBUG_ENABLE
        printf("MPU init failed: wake write failed\r\n");
#endif
        return false;
    }
    sleep_ms(100);  // Longer wake delay for stability

    // Final connectivity check using a single word read (MicroPython-style register access).
    int16_t ax_probe = 0;
    if (!read_word_reg(MPU6050_REG_ACCEL_XOUT_H, &ax_probe)) {
#if DEBUG_ENABLE
        printf("MPU init failed: accel register read test failed\r\n");
#endif
        return false;
    }

#if DEBUG_ENABLE
    printf("MPU init SUCCESS: accel_x probe = %d\r\n", ax_probe);
#endif

    return true;
}

bool mpu6050_is_connected(void) {
    uint8_t who_am_i;
    if (!read_regs(MPU6050_REG_WHO_AM_I, &who_am_i, 1)) return false;
    return is_supported_who_am_i(who_am_i);
}

bool mpu6050_read(MPU6050_Data *data) {
    if (!data) return false;
    
    uint8_t buf[14];
    if (read_regs(MPU6050_REG_ACCEL_XOUT_H, buf, 14)) {
        data->accel_x_raw = (int16_t)((buf[0] << 8) | buf[1]) - calibration.accel_x;
        data->accel_y_raw = (int16_t)((buf[2] << 8) | buf[3]) - calibration.accel_y;
        data->accel_z_raw = (int16_t)((buf[4] << 8) | buf[5]) - calibration.accel_z;
        data->temp_raw    = (int16_t)((buf[6] << 8) | buf[7]);
        data->gyro_x_raw  = (int16_t)((buf[8] << 8) | buf[9]) - calibration.gyro_x;
        data->gyro_y_raw  = (int16_t)((buf[10] << 8) | buf[11]) - calibration.gyro_y;
        data->gyro_z_raw  = (int16_t)((buf[12] << 8) | buf[13]) - calibration.gyro_z;
    } else {
        // Fallback path: read each 16-bit register pair separately.
        int16_t ax = 0, ay = 0, az = 0, t = 0, gx = 0, gy = 0, gz = 0;
        if (!read_word_reg(0x3B, &ax) ||
            !read_word_reg(0x3D, &ay) ||
            !read_word_reg(0x3F, &az) ||
            !read_word_reg(0x41, &t)  ||
            !read_word_reg(0x43, &gx) ||
            !read_word_reg(0x45, &gy) ||
            !read_word_reg(0x47, &gz)) {
            return false;
        }

        data->accel_x_raw = ax - calibration.accel_x;
        data->accel_y_raw = ay - calibration.accel_y;
        data->accel_z_raw = az - calibration.accel_z;
        data->temp_raw    = t;
        data->gyro_x_raw  = gx - calibration.gyro_x;
        data->gyro_y_raw  = gy - calibration.gyro_y;
        data->gyro_z_raw  = gz - calibration.gyro_z;
    }
    
    data->accel_x = data->accel_x_raw / accel_scale;
    data->accel_y = data->accel_y_raw / accel_scale;
    data->accel_z = data->accel_z_raw / accel_scale;
    data->gyro_x = data->gyro_x_raw / gyro_scale;
    data->gyro_y = data->gyro_y_raw / gyro_scale;
    data->gyro_z = data->gyro_z_raw / gyro_scale;
    data->temperature = (data->temp_raw / 333.87f) + 21.0f;
    
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
    
    for (uint16_t i = 0; i < samples; i++) {
        int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
        if (!read_word_reg(0x3B, &ax) ||
            !read_word_reg(0x3D, &ay) ||
            !read_word_reg(0x3F, &az) ||
            !read_word_reg(0x43, &gx) ||
            !read_word_reg(0x45, &gy) ||
            !read_word_reg(0x47, &gz)) {
            return false;
        }

        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        
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

const char* mpu6050_get_type_name(void) {
    return get_mpu_type_name(detected_who_am_i);
}

uint8_t mpu6050_get_who_am_i(void) {
    return detected_who_am_i;
}
