/**
 * @file bme280.c
 * @brief BME280 Environmental sensor driver implementation
 */

#include "bme280.h"
#include "config.h"
#include "hardware/i2c.h"
#include <math.h>
#include <stdio.h>

#define BME280_REG_CALIB00      0x88
#define BME280_REG_CALIB26      0xE1
#define BME280_REG_CHIP_ID      0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_PRESS_MSB    0xF7

#define BME280_CHIP_ID          0x60
#define BMP280_CHIP_ID          0x58
#define BME280_ALT_ADDR         0x77
#define BME280_RESET_VAL        0xB6
#define I2C_TIMEOUT_US          20000
#define I2C_RETRY_COUNT         3

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1, dig_H3;
    int16_t  dig_H2, dig_H4, dig_H5;
    int8_t   dig_H6;
} BME280_Calib;

static BME280_Calib calib;
static int32_t t_fine;
static float ground_pressure = GROUND_PRESSURE_DEFAULT;
static uint8_t sensor_chip_id = 0x00;  // Track sensor type (0x58=BMP280, 0x60=BME280)
static uint8_t bme_addr = BME280_ADDR;

static bool probe_addr(uint8_t addr, uint8_t *chip_id) {
    uint8_t reg = BME280_REG_CHIP_ID;
    int w_rc = 0;
    int r_rc = 0;

    for (int attempt = 0; attempt < I2C_RETRY_COUNT; attempt++) {
        // Preferred path: repeated-start register read.
        w_rc = i2c_write_timeout_us(I2C_PORT, addr, &reg, 1, true, I2C_TIMEOUT_US);
        if (w_rc == 1) {
            r_rc = i2c_read_timeout_us(I2C_PORT, addr, chip_id, 1, false, I2C_TIMEOUT_US);
            if (r_rc == 1) {
                return true;
            }
        }

        // Fallback path: STOP between register write and read.
        w_rc = i2c_write_timeout_us(I2C_PORT, addr, &reg, 1, false, I2C_TIMEOUT_US);
        if (w_rc == 1) {
            sleep_us(50);
            r_rc = i2c_read_timeout_us(I2C_PORT, addr, chip_id, 1, false, I2C_TIMEOUT_US);
            if (r_rc == 1) {
                return true;
            }
        }

        sleep_us(100);
    }

#if DEBUG_ENABLE
    printf("BMP probe 0x%02X failed (w_rc=%d r_rc=%d)\r\n", addr, w_rc, r_rc);
#endif
    return false;
}

static bool write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    for (int attempt = 0; attempt < I2C_RETRY_COUNT; attempt++) {
        if (i2c_write_timeout_us(I2C_PORT, bme_addr, buf, 2, false, I2C_TIMEOUT_US) == 2) {
            return true;
        }
        sleep_us(100);
    }
    return false;
}

static bool read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    for (int attempt = 0; attempt < I2C_RETRY_COUNT; attempt++) {
        // Preferred path: repeated-start register read.
        if (i2c_write_timeout_us(I2C_PORT, bme_addr, &reg, 1, true, I2C_TIMEOUT_US) == 1) {
            if (i2c_read_timeout_us(I2C_PORT, bme_addr, buf, len, false, I2C_TIMEOUT_US) == (int)len) {
                return true;
            }
        }

        // Fallback path: STOP between register write and read.
        if (i2c_write_timeout_us(I2C_PORT, bme_addr, &reg, 1, false, I2C_TIMEOUT_US) == 1) {
            sleep_us(50);
            if (i2c_read_timeout_us(I2C_PORT, bme_addr, buf, len, false, I2C_TIMEOUT_US) == (int)len) {
                return true;
            }
        }

        sleep_us(100);
    }
    return false;
}

static bool read_calibration(void) {
    uint8_t buf[26];
    
    if (!read_regs(BME280_REG_CALIB00, buf, 26)) return false;
    
    calib.dig_T1 = (uint16_t)(buf[1] << 8) | buf[0];
    calib.dig_T2 = (int16_t)(buf[3] << 8) | buf[2];
    calib.dig_T3 = (int16_t)(buf[5] << 8) | buf[4];
    calib.dig_P1 = (uint16_t)(buf[7] << 8) | buf[6];
    calib.dig_P2 = (int16_t)(buf[9] << 8) | buf[8];
    calib.dig_P3 = (int16_t)(buf[11] << 8) | buf[10];
    calib.dig_P4 = (int16_t)(buf[13] << 8) | buf[12];
    calib.dig_P5 = (int16_t)(buf[15] << 8) | buf[14];
    calib.dig_P6 = (int16_t)(buf[17] << 8) | buf[16];
    calib.dig_P7 = (int16_t)(buf[19] << 8) | buf[18];
    calib.dig_P8 = (int16_t)(buf[21] << 8) | buf[20];
    calib.dig_P9 = (int16_t)(buf[23] << 8) | buf[22];
    calib.dig_H1 = buf[25];
    
    if (!read_regs(BME280_REG_CALIB26, buf, 7)) return false;
    
    calib.dig_H2 = (int16_t)(buf[1] << 8) | buf[0];
    calib.dig_H3 = buf[2];
    calib.dig_H4 = (int16_t)((buf[3] << 4) | (buf[4] & 0x0F));
    calib.dig_H5 = (int16_t)((buf[5] << 4) | (buf[4] >> 4));
    calib.dig_H6 = (int8_t)buf[6];
    
    return true;
}

bool bme280_init(void) {
    uint8_t chip_id = 0;
    const uint8_t addrs[] = {BME280_ADDR, BME280_ALT_ADDR};
    bool found = false;

    for (size_t i = 0; i < sizeof(addrs); i++) {
        uint8_t probed_id = 0;
        if (probe_addr(addrs[i], &probed_id) &&
            (probed_id == BME280_CHIP_ID || probed_id == BMP280_CHIP_ID)) {
            bme_addr = addrs[i];
            chip_id = probed_id;
            found = true;
            break;
        }
    }

    if (!found) {
#if DEBUG_ENABLE
        printf("BME280/BMP280 probe failed (no response at 0x%02X or 0x%02X)\r\n", BME280_ADDR, BME280_ALT_ADDR);
#endif
        return false;
    }
    
    if (chip_id != BME280_CHIP_ID && chip_id != BMP280_CHIP_ID) {
#if DEBUG_ENABLE
        printf("BME280/BMP280 probe failed (CHIP_ID=0x%02X, expected 0x%02X or 0x%02X)\r\n", 
               chip_id, BME280_CHIP_ID, BMP280_CHIP_ID);
#endif
        return false;
    }
    
    sensor_chip_id = chip_id;  // Store for later use
    // Print a friendly name
    printf("INFO: Found %s sensor\n", (chip_id == BMP280_CHIP_ID) ? "BMP280" : "BME280");

    // Set the configuration
    // ... existing code ...
    if (!write_reg(BME280_REG_RESET, BME280_RESET_VAL)) return false;
    sleep_ms(10);
    
    uint8_t status;
    int retry = 0;
    do {
        if (!read_regs(BME280_REG_STATUS, &status, 1)) return false;
        sleep_ms(1);
        retry++;
    } while ((status & 0x01) && retry < 10);  // Limit retries to prevent hangs
    
    if (!read_calibration()) return false;
    
    // Only configure humidity for BME280 (has humidity sensor)
    if (chip_id == BME280_CHIP_ID) {
        if (!write_reg(BME280_REG_CTRL_HUM, BME280_OVERSAMPLE_HUM)) return false;
    }
    
    uint8_t config = (BME280_FILTER_COEFF << 2) | (BME280_STANDBY_TIME << 5);
    if (!write_reg(BME280_REG_CONFIG, config)) return false;
    
    uint8_t ctrl = (BME280_OVERSAMPLE_TEMP << 5) | (BME280_OVERSAMPLE_PRESS << 2) | BME280_MODE;
    if (!write_reg(BME280_REG_CTRL_MEAS, ctrl)) return false;
    
#if DEBUG_ENABLE
    const char *sensor_name = (sensor_chip_id == BMP280_CHIP_ID) ? "BMP280" : "BME280";
    printf("%s initialized (temp x%d, press x%d%s)\r\n", sensor_name,
           1 << (BME280_OVERSAMPLE_TEMP > 0 ? BME280_OVERSAMPLE_TEMP - 1 : 0), 
           1 << (BME280_OVERSAMPLE_PRESS > 0 ? BME280_OVERSAMPLE_PRESS - 1 : 0),
           sensor_chip_id == BME280_CHIP_ID ? ", hum x1" : "");
#endif
    
    return true;
}

bool bme280_is_connected(void) {
    uint8_t chip_id = 0;
    if (probe_addr(bme_addr, &chip_id)) {
        if (chip_id == BME280_CHIP_ID || chip_id == BMP280_CHIP_ID) return true;
    }

    const uint8_t addrs[] = {BME280_ADDR, BME280_ALT_ADDR};
    for (size_t i = 0; i < sizeof(addrs); i++) {
        if (probe_addr(addrs[i], &chip_id) &&
            (chip_id == BME280_CHIP_ID || chip_id == BMP280_CHIP_ID)) {
            bme_addr = addrs[i];
            return true;
        }
    }
    return false;
}

static float compensate_temperature(int32_t adc_T) {
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
            ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) / 25600.0f;
}

static float compensate_pressure(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
    
    if (var1 == 0) return 0;
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    
    return p / 256.0f;
}

static float compensate_humidity(int32_t adc_H) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) - (((int32_t)calib.dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) *
                   (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                   ((int32_t)2097152)) * ((int32_t)calib.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (v_x1_u32r >> 12) / 1024.0f;
}

bool bme280_read(BME280_Data *data) {
    if (!data) return false;
    
    uint8_t buf[8];
    if (!read_regs(BME280_REG_PRESS_MSB, buf, 8)) return false;
    
    int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
    int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((int32_t)buf[5] >> 4);
    
    data->temperature = compensate_temperature(adc_T);
    data->pressure = compensate_pressure(adc_P);
    
    // BMP280 doesn't have humidity sensor; set to 0 for BMP280, compute for BME280
    if (sensor_chip_id == BMP280_CHIP_ID) {
        data->humidity = 0.0f;
    } else {
        int32_t adc_H = ((int32_t)buf[6] << 8) | (int32_t)buf[7];
        data->humidity = compensate_humidity(adc_H);
    }
    
    data->altitude = bme280_calculate_altitude(data->pressure);
    
    return true;
}

bool bme280_read_pressure(float *pressure) {
    if (!pressure) return false;
    
    uint8_t buf[6];
    if (!read_regs(BME280_REG_PRESS_MSB, buf, 6)) return false;
    
    int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
    int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((int32_t)buf[5] >> 4);
    
    compensate_temperature(adc_T);
    *pressure = compensate_pressure(adc_P);
    
    return true;
}

void bme280_set_ground_pressure(float pressure_pa) { ground_pressure = pressure_pa; }
float bme280_get_ground_pressure(void) { return ground_pressure; }

float bme280_calculate_altitude(float pressure) {
    return 44330.0f * (1.0f - powf(pressure / ground_pressure, 0.1903f));
}

bool bme280_calibrate_ground(uint16_t samples) {
    float sum = 0;
    BME280_Data data;
    
    for (uint16_t i = 0; i < samples; i++) {
        if (!bme280_read(&data)) return false;
        sum += data.pressure;
        sleep_ms(50);
    }
    
    ground_pressure = sum / samples;
    return true;
}

void bme280_reset(void) {
    write_reg(BME280_REG_RESET, BME280_RESET_VAL);
    sleep_ms(10);
}
