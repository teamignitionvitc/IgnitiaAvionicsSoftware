/**
 * @file bme280.c
 * @brief BME280 Environmental sensor driver implementation
 */

#include "bme280.h"
#include "config.h"
#include "hardware/i2c.h"
#include <math.h>

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
#define BME280_RESET_VAL        0xB6

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

static bool write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_write_blocking(I2C_PORT, BME280_ADDR, buf, 2, false) == 2;
}

static bool read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    if (i2c_write_blocking(I2C_PORT, BME280_ADDR, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(I2C_PORT, BME280_ADDR, buf, len, false) == (int)len;
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
    uint8_t chip_id;
    if (!read_regs(BME280_REG_CHIP_ID, &chip_id, 1) || chip_id != BME280_CHIP_ID) {
        return false;
    }
    
    if (!write_reg(BME280_REG_RESET, BME280_RESET_VAL)) return false;
    sleep_ms(10);
    
    uint8_t status;
    do {
        if (!read_regs(BME280_REG_STATUS, &status, 1)) return false;
        sleep_ms(1);
    } while (status & 0x01);
    
    if (!read_calibration()) return false;
    
    if (!write_reg(BME280_REG_CTRL_HUM, BME280_OVERSAMPLE_HUM)) return false;
    
    uint8_t config = (BME280_FILTER_COEFF << 2);
    if (!write_reg(BME280_REG_CONFIG, config)) return false;
    
    uint8_t ctrl = (BME280_OVERSAMPLE_TEMP << 5) | (BME280_OVERSAMPLE_PRESS << 2) | 0x03;
    if (!write_reg(BME280_REG_CTRL_MEAS, ctrl)) return false;
    
    return true;
}

bool bme280_is_connected(void) {
    uint8_t chip_id;
    if (!read_regs(BME280_REG_CHIP_ID, &chip_id, 1)) return false;
    return chip_id == BME280_CHIP_ID;
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
    int32_t adc_H = ((int32_t)buf[6] << 8) | (int32_t)buf[7];
    
    data->temperature = compensate_temperature(adc_T);
    data->pressure = compensate_pressure(adc_P);
    data->humidity = compensate_humidity(adc_H);
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
