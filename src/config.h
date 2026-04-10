/**
 * @file config.h
 * @brief Configuration constants for Ignitia CanSat Avionics
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "pico/stdlib.h"

// =============================================================================
// VERSION INFO
// =============================================================================
#define FIRMWARE_VERSION_MAJOR  1
#define FIRMWARE_VERSION_MINOR  0
#define FIRMWARE_VERSION_PATCH  0
#define FIRMWARE_NAME           "Ignitia Avionics"

// =============================================================================
// PIN DEFINITIONS (RP2040-Zero)
// =============================================================================

// I2C1 - Sensors on GP2/GP3 (MPU6050, BMP280/BME280)
#define I2C_PORT            i2c1
#define I2C_SDA_PIN         2
#define I2C_SCL_PIN         3
#define I2C_BAUDRATE        100000

// UART1 - GPS (NEO-M8M-0-10)
#define GPS_UART            uart1
#define GPS_TX_PIN          4
#define GPS_RX_PIN          5
#define GPS_BAUDRATE        9600
#define GPS_LOCK_TIMEOUT_MS 6000        // Max wait for GPS fix at startup (6s)
#define GPS_NAV_RATE_HZ     5           // Navigation solution rate (Hz)

// UART0 - NRF Telemetry Radio
#define NRF_UART            uart0
#define NRF_TX_PIN          0
#define NRF_RX_PIN          1
#define NRF_BAUDRATE        115200

// UART TX timeout (ms)
#define UART_TX_TIMEOUT_MS  500

// PWM - Servos: see SERVO CONFIGURATION section below

// Status LEDs (external)
#define STATUS_LED_PIN      7
#define CAL_LED_PIN         6

// WS2812B RGB LED (RP2040-Zero built-in)
#define WS2812_PIN          16

// Passive Buzzer (PWM-driven)
#define BUZZER_PIN          13
#define BUZZER_FREQ_INIT    2000        // Hz — startup beep
#define BUZZER_FREQ_LANDED  1500        // Hz — landing tone
#define BUZZER_BEEP_MS      200         // ms — beep duration

// Digital sensor input (GP12 — reads 0 or 1)
#define SENSOR_GP12_PIN     12

// =============================================================================
// I2C ADDRESSES
// =============================================================================
#define MPU6050_ADDR        0x68
#define BME280_ADDR         0x76

// =============================================================================
// SERVO CONFIGURATION (MG90S position servo — angle-based, holds position)
// =============================================================================
#define SERVO_PIN           14
#define SERVO_PIN_2         15          // Second servo
#define SERVO_PWM_FREQ      50
#define SERVO_MIN_PULSE_US  500
#define SERVO_MAX_PULSE_US  2500
#define SERVO_NEUTRAL_US    1500
#define SERVO_CLOSED_ANGLE  270           // Locked position (init)
#define SERVO_OPEN_ANGLE    180        // Released position (deploy at apogee)

// =============================================================================
// SD CARD CONFIGURATION (SPI1 — data logging on Core 1)
// =============================================================================
#define SD_SPI_PORT         spi1
#define SD_SPI_BAUDRATE     1000000     // 1 MHz for init
#define SD_SPI_FAST_BAUD    10000000    // 10 MHz for data transfer
#define SD_PIN_MISO         8
#define SD_PIN_CS           9
#define SD_PIN_SCK          10
#define SD_PIN_MOSI         11
#define SD_LOG_RATE_HZ      50          // SD card logging rate (Hz) — independent of UART TX
#define SD_LOG_INTERVAL_MS  (1000 / SD_LOG_RATE_HZ)  // = 20ms
#define SD_LOG_FLUSH_INTERVAL_MS 1000   // Flush write buffer every 1s

// =============================================================================
// FLIGHT PARAMETERS — ALTITUDE-ONLY STATE TRANSITIONS
// =============================================================================
// States: INIT -> ARMED -> APOGEE -> DEPLOYED -> LANDED
// After init/calibration: auto-arm.
// Altitude peak + drop: apogee detected.
// Apogee: immediately deploy parachute.
// Back on ground: landed.

// ARMED -> APOGEE: altitude drops from max_altitude by this amount
#define APOGEE_DROP_THRESHOLD       0.5f        // m below max_altitude to detect apogee
#define APOGEE_CONFIRM_COUNT        5           // Consecutive readings to confirm

// Deployment (parachute release at apogee)
#define DEPLOY_DELAY_MS             100         // ms after apogee detection before deploy
#define DEPLOYMENT_SAFETY_TIME      10000       // ms - force deploy 10s after apogee detected

// DEPLOYED -> LANDED: altitude close to baseline for sustained period
#define LANDING_ALT_THRESHOLD       2.0f        // m above baseline = considered landed
#define LANDING_CONFIRM_TIME_MS     3000        // ms - sustained for this long

// =============================================================================
// SENSOR CONFIGURATION
// =============================================================================

// BME280/BMP280 register values
#define BME280_MODE_NORMAL      0x03
#define BME280_SAMPLING_X2      0x02
#define BME280_SAMPLING_X16     0x05
#define BME280_FILTER_X16       0x04
#define BME280_STANDBY_MS_1     0x00

// BME280 configuration
#define BME280_MODE             BME280_MODE_NORMAL
#define BME280_OVERSAMPLE_TEMP  BME280_SAMPLING_X2
#define BME280_OVERSAMPLE_PRESS BME280_SAMPLING_X16
#define BME280_OVERSAMPLE_HUM   1
#define BME280_FILTER_COEFF     BME280_FILTER_X16
#define BME280_STANDBY_TIME     BME280_STANDBY_MS_1
#define BME280_CONFIG           ((BME280_STANDBY_MS_1 << 5) | (BME280_FILTER_X16 << 2))
#define BME280_CTRL_MEAS        ((BME280_SAMPLING_X2 << 5) | (BME280_SAMPLING_X16 << 2) | BME280_MODE_NORMAL)
#define GROUND_PRESSURE_DEFAULT 101325.0f

// MPU6050 range values
#define MPU6050_ACCEL_RANGE_16G     3
#define MPU6050_GYRO_RANGE_2000DPS  3

// MPU6050 configuration
#define MPU6050_ACCEL_RANGE     MPU6050_ACCEL_RANGE_16G
#define MPU6050_GYRO_RANGE      MPU6050_GYRO_RANGE_2000DPS
#define MPU6050_SAMPLE_RATE     100

#define GPS_UPDATE_RATE         5
#define GPS_BUFFER_SIZE         256

// =============================================================================
// TELEMETRY & TIMING
// =============================================================================
#define TELEMETRY_RATE_HZ           10
#define TELEMETRY_BUFFER_SIZE       512
#define LOG_TIMESTAMP_ENABLE        1
#define SENSOR_SETTLE_TIME_MS       4000

#define MAIN_LOOP_INTERVAL_MS       10
#define SENSOR_READ_INTERVAL_MS     10
#define GPS_READ_INTERVAL_MS        200
#define UART_TX_RATE_HZ             1           // UART telemetry transmission rate (Hz)
#define TELEMETRY_INTERVAL_MS       (1000 / UART_TX_RATE_HZ)

// =============================================================================
// FILTER PARAMETERS
// =============================================================================
#define ALTITUDE_FILTER_ALPHA       0.3f
#define VELOCITY_FILTER_ALPHA       0.2f
#define COMPLEMENTARY_FILTER_ALPHA  0.98f
#define ALTITUDE_OUTLIER_REJECT_M   6.0f
#define VELOCITY_CLAMP_MPS          35.0f
#define ALTITUDE_ZERO_CAL_SAMPLES   30
#define ALTITUDE_KALMAN_Q           0.05f
#define ALTITUDE_KALMAN_R           10.0f
#define VELOCITY_KALMAN_Q           0.1f
#define VELOCITY_KALMAN_R           5.0f
#define VELOCITY_DEADBAND_MPS       0.05f
#define BARO_IDLE_REZERO_RATE       0.003f
#define BARO_IDLE_VEL_THRESHOLD     0.12f
#define BARO_IDLE_ACCEL_TOL_G       0.08f
#define BASELINE_SETTLE_TIME_MS     3000
#define BASELINE_TRACK_ALPHA        0.01f

// =============================================================================
// DEBUG OPTIONS
// =============================================================================
#define DEBUG_ENABLE                1
#define DEBUG_VERBOSE               0
#define DEBUG_RAW_SENSORS           0

// =============================================================================
// FLIGHT STATES
// =============================================================================
typedef enum {
    STATE_INIT = 0,         // System initialization & calibration
    STATE_ARMED,            // Ready for flight (auto after init)
    STATE_APOGEE,           // Peak altitude detected, deploying
    STATE_DEPLOYED,         // Parachute deployed, descending
    STATE_LANDED,           // On ground
    STATE_ERROR             // Error state
} FlightState;

// =============================================================================
// ERROR CODES (each has a unique LED blink pattern)
// =============================================================================
typedef enum {
    ERR_NONE = 0,           // Green LED solid
    ERR_I2C_INIT,           // 1 red blink
    ERR_MPU6050_INIT,       // 2 red blinks
    ERR_BME280_INIT,        // 3 red blinks
    ERR_GPS_INIT,           // 4 red blinks
    ERR_SERVO_INIT,         // 5 red blinks
    ERR_SD_INIT,            // 6 red blinks
    ERR_SENSOR_READ,        // 7 red blinks
    ERR_DEPLOYMENT_FAILED,  // Fast red strobe
    ERR_BUFFER_OVERFLOW     // Yellow blink
} ErrorCode;

#endif // CONFIG_H
