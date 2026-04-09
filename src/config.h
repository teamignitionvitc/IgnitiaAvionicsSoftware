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

// UART1 - GPS (NEO-M8M)
#define GPS_UART            uart1
#define GPS_TX_PIN          4
#define GPS_RX_PIN          5
#define GPS_BAUDRATE        9600

// UART0 - NRF Telemetry Radio
#define NRF_UART            uart0
#define NRF_TX_PIN          1
#define NRF_RX_PIN          0
#define NRF_BAUDRATE        115200

// PWM - Servo
#define SERVO_PIN           14
#define SERVO_PWM_FREQ      50

// Status LED
#define STATUS_LED_PIN      7
#define CAL_LED_PIN         6
#define BUZZER_PIN          13

// =============================================================================
// I2C ADDRESSES
// =============================================================================
#define MPU6050_ADDR        0x68
#define BME280_ADDR         0x76

// =============================================================================
// SERVO CONFIGURATION
// =============================================================================
#define SERVO_MIN_PULSE_US  500
#define SERVO_MAX_PULSE_US  2500
#define SERVO_CLOSED_ANGLE  0
#define SERVO_OPEN_ANGLE    90

// =============================================================================
// FLIGHT PARAMETERS (DRONE DROP MODE)
// =============================================================================

// Drop detection (drone releases cansat)
// In freefall, accelerometer reads near 0g (gravity is not sensed during freefall)
// accel_magnitude is in g's (1g = 9.81 m/s²)
#define DROP_ACCEL_THRESHOLD        0.18f       // g - freefall detection (<0.18g = true freefall, more robust)
#define DROP_VELOCITY_THRESHOLD     -0.3f       // m/s - rapid descent as backup
#define DROP_CONFIRMATION_COUNT     7           // Consecutive readings to confirm drop (more robust)
#define DROP_ALTITUDE_CHANGE        0.3f        // m - altitude drop to confirm (unused)

// Auto-arm: detect drone ascending with cansat attached
#define LAUNCH_ALTITUDE_RISE        1.0f        // m above ground to auto-arm
#define LAUNCH_VELOCITY_THRESHOLD   0.2f        // m/s upward velocity
#define LAUNCH_CONFIRMATION_COUNT   5          // Consecutive samples to confirm ascent

// Manual arm is also available via 'a' command over USB

// Deployment (parachute release during descent)
#define DEPLOY_DELAY_MS             150         // ms after drop detection before deploy (faster response)
#define DEPLOY_ALTITUDE_MIN         10.0f       // m - minimum altitude for deployment
#define DEPLOY_VELOCITY_THRESHOLD   -0.3f       // m/s - must be descending

// Landing detection
#define LANDING_VELOCITY_THRESHOLD  0.3f        // m/s - near stationary
#define LANDING_ALTITUDE_THRESHOLD  5.0f        // m AGL - close to ground
#define LANDING_CONFIRMATION_TIME   3000        // ms - sustained for this long

// Safety overrides
#define DEPLOYMENT_SAFETY_ALT       350.0f      // m - force deploy if below this while armed
#define DEPLOYMENT_SAFETY_TIME      10000       // ms - force deploy 10s after drop detected
#define ARM_ALTITUDE_MIN            20.0f       // m - minimum altitude to detect freefall (avoid false triggers)

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
#define TELEMETRY_INTERVAL_MS       100

// =============================================================================
// FILTER PARAMETERS
// =============================================================================
#define ALTITUDE_FILTER_ALPHA       0.1f
#define VELOCITY_FILTER_ALPHA       0.2f
#define COMPLEMENTARY_FILTER_ALPHA  0.98f
#define ALTITUDE_OUTLIER_REJECT_M   6.0f        // Reject abrupt baro spikes per sample
#define VELOCITY_CLAMP_MPS          35.0f       // Clamp fused vertical velocity
#define ALTITUDE_ZERO_CAL_SAMPLES   30          // Startup samples for altitude zero reference
#define ALTITUDE_KALMAN_Q           0.05f
#define ALTITUDE_KALMAN_R           10.0f
#define VELOCITY_KALMAN_Q           0.1f
#define VELOCITY_KALMAN_R           5.0f
#define VELOCITY_DEADBAND_MPS       0.05f
#define BARO_IDLE_REZERO_RATE       0.003f      // Slow auto-zero while stationary
#define BARO_IDLE_VEL_THRESHOLD     0.12f       // m/s for stationary drift correction
#define BARO_IDLE_ACCEL_TOL_G       0.08f       // |accel_mag-1g| tolerance for stationary detect
#define BASELINE_SETTLE_TIME_MS     5000        // Lock launch baseline after startup settle
#define BASELINE_TRACK_ALPHA        0.01f       // Slow baseline tracking before lock

// =============================================================================
// DEBUG OPTIONS
// =============================================================================
#define DEBUG_ENABLE                1
#define DEBUG_VERBOSE               0
#define DEBUG_RAW_SENSORS           0

// =============================================================================
// FLIGHT STATES (DRONE DROP MODE)
// =============================================================================
typedef enum {
    STATE_INIT = 0,         // System initialization
    STATE_IDLE,             // On ground, waiting
    STATE_ARMED,            // Attached to drone, waiting for drop
    STATE_FREEFALL,         // Dropped, in freefall
    STATE_DEPLOYED,         // Parachute deployed, descending
    STATE_LANDED,           // On ground
    STATE_ERROR             // Error state
} FlightState;

// =============================================================================
// ERROR CODES
// =============================================================================
typedef enum {
    ERR_NONE = 0,
    ERR_I2C_INIT,
    ERR_MPU6050_INIT,
    ERR_BME280_INIT,
    ERR_GPS_INIT,
    ERR_SERVO_INIT,
    ERR_SENSOR_READ,
    ERR_DEPLOYMENT_FAILED,
    ERR_BUFFER_OVERFLOW
} ErrorCode;



#endif // CONFIG_H
