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

// I2C0 - Sensors (MPU6050, BME280)
#define I2C_PORT            i2c0
#define I2C_SDA_PIN         4
#define I2C_SCL_PIN         5
#define I2C_BAUDRATE        400000

// UART1 - GPS (NEO-M8M)
#define GPS_UART            uart1
#define GPS_TX_PIN          8
#define GPS_RX_PIN          9
#define GPS_BAUDRATE        9600

// UART0 - NRF Telemetry Radio
#define NRF_UART            uart0
#define NRF_TX_PIN          0
#define NRF_RX_PIN          1
#define NRF_BAUDRATE        115200

// PWM - Servo
#define SERVO_PIN           15
#define SERVO_PWM_FREQ      50

// Status LED
#define STATUS_LED_PIN      16

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
#define DROP_ACCEL_THRESHOLD        0.3f        // g - freefall detection (<0.3g)
#define DROP_VELOCITY_THRESHOLD     -2.0f       // m/s - falling velocity
#define DROP_CONFIRMATION_COUNT     3           // Consecutive readings to confirm
#define DROP_ALTITUDE_CHANGE        5.0f        // m - altitude drop to confirm

// Deployment (parachute release during descent)
#define DEPLOY_DELAY_MS             500         // ms after drop detection
#define DEPLOY_ALTITUDE_MIN         30.0f       // m - minimum altitude for deployment
#define DEPLOY_VELOCITY_THRESHOLD   -1.0f       // m/s - must be descending

// Landing detection
#define LANDING_VELOCITY_THRESHOLD  0.5f        // m/s
#define LANDING_ALTITUDE_THRESHOLD  20.0f       // m AGL
#define LANDING_CONFIRMATION_TIME   5000        // ms

// Safety
#define DEPLOYMENT_SAFETY_ALT       500.0f      // m - force deploy below this
#define DEPLOYMENT_SAFETY_TIME      30000       // ms - force deploy after this from drop
#define ARM_ALTITUDE_MIN            50.0f       // m - minimum altitude to arm

// =============================================================================
// SENSOR CONFIGURATION
// =============================================================================
#define MPU6050_ACCEL_RANGE         2
#define MPU6050_GYRO_RANGE          250
#define MPU6050_SAMPLE_RATE         100

#define BME280_OVERSAMPLE_TEMP      2
#define BME280_OVERSAMPLE_PRESS     16
#define BME280_OVERSAMPLE_HUM       1
#define BME280_FILTER_COEFF         4
#define GROUND_PRESSURE_DEFAULT     101325.0f

#define GPS_UPDATE_RATE             5
#define GPS_BUFFER_SIZE             256

// =============================================================================
// TELEMETRY & TIMING
// =============================================================================
#define TELEMETRY_RATE_HZ           10
#define TELEMETRY_BUFFER_SIZE       512
#define LOG_TIMESTAMP_ENABLE        1

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
