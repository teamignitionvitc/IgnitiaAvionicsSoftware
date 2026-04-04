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

#define MAIN_LOOP_INTERVAL_MS       20
#define SENSOR_READ_INTERVAL_MS     10

// CRITICAL FIX (Issue 7): Task prioritization intervals
// HIGH priority (every loop): sensors, filtering, fusion, state, logic, deployment
// MEDIUM priority (every N loops): telemetry
// LOW priority (every N loops): GPS, logging
#define TELEMETRY_INTERVAL          2    // Every 2 loops = ~40ms at 50Hz = 25 Hz
#define GPS_READ_INTERVAL           10   // Every 10 loops = ~500ms at 50Hz = 2 Hz
#define LOGGING_INTERVAL            5    // Every 5 loops = ~100ms at 50Hz = 10 Hz

// CRITICAL FIX (Issue 5): Watchdog timer timeout
#define WATCHDOG_TIMEOUT_MS         150  // 100-200ms for 50Hz loop (20ms nominal)

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
// FEATURE FLAGS (Compile-time configuration)
// =============================================================================
#define TEST_MODE                   0    // 0 = Normal, 1 = Test mode
#define SERIAL_ECHO_TEST_MODE       0    // 0 = Normal firmware, 1 = serial-only echo loop (debug USB input)
#define LOGGING_ENABLED             1    // 0 = Disabled, 1 = Enabled
#define TELEMETRY_ENABLED           1    // 0 = Disabled, 1 = Enabled
#define GPS_ENABLED                 1    // 0 = Disabled, 1 = Enabled
#define DEBUG_OUTPUT                0    // 0 = Disabled, 1 = Enabled

// =============================================================================
// TUNABLE PARAMETERS (Easy configuration)
// =============================================================================

// Detection thresholds
#define FREEFALL_ACCEL_THRESHOLD    0.3f        // g - freefall detection
#define FREEFALL_VELOCITY_THRESHOLD -200        // cm/s - falling velocity (fixed-point)
#define APOGEE_POSITIVE_THRESHOLD   50          // cm/s - velocity must be above this before apogee
#define APOGEE_NEGATIVE_THRESHOLD   -20         // cm/s - velocity must drop below this for apogee
#define APOGEE_CONFIRMATION_COUNT   3           // Consecutive readings to confirm apogee

// Timeout values
#define FREEFALL_TIMEOUT_MS         60000       // ms - force deploy after this in freefall
#define APOGEE_TIMEOUT_MS           5000        // ms - force deploy after this at apogee
#define DEPLOYED_TIMEOUT_MS         120000      // ms - force landing after this when deployed

// Filter parameters
#define COMPLEMENTARY_ALPHA_NORMAL  0.7f        // 70% accel + 30% baro (normal)
#define COMPLEMENTARY_ALPHA_NOISY   0.5f        // 50% accel + 50% baro (high noise)
#define NOISE_VARIANCE_THRESHOLD    5.0f        // m² - variance threshold for high noise

// Clamping limits
#define MAX_VELOCITY_MPS            50.0f       // m/s - maximum velocity
#define MAX_VELOCITY_CMPS           5000        // cm/s - maximum velocity (fixed-point)
#define MAX_ALTITUDE_M              5000.0f     // m - maximum altitude
#define MAX_ACCEL_G                 3.0f        // g - maximum acceleration

#endif // CONFIG_H
