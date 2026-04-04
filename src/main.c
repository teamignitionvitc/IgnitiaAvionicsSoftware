/**
 * @file main.c
 * @brief Ignitia CanSat deterministic flight loop.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/error.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"

#include "config.h"
#include "flight_data.h"
#include "mpu6050.h"
#include "bme280.h"
#include "neo_m8m.h"
#include "servo.h"
#include "deployment.h"
#include "telemetry.h"
#include "state_machine.h"
#include "flight_logic.h"
#include "sensor_fusion.h"
#include "logger.h"
#include "health_monitor.h"
#include "filters.h"

static MPU6050_Data imu_data;
static BME280_Data env_data;
static GPS_Data gps_data;
static SensorData sensor_data;

static float last_accel_mag = 1.0f;
static float last_altitude = 0.0f;
static uint16_t imu_stuck_count = 0;
static uint16_t bme_stuck_count = 0;
static bool sensor_fault_reported = false;

static void led_blink(uint8_t count, uint32_t on_ms, uint32_t off_ms) {
    for (uint8_t i = 0; i < count; i++) {
        gpio_put(STATUS_LED_PIN, 1);
        sleep_ms(on_ms);
        gpio_put(STATUS_LED_PIN, 0);
        sleep_ms(off_ms);
    }
}

static bool init_hardware(void) {
    printf("\r\n");
    printf("========================================\r\n");
    printf("  %s v%d.%d.%d\r\n", FIRMWARE_NAME,
           FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH);
    printf("========================================\r\n");

    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);
    gpio_put(STATUS_LED_PIN, 1);

    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("Initializing MPU6050... ");
    if (!mpu6050_init()) {
        printf("FAILED\r\n");
        return false;
    }
    printf("OK\r\n");

    printf("Initializing BME280... ");
    if (!bme280_init()) {
        printf("FAILED\r\n");
        return false;
    }
    printf("OK\r\n");

    printf("Initializing GPS... ");
    if (!gps_init()) {
        printf("FAILED\r\n");
        return false;
    }
    printf("OK\r\n");

    printf("Initializing Servo... ");
    if (!servo_init()) {
        printf("FAILED\r\n");
        return false;
    }
    printf("OK\r\n");

    deployment_init();
    telemetry_init();
    
    // Watchdog will be enabled AFTER full system initialization in main()
    // to prevent USB enumeration issues
    
    printf("Calibrating sensors...\r\n");
    printf("  MPU6050: ");
    if (mpu6050_calibrate(100)) {
        printf("OK\r\n");
    } else {
        printf("FAILED\r\n");
    }

    printf("  BME280 ground level: ");
    if (bme280_calibrate_ground(20)) {
        printf("%.0f Pa\r\n", bme280_get_ground_pressure());
    } else {
        printf("FAILED\r\n");
    }

    gpio_put(STATUS_LED_PIN, 0);
    printf("Hardware initialization complete\r\n\r\n");

    return true;
}

static void read_sensors(SensorData *data) {
    bool imu_ok = mpu6050_read(&imu_data);
    bool bme_ok = bme280_read(&env_data);
    gps_read(&gps_data);

    data->imu_ok = imu_ok;
    data->bme_ok = bme_ok;
    
    // Update health monitor
    health_update_imu(imu_ok, data->timestamp_ms);
    health_update_baro(bme_ok, data->timestamp_ms);

    if (imu_ok) {
        data->accel_mag_g = imu_data.accel_magnitude;
        data->accel_z_g = imu_data.accel_z;
        
        // IMPROVEMENT (Small but Powerful): Clamp acceleration to ±3.0g
        data->accel_mag_g = clamp_float(data->accel_mag_g, 0.0f, 3.0f);
        data->accel_z_g = clamp_float(data->accel_z_g, -3.0f, 3.0f);
    } else {
        data->accel_mag_g = last_accel_mag;
        data->accel_z_g = 1.0f;
    }

    if (bme_ok) {
        data->baro_altitude_m = env_data.altitude;
        data->temperature_c = env_data.temperature;
    } else {
        data->baro_altitude_m = last_altitude;
    }

    data->gps_valid = gps_data.valid;
    data->gps_sats = gps_data.satellites;

    if (imu_ok && fabsf(data->accel_mag_g - last_accel_mag) < 0.0005f) {
        imu_stuck_count++;
    } else {
        imu_stuck_count = 0;
    }

    if (bme_ok && fabsf(data->baro_altitude_m - last_altitude) < 0.0005f) {
        bme_stuck_count++;
    } else {
        bme_stuck_count = 0;
    }

    last_accel_mag = data->accel_mag_g;
    last_altitude = data->baro_altitude_m;

    data->sensor_fault = (!imu_ok || !bme_ok || imu_stuck_count > 250 || bme_stuck_count > 250);

    if (data->sensor_fault && !sensor_fault_reported) {
        telemetry_send_status("SENSOR FAULT");
        sensor_fault_reported = true;
    } else if (!data->sensor_fault && sensor_fault_reported) {
        telemetry_send_status("SENSOR RECOVERED");
        sensor_fault_reported = false;
    }
}

static void handle_deployment(SensorData *data) {
    if (!flight_logic_should_deploy()) {
        return;
    }

    if (deployment_is_deployed()) {
        flight_logic_mark_deployed();
        return;
    }

        telemetry_send_status("DEPLOYING PARACHUTE");

    if (deployment_trigger(data->filtered_altitude_m)) {
        flight_logic_mark_deployed();
        state_request_transition(STATE_DEPLOYED);
        telemetry_send_status("DEPLOYMENT SUCCESS");
    } else {
        telemetry_send_status("DEPLOYMENT FAILED");
    }
}

/**
 * @brief Check for state timeout and force transitions if needed
 * 
 * Safety mechanism to prevent getting stuck in a state:
 * - FREEFALL timeout: 60s → force deploy
 * - APOGEE timeout: 5s → force deploy
 * - DEPLOYED timeout: 120s → force landing
 */
static void check_state_timeout(SensorData *data) {
    uint32_t time_in_state = state_time_in_state_ms(data->timestamp_ms);
    FlightState current = get_state();
    
    switch (current) {
        case STATE_FREEFALL:
            // If in freefall for more than 60 seconds, force deployment
            if (time_in_state > 60000) {
                telemetry_send_status("FREEFALL TIMEOUT - FORCE DEPLOY");
                if (!deployment_is_deployed()) {
                    deployment_trigger(data->filtered_altitude_m);
                }
                flight_logic_mark_deployed();
                state_request_transition(STATE_DEPLOYED);
            }
            break;
            
        case STATE_APOGEE:
            // If at apogee for more than 5 seconds, force deployment
            if (time_in_state > 5000) {
                telemetry_send_status("APOGEE TIMEOUT - FORCE DEPLOY");
                if (!deployment_is_deployed()) {
                    deployment_trigger(data->filtered_altitude_m);
                }
                flight_logic_mark_deployed();
                state_request_transition(STATE_DEPLOYED);
            }
            break;
            
        case STATE_DEPLOYED:
            // If deployed for more than 120 seconds, force landing state
            if (time_in_state > 120000) {
                telemetry_send_status("DEPLOYED TIMEOUT - FORCE LANDING");
                state_request_transition(STATE_LANDED);
            }
            break;
            
        default:
            // No timeout for other states
            break;
    }
}

/**
 * @brief Validate pre-flight conditions before arming
 * 
 * Checks:
 * - Sensor health (IMU and Baro must be OK)
 * - Altitude > 50m (prevents ground arming)
 * - SD card ready (optional)
 * - GPS lock (optional)
 * 
 * @param data Current sensor data
 * @param reason Output buffer for failure reason (can be NULL)
 * @return true if all checks pass, false otherwise
 */
static bool validate_preflight(SensorData *data, char *reason) {
    const SystemHealth *health = health_get_status();
    
    // Check sensor health - IMU and Baro are critical
    if (!health->imu_ok) {
        if (reason) snprintf(reason, 64, "IMU NOT READY");
        return false;
    }
    
    if (!health->baro_ok) {
        if (reason) snprintf(reason, 64, "BARO NOT READY");
        return false;
    }
    
    // Check altitude - must be above minimum arm altitude
    if (data->filtered_altitude_m < ARM_ALTITUDE_MIN) {
        if (reason) snprintf(reason, 64, "ALT TOO LOW (%.1fm)", data->filtered_altitude_m);
        return false;
    }
    
    // Check for sensor faults
    if (data->sensor_fault) {
        if (reason) snprintf(reason, 64, "SENSOR FAULT");
        return false;
    }
    
    // GPS lock is optional but recommended
    // (Not blocking arm, but log warning)
    if (!health->gps_ok) {
        telemetry_send_status("WARN: NO GPS LOCK");
    }
    
    // All critical checks passed
    if (reason) snprintf(reason, 64, "PREFLIGHT OK");
    return true;
}

static void handle_command_char(int c) {
    switch (c) {
        case 'a':
            {
                char reason[64];
                if (!validate_preflight(&sensor_data, reason)) {
                    telemetry_send_status(reason);
                    printf("ARM DENIED: %s\r\n", reason);
                    led_blink(5, 50, 50);  // Fast error blink
                    break;
                }
                
                if (state_request_transition(STATE_ARMED)) {
                    deployment_arm();
                    telemetry_send_status("ARMED");
                    led_blink(3, 100, 100);
                } else {
                    telemetry_send_status("ARM DENIED");
                }
            }
            break;

        case 'd':
            state_request_transition(STATE_IDLE);
            deployment_disarm();
            telemetry_send_status("DISARMED");
            break;

        case 't':
            telemetry_send_status("SERVO TEST");
            deployment_test();
            break;

        case 'r':
            deployment_reset();
            state_init();
            flight_logic_init();
            fusion_reset();
            logger_reset();
            memset(&sensor_data, 0, sizeof(sensor_data));
            telemetry_send_status("RESET COMPLETE");
            break;

        case 's':
            printf("$STS,State=%s,Alt=%.1f,Vel=%.2f,Accel=%.2f,DropMS=%lu,Deployed=%d\r\n",
                state_name(get_state()),
                sensor_data.filtered_altitude_m,
                sensor_data.fused_velocity_mps,
                sensor_data.accel_mag_g,
                (unsigned long)flight_logic_drop_time_ms(),
                deployment_is_deployed() ? 1 : 0);
            break;

        case 'c':
            telemetry_send_status("CALIBRATING");
            mpu6050_calibrate(100);
            bme280_calibrate_ground(20);
            flight_logic_init();
            fusion_reset();
            telemetry_send_status("CALIBRATION DONE");
            break;

        case '?':
        case 'h':
            printf("\r\n=== Ignitia CanSat Control Loop ===\r\n");
            printf("Commands:\r\n");
            printf("  a - Arm\r\n");
            printf("  d - Disarm\r\n");
            printf("  t - Test servo\r\n");
            printf("  r - Reset\r\n");
            printf("  s - Status\r\n");
            printf("  c - Calibrate sensors\r\n");
            printf("  h - Help\r\n");

            printf("\r\nState flow:\r\n");
            printf("  IDLE -> ARMED -> FREEFALL -> APOGEE -> DEPLOYED -> LANDED\r\n\r\n");
            break;
    }
}

static void handle_command_line(const char *line) {
    if (!line) return;

    // Skip leading whitespace
    while (*line == ' ' || *line == '\t') {
        line++;
    }

    // Handle empty input safely
    if (!*line) return;

    handle_command_char((int)*line);
}

static void process_commands(void) {
    // Buffered, line-based input (more reliable across Serial Monitors)
    static char rx_line[32];
    static uint8_t rx_len = 0;

    while (1) {
        int c = getchar_timeout_us(1000);
        if (c == PICO_ERROR_TIMEOUT) return;

        // Debug logging for every received character
        printf("RX: %c (%d)\n", c, c);

        // Accept both CR and LF as line terminators
        if (c == '\r' || c == '\n') {
            rx_line[rx_len] = '\0';
            handle_command_line(rx_line);
            rx_len = 0;
            continue;
        }

        // Minimal backspace support for terminals like PuTTY
        if (c == '\b' || c == 127) {
            if (rx_len > 0) rx_len--;
            continue;
        }

        // Accumulate into buffer (leave room for NUL terminator)
        if (rx_len < (sizeof(rx_line) - 1u)) {
            rx_line[rx_len++] = (char)c;
        }
    }
}

static void update_led(void) {
    static uint32_t last_blink = 0;
    static bool led_state = false;
    uint32_t now = to_ms_since_boot(get_absolute_time());

    uint32_t interval;
    switch (get_state()) {
        case STATE_IDLE:      interval = 1000; break;  // Slow blink
        case STATE_ARMED:     interval = 200;  break;  // Fast blink - ready
        case STATE_FREEFALL:  interval = 50;   break;  // Very fast - falling!
        case STATE_APOGEE:    interval = 100;  break;  // Transition marker
        case STATE_DEPLOYED:  interval = 500;  break;  // Medium - descending
        case STATE_LANDED:    interval = 2000; break;  // Very slow - done
        default:              interval = 100;  break;
    }

    if ((now - last_blink) >= interval) {
        last_blink = now;
        led_state = !led_state;
        gpio_put(STATUS_LED_PIN, led_state);
    }
}

int main(void) {
    // Ensure USB stdio is initialized immediately for CDC enumeration
    stdio_init_all();
    sleep_ms(4000);

#if SERIAL_ECHO_TEST_MODE
    printf("\r\n=== SERIAL ECHO TEST MODE ===\r\n");
    printf("Type characters; they will be echoed.\r\n\r\n");
    while (1) {
        int c = getchar_timeout_us(1000);
        if (c != PICO_ERROR_TIMEOUT) {
            printf("RX: %c (%d)\n", c, c);
            printf("Got: %c\n", c);
        }
    }
#endif

    if (!init_hardware()) {
        printf("Hardware init failed! Entering error state.\r\n");
        while (1) {
            led_blink(5, 50, 50);
            sleep_ms(500);
        }
    }

    memset(&sensor_data, 0, sizeof(sensor_data));
    state_init();
    flight_logic_init();
    fusion_init();
    logger_init();
    health_init();  // Initialize health monitoring system

    telemetry_send_status("SYSTEM READY");
    printf("Type 'h' for help\r\n\r\n");
    
    // Enable watchdog AFTER full system initialization to prevent USB enumeration issues
    sleep_ms(2000);  // Allow USB stable connection
    watchdog_enable(WATCHDOG_TIMEOUT_MS, 1);
    printf("Watchdog started (%d ms timeout)\r\n\r\n", WATCHDOG_TIMEOUT_MS);

    uint32_t last_loop_ms = to_ms_since_boot(get_absolute_time());
    uint32_t loop_counter = 0;  // CRITICAL FIX (Issue 7): Task prioritization counter

    while (1) {
        uint32_t loop_start_ms = to_ms_since_boot(get_absolute_time());
        uint32_t dt_ms = loop_start_ms - last_loop_ms;
        last_loop_ms = loop_start_ms;

        sensor_data.timestamp_ms = loop_start_ms;
        
        // Calculate dt with clamping to [0.001, 1.0] seconds (Requirement 9.4)
        float dt_s = dt_ms / 1000.0f;
        if (dt_s < 0.001f) {
            dt_s = 0.001f;  // Minimum 1ms
        } else if (dt_s > 1.0f) {
            dt_s = 1.0f;    // Maximum 1 second
        }
        sensor_data.dt_s = dt_s;

        // ===================================================================
        // HIGH PRIORITY: Critical flight control (every loop)
        // ===================================================================
        
        // Read sensors (IMU + Baro only, GPS moved to LOW priority)
        read_sensors(&sensor_data);
        
        // Filter and fuse sensor data
        filter_data(&sensor_data);
        fuse_sensors(&sensor_data);
        
        // Update state machine
        state_update(&sensor_data);
        
        // Check for state timeouts (safety mechanism)
        check_state_timeout(&sensor_data);
        
        // Flight logic (freefall, apogee detection)
        handle_flight_logic(&sensor_data);
        
        // Deployment control
        handle_deployment(&sensor_data);
        
        // ===================================================================
        // MEDIUM PRIORITY: Telemetry (every 2 loops = ~40ms at 50Hz)
        // ===================================================================
        
        if (loop_counter % 2 == 0) {
            telemetry_send(&sensor_data);
        }
        
        // ===================================================================
        // LOW PRIORITY: GPS and Logging (less frequent)
        // ===================================================================
        
        // GPS: Read every 10 loops (~500ms at 50Hz, ~200ms at 100Hz)
        // GPS updates at ~1Hz, no need to check every loop
        if (loop_counter % 10 == 0) {
            gps_read(&gps_data);
            sensor_data.gps_valid = gps_data.valid;
            sensor_data.gps_sats = gps_data.satellites;
            // Update GPS health
            health_update_gps(gps_data.valid, sensor_data.timestamp_ms);
        }
        
        // SD Card Logging: Write every 5 loops (~100ms) to prevent blocking
        if (loop_counter % 5 == 0) {
            logger_write(&sensor_data);
        }
        
        // ===================================================================
        // Housekeeping
        // ===================================================================
        
        // Check for sensor timeouts
        health_check_timeouts(sensor_data.timestamp_ms);
        
        telemetry_process();
        process_commands();
        update_led();
        
        // CRITICAL FIX (Issue 5): Reset watchdog timer to prevent system reset
        watchdog_update();
        
        // Increment loop counter
        loop_counter++;

        uint32_t elapsed_ms = to_ms_since_boot(get_absolute_time()) - loop_start_ms;
        if (elapsed_ms < MAIN_LOOP_INTERVAL_MS) {
            sleep_ms(MAIN_LOOP_INTERVAL_MS - elapsed_ms);
        }
    }

    return 0;
}
