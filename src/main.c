/**
 * @file main.c
 * @brief Ignitia CanSat Avionics - Drone Drop Mode
 * 
 * RP2040-Zero based flight computer
 * Sensors: MPU6050, BME280, NEO-M8M GPS
 * Actuator: Servo for parachute deployment
 * Telemetry: NRF UART radio + USB debug
 * 
 * Flight Profile:
 *   1. IDLE - Power on, calibrating
 *   2. ARMED - Attached to drone, ascending
 *   3. FREEFALL - Dropped, detecting descent
 *   4. DEPLOYED - Parachute released
 *   5. LANDED - On ground
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "config.h"
#include "mpu6050.h"
#include "bme280.h"
#include "neo_m8m.h"
#include "servo.h"
#include "nrf_radio.h"
#include "flight_state.h"
#include "deployment.h"
#include "telemetry.h"
#include "sensor_fusion.h"

// Global state
static FlightStateContext flight_ctx;
static FusionState fusion_state;
static MPU6050_Data imu_data;
static BME280_Data env_data;
static GPS_Data gps_data;

// Timing
static uint32_t last_sensor_read = 0;
static uint32_t last_gps_read = 0;
static uint32_t last_telemetry = 0;

// LED blink patterns
static void led_blink(uint8_t count, uint32_t on_ms, uint32_t off_ms) {
    for (uint8_t i = 0; i < count; i++) {
        gpio_put(STATUS_LED_PIN, 1);
        sleep_ms(on_ms);
        gpio_put(STATUS_LED_PIN, 0);
        sleep_ms(off_ms);
    }
}

static bool init_hardware(void) {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(1000);
    
    printf("\r\n");
    printf("========================================\r\n");
    printf("  %s v%d.%d.%d\r\n", FIRMWARE_NAME, 
           FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH);
    printf("========================================\r\n");
    
    // LED
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);
    gpio_put(STATUS_LED_PIN, 1);
    
    // I2C for sensors
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Initialize MPU6050
    printf("Initializing MPU6050... ");
    if (!mpu6050_init()) {
        printf("FAILED\r\n");
        return false;
    }
    printf("OK\r\n");
    
    // Initialize BME280
    printf("Initializing BME280... ");
    if (!bme280_init()) {
        printf("FAILED\r\n");
        return false;
    }
    printf("OK\r\n");
    
    // Initialize GPS
    printf("Initializing GPS... ");
    if (!gps_init()) {
        printf("FAILED\r\n");
        return false;
    }
    printf("OK\r\n");
    
    // Initialize servo
    printf("Initializing Servo... ");
    if (!servo_init()) {
        printf("FAILED\r\n");
        return false;
    }
    printf("OK\r\n");
    
    // Initialize deployment system
    deployment_init();
    
    // Calibrate sensors
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

static void process_sensors(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Read IMU and barometer at high rate
    if ((now - last_sensor_read) >= SENSOR_READ_INTERVAL_MS) {
        last_sensor_read = now;
        
        mpu6050_read(&imu_data);
        bme280_read(&env_data);
        
        // Update sensor fusion
        fusion_update(&fusion_state, &env_data, &imu_data);
        
        // Update flight state machine
        flight_state_update(&flight_ctx,
            fusion_get_altitude(&fusion_state),
            fusion_get_velocity(&fusion_state),
            imu_data.accel_magnitude);
    }
    
    // Read GPS at lower rate
    if ((now - last_gps_read) >= GPS_READ_INTERVAL_MS) {
        last_gps_read = now;
        gps_read(&gps_data);
    }
}

static void process_deployment(void) {
    if (flight_state_should_deploy(&flight_ctx)) {
        telemetry_send_status("DEPLOYING PARACHUTE");
        
        if (deployment_trigger()) {
            flight_state_mark_deployed(&flight_ctx);
            telemetry_send_status("DEPLOYMENT SUCCESS");
        } else {
            telemetry_send_status("DEPLOYMENT FAILED");
        }
    }
}

static void process_telemetry(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    if ((now - last_telemetry) >= TELEMETRY_INTERVAL_MS) {
        last_telemetry = now;
        
        // Send main telemetry packet via NRF
        telemetry_send_packet(&flight_ctx, &env_data, &imu_data, &gps_data);
        
        // Send GPS separately at lower rate
        static uint32_t last_gps_telem = 0;
        if ((now - last_gps_telem) >= 1000 && gps_data.valid) {
            telemetry_send_gps(&gps_data);
            last_gps_telem = now;
        }
    }
    
    // Process incoming NRF commands
    telemetry_process();
}

static void process_commands(void) {
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) return;
    
    switch (c) {
        case 'a':  // Arm
            flight_state_arm(&flight_ctx);
            deployment_arm();
            telemetry_send_status("ARMED");
            led_blink(3, 100, 100);
            break;
            
        case 'd':  // Disarm
            flight_state_disarm(&flight_ctx);
            deployment_disarm();
            telemetry_send_status("DISARMED");
            break;
            
        case 't':  // Test servo
            telemetry_send_status("SERVO TEST");
            deployment_test();
            break;
            
        case 'r':  // Reset
            telemetry_send_status("RESETTING");
            deployment_reset();
            flight_state_init(&flight_ctx);
            fusion_reset(&fusion_state);
            break;
            
        case 's':  // Status
            printf("$STS,State=%s,Alt=%.1f,Vel=%.1f,Accel=%.2f,Deployed=%d,DropAlt=%.1f\r\n",
                flight_state_name(flight_state_get(&flight_ctx)),
                flight_ctx.current_altitude,
                flight_ctx.current_velocity,
                flight_ctx.current_accel,
                flight_ctx.deployed,
                flight_ctx.drop_altitude);
            break;
            
        case 'c':  // Calibrate
            telemetry_send_status("CALIBRATING");
            mpu6050_calibrate(100);
            bme280_calibrate_ground(20);
            telemetry_send_status("CALIBRATION DONE");
            break;
            
        case '?':
        case 'h':  // Help
            printf("\r\n=== Ignitia CanSat (Drone Drop Mode) ===\r\n");
            printf("Commands:\r\n");
            printf("  a - Arm (attach to drone first!)\r\n");
            printf("  d - Disarm\r\n");
            printf("  t - Test servo\r\n");
            printf("  r - Reset\r\n");
            printf("  s - Status\r\n");
            printf("  c - Calibrate sensors\r\n");
            printf("  h - Help\r\n");
            printf("\r\nFlight sequence:\r\n");
            printf("  IDLE -> ARM -> (drone drop) -> FREEFALL -> DEPLOYED -> LANDED\r\n\r\n");
            break;
    }
}

static void update_led(void) {
    static uint32_t last_blink = 0;
    static bool led_state = false;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    uint32_t interval;
    switch (flight_state_get(&flight_ctx)) {
        case STATE_IDLE:      interval = 1000; break;  // Slow blink
        case STATE_ARMED:     interval = 200;  break;  // Fast blink - ready
        case STATE_FREEFALL:  interval = 50;   break;  // Very fast - falling!
        case STATE_DEPLOYED:  interval = 500;  break;  // Medium - descending
        case STATE_LANDED:    interval = 2000; break;  // Very slow - done
        default:              interval = 100;  break;  // Error
    }
    
    if ((now - last_blink) >= interval) {
        last_blink = now;
        led_state = !led_state;
        gpio_put(STATUS_LED_PIN, led_state);
    }
}

int main(void) {
    // Initialize everything
    if (!init_hardware()) {
        printf("Hardware init failed! Entering error state.\r\n");
        while (1) {
            led_blink(5, 50, 50);
            sleep_ms(500);
        }
    }
    
    // Initialize state machines
    flight_state_init(&flight_ctx);
    fusion_init(&fusion_state);
    
    telemetry_send_status("SYSTEM READY");
    printf("Type 'h' for help\r\n\r\n");
    
    // Main loop
    while (1) {
        process_sensors();
        process_deployment();
        process_telemetry();
        process_commands();
        update_led();
        
        // Small delay to prevent CPU hogging
        sleep_us(100);
    }
    
    return 0;
}
