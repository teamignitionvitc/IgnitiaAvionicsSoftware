/**
 * @file main.c
 * @brief Ignitia CanSat Avionics - Drone Drop Mode
 * 
 * RP2040-Zero based flight computer
 * Sensors: MPU6050, BME280, NEO-M8M GPS
 * Actuator: Servo for parachute deployment
 * Telemetry: UART radio on Core 1 + USB debug on Core 0
 * 
 * Flight Profile (altitude-only transitions):
 *   1. IDLE    - Power on, calibrating
 *   2. ARMED   - Attached to drone, ascending (altitude > 1m)
 *   3. APOGEE  - Peak altitude detected, altitude dropping
 *   4. FREEFALL- Confirmed descent, parachute deploys
 *   5. DEPLOYED- Parachute released, descending
 *   6. LANDED  - On ground (altitude near baseline for 3s)
 * 
 * Core 0: Sensor polling, state machine, deployment, USB debug
 * Core 1: UART telemetry TX (GP0/GP1 at 115200 baud)
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include "config.h"
#include "mpu6050.h"
#include "bme280.h"
#include "neo_m8m.h"
#include "servo.h"
#include "nrf_radio.h"
#include "guva_hw837.h"
#include "ws2812.h"
#include "buzzer.h"
#include "flight_state.h"
#include "deployment.h"
#include "telemetry.h"
#include "sensor_fusion.h"
#include "sd_logger.h"

// =============================================================================
// TELEMETRY SNAPSHOT — Shared between Core 0 (writer) and Core 1 (reader)
// =============================================================================
// Lock-free synchronization: Core 0 increments sequence before and after writing.
// Core 1 reads seq1, copies data, reads seq2. If seq1 == seq2 and even → valid.
typedef struct {
    volatile uint32_t sequence;       // Incremented by Core 0 before/after write
    float altitude;
    uint8_t state;
    float temperature;
    float pressure;
    float humidity;
    float baro_altitude;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float imu_temperature;
    double latitude, longitude;
    float gps_altitude;
    int gps_satellites;
    float gps_speed;
    int gps_fix_quality;
    float velocity;
    float max_altitude;
    float baseline_altitude;
    float uv_voltage;
    bool deployed;
    uint8_t gp12_value;           // Digital sensor on GP12 (0 or 1)
} TelemetrySnapshot;

static volatile TelemetrySnapshot telem_shared;

// =============================================================================
// CORE 1 — UART Telemetry + SD Card Logging
// =============================================================================
static void core1_telemetry_loop(void) {
    // Initialize UART0 on GP0 (TX) and GP1 (RX) at 115200 baud.
    uart_init(NRF_UART, NRF_BAUDRATE);
    gpio_set_function(NRF_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(NRF_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(NRF_UART, false, false);
    uart_set_format(NRF_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(NRF_UART, true);

    // Initialize SD card logger
    bool sd_ok = sd_logger_init();
    if (!sd_ok) {
        printf("[CORE1] SD card not available — UART only\r\n");
    }

    uint32_t last_flush = to_ms_since_boot(get_absolute_time());
    uint32_t last_uart_tx = 0;
    uint32_t last_tx_time = to_ms_since_boot(get_absolute_time());
    float measured_hz = 0.0f;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Read snapshot with sequence-counter validation.
        TelemetrySnapshot snap;
        uint32_t seq1, seq2;
        do {
            seq1 = telem_shared.sequence;
            __dmb();
            memcpy((void*)&snap, (const void*)&telem_shared, sizeof(TelemetrySnapshot));
            __dmb();
            seq2 = telem_shared.sequence;
        } while (seq1 != seq2 || (seq1 & 1));

        // Format CSV line (shared by both SD and UART).
        char csv_packet[400];
        int len = snprintf(csv_packet, sizeof(csv_packet),
            "%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%d,%.2f,%d,%.2f,%.2f,%.3f,%d,%d\n",
            (unsigned long)now,//1 time_ms
            (int)snap.state,//2 state
            snap.altitude,//3
            snap.temperature,//4
            snap.pressure / 100.0f,//5
            snap.baro_altitude,//6
            snap.accel_x,//7
            snap.accel_y,//8
            snap.accel_z,//9
            snap.gyro_x,//10
            snap.gyro_y,//11
            snap.gyro_z,//12
            snap.latitude,//13
            snap.longitude,//14
            snap.gps_altitude,//15
            snap.gps_satellites,//16
            snap.gps_speed,//17
            snap.gps_fix_quality,//18
            snap.velocity,//19
            snap.max_altitude,//20
            snap.uv_voltage,//21
            snap.deployed ? 1 : 0,//22
            snap.gp12_value//23
        );

        // --- SD CARD: log every cycle (fast, SD_LOG_RATE_HZ) ---
        if (sd_ok && sd_logger_is_ready()) {
            sd_logger_write_line(csv_packet);

            // Periodic flush
            if ((now - last_flush) >= SD_LOG_FLUSH_INTERVAL_MS) {
                sd_logger_flush();
                last_flush = now;
            }
        }

        // --- UART TX: only at TELEMETRY_INTERVAL_MS (slow) ---
        if ((now - last_uart_tx) >= TELEMETRY_INTERVAL_MS) {
            last_uart_tx = now;

            // UART transmit with timeout.
            size_t sent = 0;
            uint32_t start = to_ms_since_boot(get_absolute_time());
            while (sent < (size_t)len) {
                if (uart_is_writable(NRF_UART)) {
                    uart_putc_raw(NRF_UART, csv_packet[sent]);
                    sent++;
                }
                if ((to_ms_since_boot(get_absolute_time()) - start) > UART_TX_TIMEOUT_MS) {
                    break;
                }
            }

            // Confirm UART TX on USB debug
            if (sent > 0) {
                uint32_t tx_time = to_ms_since_boot(get_absolute_time());
                uint32_t dt = tx_time - last_tx_time;
                if (dt > 0) {
                    measured_hz = 1000.0f / (float)dt;
                }
                last_tx_time = tx_time;
                printf("TX:%d bytes @ %.1f Hz\r\n", (int)sent, measured_hz);
            }
        }

        // Loop at SD_LOG_INTERVAL_MS (20ms = 50 Hz)
        sleep_ms(SD_LOG_INTERVAL_MS);
    }
}

// UV sensor reading (updated in process_sensors, read by telemetry snapshot)
static float uv_voltage = 0.0f;

// Helper: Core 0 updates the shared telemetry snapshot.
static void update_telemetry_snapshot(
    const FlightStateContext *flight,
    const BME280_Data *env,
    const MPU6050_Data *imu,
    const GPS_Data *gps,
    bool bme_available,
    bool mpu_available
) {
    // Odd sequence = "write in progress".
    telem_shared.sequence++;
    __dmb();

    telem_shared.altitude = flight->current_altitude;
    telem_shared.state = (uint8_t)flight_state_get(flight);
    telem_shared.velocity = flight->current_velocity;
    telem_shared.max_altitude = flight->max_altitude;
    telem_shared.baseline_altitude = flight->baseline_altitude;
    telem_shared.deployed = flight->deployed;

    if (bme_available && env) {
        telem_shared.temperature = env->temperature;
        telem_shared.pressure = env->pressure;
        telem_shared.humidity = env->humidity;
        telem_shared.baro_altitude = env->altitude;
    }

    if (mpu_available && imu) {
        telem_shared.accel_x = imu->accel_x;
        telem_shared.accel_y = imu->accel_y;
        telem_shared.accel_z = imu->accel_z;
        telem_shared.gyro_x = imu->gyro_x;
        telem_shared.gyro_y = imu->gyro_y;
        telem_shared.gyro_z = imu->gyro_z;
        telem_shared.imu_temperature = imu->temperature;
    }

    if (gps) {
        telem_shared.latitude = gps->latitude;
        telem_shared.longitude = gps->longitude;
        telem_shared.gps_altitude = gps->altitude;
        telem_shared.gps_satellites = gps->satellites;
        telem_shared.gps_speed = gps->speed;
        telem_shared.gps_fix_quality = gps->fix_quality;
    }

    // UV sensor (use value already read in process_sensors)
    telem_shared.uv_voltage = uv_voltage;
    telem_shared.gp12_value = gpio_get(SENSOR_GP12_PIN) ? 1 : 0;

    __dmb();
    // Even sequence = "write complete".
    telem_shared.sequence++;
}

// =============================================================================
// CORE 0 — Sensor polling, state machine, deployment, USB debug
// =============================================================================

// Global state
static FlightStateContext flight_ctx;
static FusionState fusion_state;
static MPU6050_Data imu_data;
static BME280_Data env_data;
static GPS_Data gps_data;

// Sensor availability
static bool sensor_mpu_available = false;
static bool sensor_bme_available = false;
static bool sensor_mpu_direct_mode = false;
static uint32_t mpu_read_fail_count = 0;
static uint16_t mpu_consecutive_failures = 0;
static bool imu_runtime_zero_valid = false;
static float imu_bias_ax = 0.0f;
static float imu_bias_ay = 0.0f;
static float imu_bias_az = 0.0f;
static float imu_bias_gx = 0.0f;
static float imu_bias_gy = 0.0f;
static float imu_bias_gz = 0.0f;

// Timing
static uint32_t last_sensor_read = 0;
static uint32_t last_gps_read = 0;
static uint32_t last_debug_print = 0;
static uint32_t last_mpu_retry = 0;
static uint32_t last_mpu_fail_log = 0;

// Debug stream control
static bool fast_stream_enabled = true;
static uint32_t debug_print_interval_ms = 50;
static FlightState last_logged_state = STATE_ERROR;

#define STATE_EVENT_BUFFER_SIZE 16
static FlightState state_event_buffer[STATE_EVENT_BUFFER_SIZE];
static uint32_t state_event_time_ms[STATE_EVENT_BUFFER_SIZE];
static uint8_t state_event_index = 0;
static uint8_t state_event_count = 0;

static void init_peripherals(void) {
    // WS2812B RGB LED (RP2040-Zero built-in on GP16)
    ws2812_init();
    ws2812_blue();  // Blue = initializing

    // Passive buzzer (PWM-driven)
    buzzer_init();

    // External status LEDs (optional)
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);
    gpio_put(STATUS_LED_PIN, 0);
    gpio_init(CAL_LED_PIN);
    gpio_set_dir(CAL_LED_PIN, GPIO_OUT);
    gpio_put(CAL_LED_PIN, 0);

    // Digital sensor on GP12 (input)
    gpio_init(SENSOR_GP12_PIN);
    gpio_set_dir(SENSOR_GP12_PIN, GPIO_IN);
    gpio_pull_down(SENSOR_GP12_PIN);
}

static void indicate_phase(uint8_t phase) {
    switch (phase) {
        case 1: // Boot/startup
            ws2812_blue();
            buzzer_tone(1000, 100);
            break;
        case 2: // MPU init/calibration
            ws2812_set_rgb(0, 0, 40);  // Brighter blue
            buzzer_tone(1200, 80);
            break;
        case 3: // BMP init/calibration
            ws2812_set_rgb(0, 15, 30); // Cyan-ish
            buzzer_tone(1400, 80);
            break;
        case 4: // Settling
            ws2812_set_rgb(20, 10, 0); // Amber
            break;
        case 5: // Ready — play startup melody
            ws2812_green();
            buzzer_startup_melody();
            break;
        default:
            break;
    }
}

static void push_state_event(FlightState state, uint32_t now_ms) {
    state_event_buffer[state_event_index] = state;
    state_event_time_ms[state_event_index] = now_ms;
    state_event_index = (uint8_t)((state_event_index + 1) % STATE_EVENT_BUFFER_SIZE);
    if (state_event_count < STATE_EVENT_BUFFER_SIZE) state_event_count++;
}

static void print_state_event_buffer(uint8_t max_entries) {
    if (state_event_count == 0) {
        printf("StateBuf: empty\r\n");
        return;
    }

    uint8_t n = state_event_count;
    if (n > max_entries) n = max_entries;
    printf("StateBuf:");
    for (uint8_t i = 0; i < n; i++) {
        int idx = (int)state_event_index - 1 - i;
        while (idx < 0) idx += STATE_EVENT_BUFFER_SIZE;
        printf(" [%lu:%s]",
            (unsigned long)state_event_time_ms[idx],
            flight_state_name(state_event_buffer[idx]));
    }
    printf("\r\n");
}

static void i2c_boot_probe(i2c_inst_t *port, uint sda_pin, uint scl_pin, const char *name) {
#if DEBUG_ENABLE
    i2c_init(port, 100000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    // Give bus time to stabilize
    sleep_ms(10);

    printf("I2C probe %s (SDA=GP%u, SCL=GP%u):\r\n", name, sda_pin, scl_pin);
    bool found_any = false;
    
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy = 0;
        int r = i2c_read_timeout_us(port, addr, &dummy, 1, false, 2000);
        if (r >= 0) {
            // Device found - try to identify it by reading common ID registers
            const char *device_name = "unknown";
            uint8_t id_val = 0;
            
            if (addr == 0x68 || addr == 0x69) {
                // MPU family - read WHO_AM_I (0x75)
                uint8_t reg = 0x75;
                if (i2c_write_timeout_us(port, addr, &reg, 1, true, 2000) == 1) {
                    sleep_us(50);
                    if (i2c_read_timeout_us(port, addr, &id_val, 1, false, 2000) == 1) {
                        switch (id_val) {
                            case 0x68: device_name = "MPU6050"; break;
                            case 0x70: device_name = "MPU6500"; break;
                            case 0x71: device_name = "MPU9250"; break;
                            case 0x73: device_name = "MPU9255"; break;
                            case 0x11: device_name = "ICM20600"; break;
                            case 0x12: device_name = "ICM20602"; break;
                            case 0xAF: device_name = "ICM20608"; break;
                            default: device_name = "MPU?"; break;
                        }
                    }
                }
            } else if (addr == 0x76 || addr == 0x77) {
                // BMP/BME family - read chip ID (0xD0)
                uint8_t reg = 0xD0;
                if (i2c_write_timeout_us(port, addr, &reg, 1, true, 2000) == 1) {
                    sleep_us(50);
                    if (i2c_read_timeout_us(port, addr, &id_val, 1, false, 2000) == 1) {
                        switch (id_val) {
                            case 0x58: device_name = "BMP280"; break;
                            case 0x60: device_name = "BME280"; break;
                            case 0x61: device_name = "BME688"; break;
                            default: device_name = "BMx280?"; break;
                        }
                    }
                }
            }
            
            printf("  0x%02X: %s", addr, device_name);
            if (id_val != 0) {
                printf(" (ID=0x%02X)", id_val);
            }
            printf("\r\n");
            found_any = true;
        }
    }
    
    if (!found_any) {
        printf("  (no devices found)\r\n");
    }
    printf("\r\n");

    i2c_deinit(port);
#else
    (void)port;
    (void)sda_pin;
    (void)scl_pin;
    (void)name;
#endif
}

static void i2c_runtime_probe_all(void) {
#if DEBUG_ENABLE
    printf("\r\n=== I2C Runtime Probe ===\r\n");
    i2c_boot_probe(i2c1, 2, 3, "i2c1");
    i2c_boot_probe(i2c0, 4, 5, "i2c0");

    // Restore configured I2C bus used by drivers.
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("Restored active I2C bus: SDA=GP%d, SCL=GP%d\r\n", I2C_SDA_PIN, I2C_SCL_PIN);
    printf("=========================\r\n\r\n");
#endif
}

static void i2c_reset_active_bus(void) {
    i2c_deinit(I2C_PORT);
    sleep_us(200);
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

// Try to recover a stuck I2C bus by clocking SCL manually
static void i2c_bus_recovery(void) {
#if DEBUG_ENABLE
    printf("Attempting I2C bus recovery...\r\n");
#endif
    
    // Deinit I2C and use GPIO bit-banging
    i2c_deinit(I2C_PORT);
    
    gpio_init(I2C_SCL_PIN);
    gpio_init(I2C_SDA_PIN);
    gpio_set_dir(I2C_SCL_PIN, GPIO_OUT);
    gpio_set_dir(I2C_SDA_PIN, GPIO_IN);
    gpio_pull_up(I2C_SDA_PIN);
    
    // Clock SCL up to 9 times to release any stuck slave
    for (int i = 0; i < 9; i++) {
        gpio_put(I2C_SCL_PIN, 0);
        sleep_us(5);
        gpio_put(I2C_SCL_PIN, 1);
        sleep_us(5);
        
        // Check if SDA is high (released)
        if (gpio_get(I2C_SDA_PIN)) {
            break;
        }
    }
    
    // Generate STOP condition
    gpio_set_dir(I2C_SDA_PIN, GPIO_OUT);
    gpio_put(I2C_SDA_PIN, 0);
    sleep_us(5);
    gpio_put(I2C_SCL_PIN, 1);
    sleep_us(5);
    gpio_put(I2C_SDA_PIN, 1);
    sleep_us(5);
    
    // Reinit I2C
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    sleep_ms(10);
#if DEBUG_ENABLE
    printf("I2C bus recovery complete\r\n");
#endif
}

static bool mpu_direct_read_addr(uint8_t addr, MPU6050_Data *data) {
    if (!data) return false;

    uint8_t reg;
    uint8_t buf[2];

    // Wake device (PWR_MGMT_1 = 0x00). Ignore failure; reads below will decide success.
    uint8_t wake_cmd[2] = {0x6B, 0x00};
    (void)i2c_write_timeout_us(I2C_PORT, addr, wake_cmd, 2, false, 20000);
    sleep_us(100);

    // Helper macro to read a 16-bit register pair exactly like MicroPython writeto_then_readfrom.
#define READ_WORD(REG, OUT) \
    do { \
        reg = (REG); \
        if (i2c_write_timeout_us(I2C_PORT, addr, &reg, 1, true, 20000) != 1) return false; \
        sleep_us(50); \
        if (i2c_read_timeout_us(I2C_PORT, addr, buf, 2, false, 20000) != 2) return false; \
        sleep_us(50); \
        (OUT) = (int16_t)((buf[0] << 8) | buf[1]); \
    } while (0)

    int16_t ax, ay, az, tr, gx, gy, gz;
    READ_WORD(0x3B, ax);
    READ_WORD(0x3D, ay);
    READ_WORD(0x3F, az);
    READ_WORD(0x41, tr);
    READ_WORD(0x43, gx);
    READ_WORD(0x45, gy);
    READ_WORD(0x47, gz);

#undef READ_WORD

    data->accel_x_raw = ax;
    data->accel_y_raw = ay;
    data->accel_z_raw = az;
    data->temp_raw = tr;
    data->gyro_x_raw = gx;
    data->gyro_y_raw = gy;
    data->gyro_z_raw = gz;

    data->accel_x = data->accel_x_raw / 16384.0f;
    data->accel_y = data->accel_y_raw / 16384.0f;
    data->accel_z = data->accel_z_raw / 16384.0f;
    data->gyro_x = data->gyro_x_raw / 131.0f;
    data->gyro_y = data->gyro_y_raw / 131.0f;
    data->gyro_z = data->gyro_z_raw / 131.0f;
    data->temperature = (data->temp_raw / 333.87f) + 21.0f;

    data->accel_magnitude = sqrtf(
        data->accel_x * data->accel_x +
        data->accel_y * data->accel_y +
        data->accel_z * data->accel_z
    );

    return true;
}

// Direct MPU read path that mirrors the known-working MicroPython flow.
static bool mpu6500_direct_read(MPU6050_Data *data) {
    // Try configured/default address first, then the alternate one.
    if (mpu_direct_read_addr(0x68, data)) return true;
    if (mpu_direct_read_addr(0x69, data)) return true;
    return false;
}

static void apply_runtime_imu_zero(MPU6050_Data *data) {
    if (!data || !imu_runtime_zero_valid) return;

    data->accel_x -= imu_bias_ax;
    data->accel_y -= imu_bias_ay;
    data->accel_z -= imu_bias_az;
    data->gyro_x -= imu_bias_gx;
    data->gyro_y -= imu_bias_gy;
    data->gyro_z -= imu_bias_gz;

    data->accel_magnitude = sqrtf(
        data->accel_x * data->accel_x +
        data->accel_y * data->accel_y +
        data->accel_z * data->accel_z
    );
}

static bool calibrate_runtime_imu_zero(uint16_t samples) {
    if (!sensor_mpu_available || samples == 0) return false;

    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    uint16_t collected = 0;

    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_Data sample = {0};
        bool ok = sensor_mpu_direct_mode ?
            mpu6500_direct_read(&sample) :
            mpu6050_read(&sample);

        if (ok) {
            sum_ax += sample.accel_x;
            sum_ay += sample.accel_y;
            sum_az += sample.accel_z;
            sum_gx += sample.gyro_x;
            sum_gy += sample.gyro_y;
            sum_gz += sample.gyro_z;
            collected++;
        }

        sleep_ms(5);
    }

    if (collected < (samples / 2)) {
        return false;
    }

    imu_bias_ax = sum_ax / collected;
    imu_bias_ay = sum_ay / collected;
    imu_bias_az = (sum_az / collected) - 1.0f;  // Keep gravity on Z at ~1g while stationary.
    imu_bias_gx = sum_gx / collected;
    imu_bias_gy = sum_gy / collected;
    imu_bias_gz = sum_gz / collected;
    imu_runtime_zero_valid = true;
    return true;
}

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
    sleep_ms(2000);  // Longer wait for USB serial to connect
    
    printf("\r\n\r\n\r\n");
    printf("========================================\r\n");
    printf("  %s v%d.%d.%d\r\n", FIRMWARE_NAME, 
           FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH);
    printf("========================================\r\n");
    
    // LEDs, WS2812B, buzzer
    init_peripherals();
    indicate_phase(1);
    
    // Early diagnostics: detect where I2C devices are physically present.
    printf("\r\n=== I2C BUS SCAN ===\r\n");
    i2c_boot_probe(i2c1, 2, 3, "i2c1");
    i2c_boot_probe(i2c0, 4, 5, "i2c0");
    printf("====================\r\n\r\n");

    // I2C for sensors
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("I2C initialized (SDA=GP%d, SCL=GP%d, %d baud)\r\n", 
           I2C_SDA_PIN, I2C_SCL_PIN, I2C_BAUDRATE);
    
    // Try bus recovery before MPU init (in case bus is stuck)
    i2c_bus_recovery();
    indicate_phase(1);
    
    // Initialize MPU6500
    printf("Initializing MPU6500...\r\n");
    indicate_phase(2);
    sensor_mpu_available = mpu6050_init();
    if (sensor_mpu_available) {
        sensor_mpu_direct_mode = false;
        printf("MPU6500: OK (%s)\r\n", mpu6050_get_type_name());
    } else {
        // WHO_AM_I may be unreliable on some clones; try raw register path directly.
        if (mpu6500_direct_read(&imu_data)) {
            sensor_mpu_available = true;
            sensor_mpu_direct_mode = true;
            printf("MPU6500: ONLINE via direct raw reads\r\n");
        } else {
            printf("MPU6500: FAILED\r\n");
            printf("Will retry in background\r\n");
        }
    }
    
    // Reset I2C peripheral state before probing next sensor.
    i2c_reset_active_bus();
    
    // Initialize BME280
    printf("Initializing BME280... ");
    indicate_phase(3);
    sensor_bme_available = bme280_init();
    if (sensor_bme_available) {
        printf("OK\r\n");
    } else {
        printf("SKIPPED (I2C bus issue)\r\n");
    }
    
    // Initialize GPS
    printf("Initializing GPS... ");
    if (!gps_init()) {
        printf("FAILED\r\n");
        return false;
    }
    printf("OK\r\n");
    
    // Wait for GPS lock (NEO-M8M-0-10)
    gps_wait_for_lock(GPS_LOCK_TIMEOUT_MS);
    
    // Initialize servo
    printf("Initializing Servo... ");
    if (!servo_init()) {
        printf("FAILED\r\n");
        return false;
    }
    printf("OK\r\n");
    
    // Initialize deployment system
    deployment_init();

    // Initialize UV sensor
    printf("Initializing GUVA-HW837 UV... ");
    guva_hw837_init();
    printf("OK\r\n");

    // Allow sensors to thermally/electrically settle before calibration.
    printf("Settling sensors for %d ms...\r\n", SENSOR_SETTLE_TIME_MS);
    uint32_t settle_start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - settle_start) < SENSOR_SETTLE_TIME_MS) {
        indicate_phase(4);
        sleep_ms(140);
    }
    
    // Calibrate sensors (only if available)
    if (sensor_mpu_available) {
        printf("Calibrating MPU6500...\r\n");
        indicate_phase(2);
        if (!sensor_mpu_direct_mode && mpu6050_calibrate(100)) {
            printf("  Driver offset calibration: OK\r\n");
        }

        printf("Zeroing IMU (startup bias)...\r\n");
        if (calibrate_runtime_imu_zero(80)) {
            printf("  Runtime zero calibration: OK\r\n");
        } else {
            printf("  Runtime zero calibration: FAILED\r\n");
        }
    }
    if (sensor_bme_available) {
        printf("Calibrating BME280...\r\n");
        indicate_phase(3);
        if (bme280_calibrate_ground(20)) {
            printf("  OK (ground pressure: %.0f Pa)\r\n", bme280_get_ground_pressure());
        }
    }
    
    gpio_put(STATUS_LED_PIN, 0);
    gpio_put(CAL_LED_PIN, 0);
    indicate_phase(5);
    printf("Hardware initialization complete\r\n\r\n");
    
    return true;
}

static void process_sensors(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // If IMU is unavailable, keep trying to bring it online in the background.
    if (!sensor_mpu_available && (now - last_mpu_retry) >= 2000) {
        last_mpu_retry = now;
        // First try direct raw read path (matches working MicroPython behavior).
        if (mpu6500_direct_read(&imu_data)) {
            sensor_mpu_available = true;
            sensor_mpu_direct_mode = true;
            mpu_consecutive_failures = 0;
            printf("MPU direct-read path online\r\n");
        } else if (mpu6050_init()) {
            sensor_mpu_available = true;
            sensor_mpu_direct_mode = false;
            mpu_consecutive_failures = 0;
            printf("MPU reinit successful\r\n");
            // Optional quick calibration after successful hot-reinit.
            mpu6050_calibrate(50);
        }
    }
    
    // Read IMU and barometer at high rate (only if available)
    if ((now - last_sensor_read) >= SENSOR_READ_INTERVAL_MS) {
        last_sensor_read = now;
        
        if (sensor_mpu_available) {
            bool imu_ok = sensor_mpu_direct_mode ?
                mpu6500_direct_read(&imu_data) :
                mpu6050_read(&imu_data);

            if (!imu_ok) {
                mpu_read_fail_count++;
                mpu_consecutive_failures++;

                // Fall back to direct raw register reads before dropping IMU.
                if (!mpu6500_direct_read(&imu_data)) {
                    sensor_mpu_available = false;
                    sensor_mpu_direct_mode = false;

                    // Escalate recovery after repeated failures.
                    if (mpu_consecutive_failures >= 3) {
                        i2c_bus_recovery();
                        i2c_reset_active_bus();
                    }

                    if ((now - last_mpu_fail_log) >= 500) {
                        last_mpu_fail_log = now;
                        printf("MPU read failed x%u (total=%lu); unavailable, retrying\r\n",
                            (unsigned)mpu_consecutive_failures,
                            (unsigned long)mpu_read_fail_count);
                    }
                } else {
                    sensor_mpu_direct_mode = true;
                    mpu_consecutive_failures = 0;
                    apply_runtime_imu_zero(&imu_data);
                    if ((now - last_mpu_fail_log) >= 500) {
                        last_mpu_fail_log = now;
                        printf("MPU recovered via direct-read fallback\r\n");
                    }
                }
            } else {
                mpu_consecutive_failures = 0;
                apply_runtime_imu_zero(&imu_data);
            }
        }
        if (sensor_bme_available) {
            if (!bme280_read(&env_data)) {
                sensor_bme_available = false;
                printf("BME280 read failed; marking unavailable\r\n");
            }
        }

        // Read UV sensor (analog, always available)
        uv_voltage = guva_hw837_read_uv();
        
        // Update fusion with whichever sensors are available.
        const BME280_Data *baro_ptr = sensor_bme_available ? &env_data : NULL;
        const MPU6050_Data *imu_ptr = sensor_mpu_available ? &imu_data : NULL;
        if (baro_ptr || imu_ptr) {
            fusion_update(&fusion_state, baro_ptr, imu_ptr);
        }

        // Always run state machine so INIT->IDLE transition and logging continue.
        flight_state_update(&flight_ctx,
            fusion_get_altitude(&fusion_state),
            fusion_get_velocity(&fusion_state),
            sensor_mpu_available ? imu_data.accel_magnitude : 1.0f);

        // Update shared telemetry snapshot for Core 1.
        update_telemetry_snapshot(&flight_ctx, &env_data, &imu_data, &gps_data,
            sensor_bme_available, sensor_mpu_available);

        // Emit state transition logs with key fusion values.
        FlightState current_state = flight_state_get(&flight_ctx);
        if (current_state != last_logged_state) {
            FlightState prev_state = last_logged_state;
            last_logged_state = current_state;
            push_state_event(current_state, now);
            printf("STATE -> %s | Alt=%.1f m Vel=%.2f m/s Accel=%.2f g | ax=%.2f ay=%.2f az=%.2f am=%.2f\r\n",
                flight_state_name(current_state),
                flight_ctx.current_altitude,
                flight_ctx.current_velocity,
                flight_ctx.current_accel,
                imu_data.accel_x,
                imu_data.accel_y,
                imu_data.accel_z,
                imu_data.accel_magnitude);
            
            // Auto-arm deployment when flight state becomes ARMED
            if (current_state == STATE_ARMED && prev_state != STATE_ARMED) {
                deployment_arm();
                printf("Deployment system ARMED\r\n");
            }
        }
    }
    
    // Read GPS at lower rate
    if ((now - last_gps_read) >= GPS_READ_INTERVAL_MS) {
        last_gps_read = now;
        gps_read(&gps_data);
    }
}

static void process_deployment(void) {
    if (flight_state_should_deploy(&flight_ctx)) {
        printf("DEPLOYING PARACHUTE...\r\n");
        telemetry_send_status("DEPLOYING PARACHUTE");
        
        if (deployment_trigger()) {
            flight_state_mark_deployed(&flight_ctx);
            printf("DEPLOYMENT SUCCESS\r\n");
            telemetry_send_status("DEPLOYMENT SUCCESS");
        } else {
            printf("DEPLOYMENT FAILED - trigger returned false\r\n");
            telemetry_send_status("DEPLOYMENT FAILED");
        }
        // Debug: Confirm main loop continues after deployment
        printf("[DEBUG] process_deployment() completed. State: %s\r\n", flight_state_name(flight_state_get(&flight_ctx)));
    }
}

static void print_sensor_data(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    if ((now - last_debug_print) >= debug_print_interval_ms) {
        last_debug_print = now;

        // Print ALL telemetry data (same fields as UART CSV)
        printf("T=%lu,ST=%s,ALT=%.2f,VEL=%.2f,MAX=%.2f,BASE=%.2f,"
               "TEMP=%.2f,PRESS=%.2f,HUM=%.1f,BALT=%.2f,"
               "AX=%.2f,AY=%.2f,AZ=%.2f,GX=%.1f,GY=%.1f,GZ=%.1f,"
               "UV=%.3f,DEP=%d\r\n",
            (unsigned long)now,
            flight_state_name(flight_state_get(&flight_ctx)),
            fusion_get_altitude(&fusion_state),
            fusion_get_velocity(&fusion_state),
            flight_ctx.max_altitude,
            flight_ctx.baseline_altitude,
            env_data.temperature,
            env_data.pressure / 100.0f,
            env_data.humidity,
            env_data.altitude,
            imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
            imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
            uv_voltage,
            flight_ctx.deployed);
    }
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
            
        case 'd':  // Reset and re-arm
            flight_state_init(&flight_ctx);
            flight_state_arm(&flight_ctx);
            deployment_disarm();
            telemetry_send_status("RE-ARMED");
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
            printf("$STS,State=%s,Alt=%.1f,Vel=%.1f,Accel=%.2f,Deployed=%d,BaseAlt=%.1f,MaxAlt=%.1f\r\n",
                flight_state_name(flight_state_get(&flight_ctx)),
                flight_ctx.current_altitude,
                flight_ctx.current_velocity,
                flight_ctx.current_accel,
                flight_ctx.deployed,
                flight_ctx.baseline_altitude,
                flight_ctx.max_altitude);
            print_state_event_buffer(8);
            break;
        
        case 'v':  // Verbose debug output
            printf("\r\n=== Sensor Debug Output ===\r\n");
            printf("Sensors Available: MPU6050=%s, BME280=%s\r\n",
                sensor_mpu_available ? "YES" : "NO",
                sensor_bme_available ? "YES" : "NO");
            printf("IMU mode: %s | MPU read failures total=%lu\r\n",
                sensor_mpu_direct_mode ? "DIRECT" : "DRIVER",
                (unsigned long)mpu_read_fail_count);
            printf("IMU zero: %s | bias ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f\r\n",
                imu_runtime_zero_valid ? "OK" : "NO",
                imu_bias_ax, imu_bias_ay, imu_bias_az,
                imu_bias_gx, imu_bias_gy, imu_bias_gz);
            
            if (sensor_mpu_available) {
                printf("IMU: Accel=(%.2f,%.2f,%.2f) m/s², Gyro=(%.1f,%.1f,%.1f) deg/s, Temp=%.1f°C\r\n",
                    imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                    imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                    imu_data.temperature);
            }
            
            if (sensor_bme_available) {
                printf("ENV: Temp=%.2f°C, Press=%.1f hPa, Humidity=%.1f%%, Alt=%.1f m\r\n",
                    env_data.temperature, env_data.pressure / 100.0f,
                    env_data.humidity, env_data.altitude);
            }
            
            if (gps_data.valid) {
                printf("GPS: Lat=%.6f, Lon=%.6f, Alt=%.1f m, Sats=%d, Speed=%.2f m/s, Fix=%d\r\n",
                    gps_data.latitude, gps_data.longitude,
                    gps_data.altitude, gps_data.satellites,
                    gps_data.speed, gps_data.fix_quality);
            } else {
                printf("GPS: NO FIX (searching...)\r\n");
            }
            
            printf("Flight: State=%s, Alt=%.1f m, Vel=%.1f m/s, Accel=%.2f g\r\n",
                flight_state_name(flight_state_get(&flight_ctx)),
                flight_ctx.current_altitude,
                flight_ctx.current_velocity,
                flight_ctx.current_accel);
            printf("Baseline: %.2f m, Max: %.2f m, Apogee confirm: %u\r\n",
                flight_ctx.baseline_altitude,
                flight_ctx.max_altitude,
                (unsigned)flight_ctx.apogee_confirm_count);
            printf("Altitude zero: %s, ref=%.2f m, samples=%u\r\n\r\n",
                fusion_state.altitude_zeroed ? "YES" : "NO",
                fusion_state.altitude_reference,
                (unsigned)fusion_state.altitude_ref_samples);
            break;

        case 'f':  // Toggle fast sensor stream
            fast_stream_enabled = !fast_stream_enabled;
            debug_print_interval_ms = fast_stream_enabled ? 50 : 500;
            printf("Sensor stream: %s (%lu ms interval)\r\n",
                fast_stream_enabled ? "FAST" : "SLOW",
                (unsigned long)debug_print_interval_ms);
            break;

        case 'b':  // Buzzer test
            buzzer_tone(BUZZER_FREQ_INIT, 200);
            break;

        case 'p':  // Probe I2C buses now
            i2c_runtime_probe_all();
            break;
            
        case 'c':  // Calibrate
            telemetry_send_status("CALIBRATING\n");
            if (sensor_mpu_available) {
                if (!sensor_mpu_direct_mode) {
                    mpu6050_calibrate(100);
                }
                calibrate_runtime_imu_zero(80);
            }
            if (sensor_bme_available) bme280_calibrate_ground(20);
            fusion_reset(&fusion_state);
            telemetry_send_status("CALIBRATION DONE\n");
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
            printf("  v - Verbose sensor debug\r\n");
            printf("  f - Toggle sensor stream FAST/SLOW\r\n");
            printf("  p - Probe I2C buses now\r\n");
            printf("  b - Buzzer test\r\n");
            printf("  c - Calibrate sensors\r\n");
            printf("  h - Help\r\n");
            printf("\r\nFlight sequence (altitude-only):\r\n");
            printf("  IDLE -> ARMED -> APOGEE -> FREEFALL -> DEPLOYED -> LANDED\r\n");
            printf("  Core 1: UART telemetry on GP0/GP1 @ 115200 baud\r\n\r\n");
            break;
    }
}

static void update_led(void) {
    static FlightState last_state = STATE_INIT;
    static uint32_t last_landed_beep = 0;
    FlightState state = flight_state_get(&flight_ctx);

    // Update WS2812B LED color based on state
    if (state != last_state) {
        switch (state) {
            case STATE_ARMED:    ws2812_green();  break;  // Green = ready
            case STATE_APOGEE:   ws2812_white();  break;  // White = deploying
            case STATE_DEPLOYED: ws2812_blue();   break;  // Blue = descending
            case STATE_LANDED:   ws2812_yellow(); break;  // Yellow = landed
            case STATE_ERROR:    ws2812_red();    break;  // Red = error
            default:             ws2812_blue();   break;
        }
        last_state = state;
    }

    // Buzzer: only during LANDED (locator beep)
    if (state == STATE_LANDED) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - last_landed_beep) >= 2000) {
            last_landed_beep = now;
            buzzer_tone(BUZZER_FREQ_LANDED, BUZZER_BEEP_MS);
        }
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

    if (!sensor_mpu_available || !sensor_bme_available) {
        printf("Auto-running I2C diagnostics (one or more I2C sensors unavailable)...\r\n");
        i2c_runtime_probe_all();
    }
    
    printf("[DEBUG] Initializing flight state...\r\n");
    flight_state_init(&flight_ctx);
    printf("[DEBUG] Initializing sensor fusion...\r\n");
    fusion_init(&fusion_state);

    // Auto-arm: go directly to ARMED after calibration (no IDLE state)
    flight_state_arm(&flight_ctx);
    ws2812_green();  // Green = armed and ready
    
    // Initialize shared telemetry snapshot.
    memset((void*)&telem_shared, 0, sizeof(TelemetrySnapshot));

    // Launch Core 1 for UART telemetry.
    printf("[CORE1] Launching UART telemetry on Core 1 (GP%d TX, GP%d RX, %d baud)...\r\n",
        NRF_TX_PIN, NRF_RX_PIN, NRF_BAUDRATE);
    multicore_launch_core1(core1_telemetry_loop);
    printf("[CORE1] Telemetry running on Core 1\r\n");
    
    printf("Type 'h' for help\r\n\r\n");
    printf("[DEBUG] Entering main loop (Core 0: sensors + state machine)...\r\n");
    
    // Main loop (Core 0: sensors, state machine, deployment, USB debug)
    while (1) {
        static uint32_t loop_counter = 0;
        if ((loop_counter++ % 1000) == 0) {
            printf("[DEBUG] Main loop alive. State: %s\r\n", flight_state_name(flight_state_get(&flight_ctx)));
        }
        process_sensors();
        process_deployment();
        // Telemetry is handled by Core 1 — no process_telemetry() call here.
        print_sensor_data();
        process_commands();
        update_led();
        // Small delay to prevent CPU hogging
        sleep_us(100);
    }
    
    return 0;
}
