/**
 * @file telemetry.c
 * @brief Telemetry implementation - USB debug and NRF radio packets
 */

#include "telemetry.h"
#include "config.h"
#include "nrf_radio.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "hardware/uart.h"
#include "hardware/gpio.h"

// UART and LED pin config (matching Python example)
#ifndef SIMPLE_TELEM_UART
#define SIMPLE_TELEM_UART uart0
#endif
#ifndef SIMPLE_TELEM_TX_PIN
#define SIMPLE_TELEM_TX_PIN 0
#endif
#ifndef SIMPLE_TELEM_RX_PIN
#define SIMPLE_TELEM_RX_PIN 1
#endif
#ifndef SIMPLE_TELEM_BAUD
#define SIMPLE_TELEM_BAUD 115200
#endif
#ifndef SIMPLE_TELEM_LED_PIN
#define SIMPLE_TELEM_LED_PIN 16
#endif

static bool simple_telem_uart_initialized = false;
static bool simple_telem_led_initialized = false;

void telemetry_send_simple_packet(uint32_t timestamp, float altitude, float velocity, uint8_t state) {
    // Packet: 0xAA | <IffB> | 0x55 | '\n'
    uint8_t packet[1 + 4 + 4 + 4 + 1 + 1 + 1];
    size_t offset = 0;
    packet[offset++] = 0xAA;
    memcpy(&packet[offset], &timestamp, 4); offset += 4;
    memcpy(&packet[offset], &altitude, 4); offset += 4;
    memcpy(&packet[offset], &velocity, 4); offset += 4;
    packet[offset++] = state;
    packet[offset++] = 0x55;
    packet[offset++] = '\n';

    // Init UART if not done
    if (!simple_telem_uart_initialized) {
        uart_init(SIMPLE_TELEM_UART, SIMPLE_TELEM_BAUD);
        gpio_set_function(SIMPLE_TELEM_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(SIMPLE_TELEM_RX_PIN, GPIO_FUNC_UART);
        simple_telem_uart_initialized = true;
    }

    // Init LED if not done
    if (!simple_telem_led_initialized) {
        gpio_init(SIMPLE_TELEM_LED_PIN);
        gpio_set_dir(SIMPLE_TELEM_LED_PIN, GPIO_OUT);
        gpio_put(SIMPLE_TELEM_LED_PIN, 0);
        simple_telem_led_initialized = true;
    }

    // LED on
    if (simple_telem_led_initialized) gpio_put(SIMPLE_TELEM_LED_PIN, 1);

    uart_write_blocking(SIMPLE_TELEM_UART, packet, sizeof(packet));

    // LED off
    if (simple_telem_led_initialized) gpio_put(SIMPLE_TELEM_LED_PIN, 0);

    // Debug print (always assume TX OK since uart_write_blocking is void)
    printf("[SIMPLE_TELEM] TX OK: %d\r\n", (int)sizeof(packet));
}
void telemetry_init(void) {
    // Initialize NRF radio
    nrf_init();

    // Check NRF radio by sending a test status packet
    const char *test_msg = "NRF TEST";
    printf("[NRF] Checking radio link...\r\n");
    nrf_send_status(test_msg);
    // The result will be printed by nrf_send_status (success/fail)
}

void telemetry_send_packet(const FlightStateContext *flight, const BME280_Data *env,
                           const MPU6050_Data *imu, const GPS_Data *gps) {
    TelemetryPacket pkt = {0};
    
    pkt.timestamp = to_ms_since_boot(get_absolute_time());
    pkt.state = flight_state_get(flight);
    
    // Convert to packet format (scaled integers for smaller packets)
    pkt.altitude = (int16_t)(flight->current_altitude * 10);      // dm
    pkt.velocity = (int16_t)(flight->current_velocity * 100);     // cm/s
    pkt.accel = (int16_t)(flight->current_accel * 1000);          // mg
    
    if (env) {
        pkt.temperature = (int16_t)(env->temperature * 10);       // 0.1°C
    }
    
    if (gps) {
        pkt.gps_sats = gps->satellites;
    }
    
    // Build flags
    pkt.flags = 0;
    if (flight_state_is_armed(flight)) pkt.flags |= FLAG_ARMED;
    if (flight->deployed) pkt.flags |= FLAG_DEPLOYED;
    if (gps && gps->valid) pkt.flags |= FLAG_GPS_FIX;
    
    // Send via NRF radio
    nrf_send_telemetry(&pkt);
    
    // Also print to USB for debugging
    #if DEBUG_ENABLE
    printf("$T,%lu,%s,%.1f,%.1f,%.2f,%d\r\n",
        pkt.timestamp,
        flight_state_name(flight_state_get(flight)),
        flight->current_altitude,
        flight->current_velocity,
        flight->current_accel,
        flight->deployed);
    #endif
}

void telemetry_send_gps(const GPS_Data *gps) {
    if (!gps || !gps->valid) return;
    
    GPSPacket pkt = {0};
    pkt.timestamp = to_ms_since_boot(get_absolute_time());
    pkt.latitude = (int32_t)(gps->latitude * 1e6);
    pkt.longitude = (int32_t)(gps->longitude * 1e6);
    pkt.gps_alt = (int16_t)gps->altitude;
    pkt.gps_sats = gps->satellites;
    pkt.hdop = (uint8_t)(gps->hdop * 10);
    
    nrf_send_gps(&pkt);
    
    #if DEBUG_ENABLE
    printf("$G,%.6f,%.6f,%.0f,%d\r\n",
        gps->latitude, gps->longitude, gps->altitude, gps->satellites);
    #endif
}

void telemetry_send_status(const char *message) {
    if (!message) return;
    
    // Send via NRF
    nrf_send_status(message);
    
    // Also print to USB
    uint32_t ts = to_ms_since_boot(get_absolute_time());
    printf("$S,%lu,%s\r\n", ts, message);
}

void telemetry_send_debug(const char *format, ...) {
    #if DEBUG_ENABLE
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    printf("%s", buffer);
    #endif
}

void telemetry_process(void) {
    // Process NRF radio (receive commands)
    nrf_process();
}
