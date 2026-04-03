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

void telemetry_init(void) {
    // Initialize NRF radio
    nrf_init();
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
