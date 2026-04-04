/**
 * @file telemetry.c
 * @brief Telemetry implementation - USB debug and NRF radio packets
 */

#include "telemetry.h"
#include "config.h"
#include "state_machine.h"
#include "nrf_radio.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define TELEMETRY_INTERVAL_MS            50u
#define TELEMETRY_VALIDATE_CHECKSUM      1

static uint16_t packet_sequence = 0;
static uint32_t last_telemetry_time = 0;

static uint8_t calculate_checksum(const uint8_t *data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void telemetry_init(void) {
    nrf_init();
    packet_sequence = 0;
    last_telemetry_time = 0;
}

void telemetry_send(const SensorData *data) {
    if (!data) return;

    if ((data->timestamp_ms - last_telemetry_time) < TELEMETRY_INTERVAL_MS) {
        return;
    }

    last_telemetry_time = data->timestamp_ms;

    TelemetryPacket pkt = {0};

    pkt.header = 0xAA;
    pkt.type = PKT_TELEMETRY;
    pkt.seq = packet_sequence++;
    pkt.timestamp = data->timestamp_ms;
    pkt.state = (uint8_t)get_state();

    pkt.altitude = (int16_t)(data->filtered_altitude_m * 10.0f);   // dm
    pkt.velocity = (int16_t)(data->fused_velocity_mps * 100.0f);   // cm/s
    pkt.accel = (int16_t)(data->accel_mag_g * 1000.0f);            // mg
    pkt.temperature = (int16_t)(data->temperature_c * 10.0f);      // 0.1C
    pkt.gps_sats = data->gps_sats;

    pkt.flags = 0;
    if (get_state() != STATE_IDLE && get_state() != STATE_LANDED) pkt.flags |= FLAG_ARMED;
    if (get_state() == STATE_DEPLOYED || get_state() == STATE_LANDED) pkt.flags |= FLAG_DEPLOYED;
    if (data->gps_valid) pkt.flags |= FLAG_GPS_FIX;
    if (data->sensor_fault) pkt.flags |= FLAG_ERROR;

    pkt.checksum = calculate_checksum((uint8_t*)&pkt, sizeof(pkt) - 1);

#if TELEMETRY_VALIDATE_CHECKSUM
    uint8_t verify = calculate_checksum((uint8_t*)&pkt, sizeof(pkt) - 1);
    if (verify != pkt.checksum) {
        return;
    }
#endif

    nrf_send_raw((uint8_t*)&pkt, sizeof(pkt));
    
    #if DEBUG_ENABLE
    printf("$T,%lu,%s,%.1f,%.2f,%.2f,%u\r\n",
        pkt.timestamp,
        state_name(get_state()),
        data->filtered_altitude_m,
        data->fused_velocity_mps,
        data->accel_mag_g,
        pkt.seq);
    #endif
}

void telemetry_send_status(const char *message) {
    if (!message) return;

    nrf_send_status(message);

    #if DEBUG_ENABLE
    printf("$S,%s\r\n", message);
    #endif
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
    nrf_process();
}

uint16_t telemetry_get_sequence(void) {
    return packet_sequence;
}
