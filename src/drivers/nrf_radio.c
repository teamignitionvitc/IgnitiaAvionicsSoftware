
// All includes and macros at the top
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include "nrf_radio.h"
#include "config.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

// Forward declaration for uart_write_with_timeout (must be before any use)
size_t uart_write_with_timeout(uart_inst_t *uart, const uint8_t *data, size_t len, uint32_t timeout_us);


// Send a telemetry packet: header (0xAA), timestamp (uint32_t), altitude (float), velocity (float), state (uint8_t), footer (0x55), newline ('\n')
void send_telemetry_packet(uint32_t timestamp, float altitude, float velocity, uint8_t state) {
    uint8_t packet[1 + 4 + 4 + 4 + 1 + 1 + 1];
    size_t idx = 0;
    packet[idx++] = 0xAA;
    memcpy(&packet[idx], &timestamp, 4); idx += 4;
    memcpy(&packet[idx], &altitude, 4); idx += 4;
    memcpy(&packet[idx], &velocity, 4); idx += 4;
    packet[idx++] = state;
    packet[idx++] = 0x55;
    packet[idx++] = '\n';

    size_t sent = uart_write_with_timeout(NRF_UART, packet, idx, NRF_UART_SEND_TIMEOUT_US);
    if (sent == idx) {
        printf("TX OK: %u bytes\r\n", (unsigned)sent);
    } else {
        printf("TX FAIL\r\n");
    }
}

// Timeout for UART send (microseconds)
// (now defined in header if not already)

// Helper: Write data to UART with timeout (returns bytes sent)
size_t uart_write_with_timeout(uart_inst_t *uart, const uint8_t *data, size_t len, uint32_t timeout_us) {
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    size_t sent = 0;
    while (sent < len) {
        if (uart_is_writable(uart)) {
            uart_putc_raw(uart, data[sent++]);
        } else if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            break; // Timeout
        }
    }
    return sent;
}

#define PACKET_HEADER   0xAA
#define RX_BUFFER_SIZE  64

static uint16_t tx_sequence = 0;
static uint16_t tx_count = 0;
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t rx_index = 0;

static uint8_t calculate_checksum(const uint8_t *data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

bool nrf_init(void) {
    // Initialize UART0 for NRF radio
    uart_init(NRF_UART, NRF_BAUDRATE);
    
    gpio_set_function(NRF_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(NRF_RX_PIN, GPIO_FUNC_UART);
    
    uart_set_hw_flow(NRF_UART, false, false);
    uart_set_format(NRF_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(NRF_UART, true);
    
    tx_sequence = 0;
    tx_count = 0;
    rx_index = 0;
    
    return true;
}

void nrf_send_telemetry(const TelemetryPacket *pkt) {
    if (!pkt) return;
    
    TelemetryPacket packet = *pkt;
    packet.header = PACKET_HEADER;
    packet.type = PKT_TELEMETRY;
    packet.seq = tx_sequence++;
    
    // Calculate checksum (exclude checksum field)
    packet.checksum = calculate_checksum((uint8_t*)&packet, sizeof(packet) - 1);
    
    size_t sent = uart_write_with_timeout(NRF_UART, (uint8_t*)&packet, sizeof(packet), NRF_UART_SEND_TIMEOUT_US);
    uint8_t nl = '\n';
    size_t sent_nl = uart_write_with_timeout(NRF_UART, &nl, 1, NRF_UART_SEND_TIMEOUT_US);
    bool ok = (sent == sizeof(packet) && sent_nl == 1);
    if (ok) {
        tx_count++;
    }
    printf("[NRF] Telemetry TX %s (%u bytes)\r\n", ok ? "OK" : "FAILED", (unsigned)(sizeof(packet)));
}

void nrf_send_gps(const GPSPacket *pkt) {
    if (!pkt) return;
    
    GPSPacket packet = *pkt;
    packet.header = PACKET_HEADER;
    packet.type = PKT_GPS;
    packet.seq = tx_sequence++;
    
    packet.checksum = calculate_checksum((uint8_t*)&packet, sizeof(packet) - 1);
    
    size_t sent = uart_write_with_timeout(NRF_UART, (uint8_t*)&packet, sizeof(packet), NRF_UART_SEND_TIMEOUT_US);
    uint8_t nl = '\n';
    size_t sent_nl = uart_write_with_timeout(NRF_UART, &nl, 1, NRF_UART_SEND_TIMEOUT_US);
    bool ok = (sent == sizeof(packet) && sent_nl == 1);
    if (ok) {
        tx_count++;
    }
    printf("[NRF] GPS TX %s (%u bytes)\r\n", ok ? "OK" : "FAILED", (unsigned)(sizeof(packet)));
}

void nrf_send_status(const char *message) {
    if (!message) return;
    
    StatusPacket packet = {0};
    packet.header = PACKET_HEADER;
    packet.type = PKT_STATUS;
    packet.seq = tx_sequence++;
    packet.timestamp = to_ms_since_boot(get_absolute_time());
    
    strncpy(packet.message, message, sizeof(packet.message) - 1);
    
    packet.checksum = calculate_checksum((uint8_t*)&packet, sizeof(packet) - 1);
    
    size_t sent = uart_write_with_timeout(NRF_UART, (uint8_t*)&packet, sizeof(packet), NRF_UART_SEND_TIMEOUT_US);
    uint8_t nl = '\n';
    size_t sent_nl = uart_write_with_timeout(NRF_UART, &nl, 1, NRF_UART_SEND_TIMEOUT_US);
    bool ok = (sent == sizeof(packet) && sent_nl == 1);
    if (ok) {
        tx_count++;
    }
    printf("[NRF] Status TX %s (%u bytes)\r\n", ok ? "OK" : "FAILED", (unsigned)(sizeof(packet)));
}

void nrf_send_raw(const uint8_t *data, size_t len) {
    if (!data || len == 0) return;
    size_t sent = uart_write_with_timeout(NRF_UART, data, len, NRF_UART_SEND_TIMEOUT_US);
    uint8_t nl = '\n';
    size_t sent_nl = uart_write_with_timeout(NRF_UART, &nl, 1, NRF_UART_SEND_TIMEOUT_US);
    bool ok = (sent == len && sent_nl == 1);
    printf("[NRF] Raw TX %s (%u bytes)\r\n", ok ? "OK" : "FAILED", (unsigned)len);
}

bool nrf_receive(uint8_t *buffer, size_t max_len, size_t *received) {
    if (!buffer || !received) return false;
    
    *received = 0;
    
    while (uart_is_readable(NRF_UART) && *received < max_len) {
        buffer[(*received)++] = uart_getc(NRF_UART);
    }
    
    return *received > 0;
}

void nrf_process(void) {
    // Process any incoming data
    while (uart_is_readable(NRF_UART)) {
        uint8_t c = uart_getc(NRF_UART);
        
        if (rx_index == 0 && c != PACKET_HEADER) {
            continue;  // Wait for header
        }
        
        if (rx_index < RX_BUFFER_SIZE) {
            rx_buffer[rx_index++] = c;
        }
        
        // Check for complete command packet
        if (rx_index >= 4 && rx_buffer[1] == PKT_COMMAND) {
            // Process command
            uint8_t cmd = rx_buffer[3];
            
            // TODO: Handle commands here
            // 'A' = arm, 'D' = disarm, 'T' = test, etc.
            
            rx_index = 0;
        }
        
        // Prevent overflow
        if (rx_index >= RX_BUFFER_SIZE) {
            rx_index = 0;
        }
    }
}

uint16_t nrf_get_tx_count(void) {
    return tx_count;
}
