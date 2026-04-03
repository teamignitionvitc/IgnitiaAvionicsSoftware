/**
 * @file nrf_radio.c
 * @brief NRF UART Radio driver implementation
 */

#include "nrf_radio.h"
#include "config.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <string.h>

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
    
    uart_write_blocking(NRF_UART, (uint8_t*)&packet, sizeof(packet));
    tx_count++;
}

void nrf_send_gps(const GPSPacket *pkt) {
    if (!pkt) return;
    
    GPSPacket packet = *pkt;
    packet.header = PACKET_HEADER;
    packet.type = PKT_GPS;
    packet.seq = tx_sequence++;
    
    packet.checksum = calculate_checksum((uint8_t*)&packet, sizeof(packet) - 1);
    
    uart_write_blocking(NRF_UART, (uint8_t*)&packet, sizeof(packet));
    tx_count++;
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
    
    uart_write_blocking(NRF_UART, (uint8_t*)&packet, sizeof(packet));
    tx_count++;
}

void nrf_send_raw(const uint8_t *data, size_t len) {
    if (!data || len == 0) return;
    uart_write_blocking(NRF_UART, data, len);
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
