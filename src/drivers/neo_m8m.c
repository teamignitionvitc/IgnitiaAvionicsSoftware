/**
 * @file neo_m8m.c
 * @brief NEO-M8M-0-10 GPS driver with UBX configuration and NMEA parser
 */

#include "neo_m8m.h"
#include "config.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static char rx_buffer[GPS_BUFFER_SIZE];
static uint16_t rx_index = 0;
static GPS_Data current_data = {0};
static bool data_ready = false;

// ---------------------------------------------------------------------------
// NMEA parsing helpers
// ---------------------------------------------------------------------------
static float parse_coord(const char *str, char dir) {
    if (!str || strlen(str) < 4) return 0.0f;
    
    float raw = atof(str);
    int degrees = (int)(raw / 100);
    float minutes = raw - (degrees * 100);
    float decimal = degrees + (minutes / 60.0f);
    
    if (dir == 'S' || dir == 'W') decimal = -decimal;
    return decimal;
}

static void parse_gga(char *sentence) {
    char *token;
    int field = 0;
    
    token = strtok(sentence, ",");
    while (token != NULL) {
        switch (field) {
            case 1: // Time
                if (strlen(token) >= 6) {
                    current_data.hour = (token[0] - '0') * 10 + (token[1] - '0');
                    current_data.minute = (token[2] - '0') * 10 + (token[3] - '0');
                    current_data.second = (token[4] - '0') * 10 + (token[5] - '0');
                }
                break;
            case 2: // Latitude
                current_data.latitude = parse_coord(token, 'N');
                break;
            case 3: // N/S
                if (token[0] == 'S') current_data.latitude = -current_data.latitude;
                break;
            case 4: // Longitude
                current_data.longitude = parse_coord(token, 'E');
                break;
            case 5: // E/W
                if (token[0] == 'W') current_data.longitude = -current_data.longitude;
                break;
            case 6: // Fix quality
                current_data.fix_quality = atoi(token);
                current_data.valid = (current_data.fix_quality > 0);
                break;
            case 7: // Satellites
                current_data.satellites = atoi(token);
                break;
            case 8: // HDOP
                current_data.hdop = atof(token);
                break;
            case 9: // Altitude
                current_data.altitude = atof(token);
                break;
        }
        field++;
        token = strtok(NULL, ",");
    }
}

static void parse_rmc(char *sentence) {
    char *token;
    int field = 0;
    
    token = strtok(sentence, ",");
    while (token != NULL) {
        switch (field) {
            case 2: // Status (A=valid, V=invalid)
                current_data.valid = (token[0] == 'A');
                break;
            case 7: // Speed in knots
                current_data.speed = atof(token) * 1.852f; // Convert to km/h
                break;
            case 8: // Course
                current_data.course = atof(token);
                break;
            case 9: // Date
                if (strlen(token) >= 6) {
                    current_data.day = (token[0] - '0') * 10 + (token[1] - '0');
                    current_data.month = (token[2] - '0') * 10 + (token[3] - '0');
                    current_data.year = 2000 + (token[4] - '0') * 10 + (token[5] - '0');
                }
                break;
        }
        field++;
        token = strtok(NULL, ",");
    }
}

static bool verify_checksum(const char *sentence) {
    if (sentence[0] != '$') return false;
    
    const char *asterisk = strchr(sentence, '*');
    if (!asterisk) return false;
    
    uint8_t checksum = 0;
    for (const char *p = sentence + 1; p < asterisk; p++) {
        checksum ^= *p;
    }
    
    uint8_t given = (uint8_t)strtol(asterisk + 1, NULL, 16);
    return checksum == given;
}

static void process_sentence(char *sentence) {
    if (!verify_checksum(sentence)) return;
    
    if (strncmp(sentence + 3, "GGA", 3) == 0) {
        parse_gga(sentence);
        data_ready = true;
    } else if (strncmp(sentence + 3, "RMC", 3) == 0) {
        parse_rmc(sentence);
    }
}

// ---------------------------------------------------------------------------
// UBX protocol helpers (for NEO-M8M configuration)
// ---------------------------------------------------------------------------
static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
    uint8_t header[6];
    header[0] = 0xB5;  // Sync char 1
    header[1] = 0x62;  // Sync char 2
    header[2] = cls;
    header[3] = id;
    header[4] = len & 0xFF;
    header[5] = (len >> 8) & 0xFF;

    // Calculate checksum (Fletcher-16 over class, id, length, payload)
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6; i++) {
        ck_a += header[i];
        ck_b += ck_a;
    }
    for (uint16_t i = 0; i < len; i++) {
        ck_a += payload[i];
        ck_b += ck_a;
    }

    // Send header
    for (int i = 0; i < 6; i++) {
        uart_putc_raw(GPS_UART, header[i]);
    }
    // Send payload
    for (uint16_t i = 0; i < len; i++) {
        uart_putc_raw(GPS_UART, payload[i]);
    }
    // Send checksum
    uart_putc_raw(GPS_UART, ck_a);
    uart_putc_raw(GPS_UART, ck_b);
}

/**
 * Configure NEO-M8M navigation rate.
 * UBX-CFG-RATE: measRate (ms), navRate (cycles), timeRef (0=UTC)
 */
static void gps_configure_rate(uint16_t rate_hz) {
    uint16_t meas_ms = 1000 / rate_hz;
    uint8_t payload[6];
    payload[0] = meas_ms & 0xFF;
    payload[1] = (meas_ms >> 8) & 0xFF;
    payload[2] = 1;   // navRate = 1 (every measurement)
    payload[3] = 0;
    payload[4] = 0;   // timeRef = UTC
    payload[5] = 0;
    ubx_send(0x06, 0x08, payload, 6);  // CFG-RATE
    sleep_ms(100);
}

/**
 * Configure NEO-M8M for airborne <1g dynamics model.
 * UBX-CFG-NAV5: dynModel=6 (airborne <1g), fixMode=3 (auto 2D/3D)
 */
static void gps_configure_airborne(void) {
    uint8_t payload[36];
    memset(payload, 0, sizeof(payload));
    // mask: apply dynModel and fixMode
    payload[0] = 0x05; payload[1] = 0x00;
    // dynModel: 6 = airborne <1g (best for CanSat/drone)
    payload[2] = 6;
    // fixMode: 3 = auto 2D/3D
    payload[3] = 3;
    ubx_send(0x06, 0x24, payload, 36);  // CFG-NAV5
    sleep_ms(100);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
bool gps_init(void) {
    uart_init(GPS_UART, GPS_BAUDRATE);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    
    uart_set_hw_flow(GPS_UART, false, false);
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);
    
    memset(&current_data, 0, sizeof(current_data));
    rx_index = 0;

    // Give the NEO-M8M time to boot (needs ~1s after power-on)
    sleep_ms(1000);

    // Configure airborne dynamics model (for CanSat/drone use)
    printf("  Configuring airborne mode...\r\n");
    gps_configure_airborne();

    // Configure navigation rate
    printf("  Setting nav rate to %d Hz...\r\n", GPS_NAV_RATE_HZ);
    gps_configure_rate(GPS_NAV_RATE_HZ);
    
    return true;
}

/**
 * Wait for GPS fix with timeout. Call after gps_init().
 * Prints satellite count while waiting.
 * Returns true if fix acquired, false if timeout.
 */
bool gps_wait_for_lock(uint32_t timeout_ms) {
    printf("[GPS] Waiting for fix (timeout %lus)...\r\n",
           (unsigned long)(timeout_ms / 1000));

    uint32_t start = to_ms_since_boot(get_absolute_time());
    uint32_t last_print = 0;

    while ((to_ms_since_boot(get_absolute_time()) - start) < timeout_ms) {
        gps_process();

        uint32_t now = to_ms_since_boot(get_absolute_time());
        // Print status every 2 seconds
        if ((now - last_print) >= 2000) {
            last_print = now;
            uint32_t elapsed = (now - start) / 1000;
            if (current_data.valid && current_data.fix_quality > 0) {
                printf("[GPS] LOCKED! Sats=%d Fix=%d HDOP=%.1f (%lus)\r\n",
                       current_data.satellites, current_data.fix_quality,
                       current_data.hdop, (unsigned long)elapsed);
                return true;
            } else {
                printf("[GPS] Searching... Sats=%d (%lus)\r\n",
                       current_data.satellites, (unsigned long)elapsed);
            }
        }

        sleep_ms(50);  // Don't spin too fast
    }

    printf("[GPS] Timeout — no fix after %lus (sats=%d). Continuing anyway.\r\n",
           (unsigned long)(timeout_ms / 1000), current_data.satellites);
    return false;
}

void gps_process(void) {
    while (uart_is_readable(GPS_UART)) {
        char c = uart_getc(GPS_UART);
        
        if (c == '$') {
            rx_index = 0;
        }
        
        if (rx_index < GPS_BUFFER_SIZE - 1) {
            rx_buffer[rx_index++] = c;
            rx_buffer[rx_index] = '\0';
        }
        
        if (c == '\n') {
            if (rx_buffer[0] == '$') {
                process_sentence(rx_buffer);
            }
            rx_index = 0;
        }
    }
}

bool gps_read(GPS_Data *data) {
    gps_process();
    
    if (data && data_ready) {
        *data = current_data;
        return true;
    }
    return false;
}

bool gps_has_fix(void) {
    return current_data.valid && current_data.fix_quality > 0;
}

uint8_t gps_get_satellites(void) {
    return current_data.satellites;
}

void gps_reset(void) {
    memset(&current_data, 0, sizeof(current_data));
    rx_index = 0;
    data_ready = false;
}
