/**
 * @file neo_m8m.c
 * @brief NEO-M8M GPS NMEA parser implementation
 */

#include "neo_m8m.h"
#include "config.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <string.h>
#include <stdlib.h>

static char rx_buffer[GPS_BUFFER_SIZE];
static uint16_t rx_index = 0;
static GPS_Data current_data = {0};
static bool data_ready = false;

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

bool gps_init(void) {
    uart_init(GPS_UART, GPS_BAUDRATE);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    
    uart_set_hw_flow(GPS_UART, false, false);
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);
    
    memset(&current_data, 0, sizeof(current_data));
    rx_index = 0;
    
    return true;
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
