/**
 * @file neo_m8m.h
 * @brief NEO-M8M-0-10 GPS driver for RP2040
 */

#ifndef NEO_M8M_H
#define NEO_M8M_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    float speed;
    float course;
    uint8_t satellites;
    uint8_t fix_quality;
    uint8_t hour, minute, second;
    uint8_t day, month;
    uint16_t year;
    float hdop;
    bool valid;
} GPS_Data;

bool gps_init(void);
bool gps_wait_for_lock(uint32_t timeout_ms);
bool gps_read(GPS_Data *data);
void gps_process(void);
bool gps_has_fix(void);
uint8_t gps_get_satellites(void);
void gps_reset(void);

#endif // NEO_M8M_H
