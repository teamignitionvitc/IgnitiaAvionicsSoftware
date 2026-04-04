/**
 * @file health_monitor.c
 * @brief System health monitoring implementation
 */

#include "health_monitor.h"
#include <string.h>

// Timeout thresholds (milliseconds)
#define IMU_TIMEOUT_MS      100    // IMU should update at 50-100 Hz
#define BARO_TIMEOUT_MS     100    // Baro should update at 50-100 Hz
#define GPS_TIMEOUT_MS      2000   // GPS updates at ~1 Hz

static SystemHealth health_status;

void health_init(void) {
    memset(&health_status, 0, sizeof(health_status));
    health_status.imu_ok = false;
    health_status.baro_ok = false;
    health_status.gps_ok = false;
}

void health_update_imu(bool ok, uint32_t timestamp_ms) {
    if (ok) {
        health_status.imu_ok = true;
        health_status.imu_last_ms = timestamp_ms;
    } else {
        health_status.imu_ok = false;
    }
}

void health_update_baro(bool ok, uint32_t timestamp_ms) {
    if (ok) {
        health_status.baro_ok = true;
        health_status.baro_last_ms = timestamp_ms;
    } else {
        health_status.baro_ok = false;
    }
}

void health_update_gps(bool ok, uint32_t timestamp_ms) {
    if (ok) {
        health_status.gps_ok = true;
        health_status.gps_last_ms = timestamp_ms;
    } else {
        health_status.gps_ok = false;
    }
}

void health_check_timeouts(uint32_t timestamp_ms) {
    // Check IMU timeout
    if (health_status.imu_ok && 
        (timestamp_ms - health_status.imu_last_ms) > IMU_TIMEOUT_MS) {
        health_status.imu_ok = false;
    }
    
    // Check barometer timeout
    if (health_status.baro_ok && 
        (timestamp_ms - health_status.baro_last_ms) > BARO_TIMEOUT_MS) {
        health_status.baro_ok = false;
    }
    
    // Check GPS timeout
    if (health_status.gps_ok && 
        (timestamp_ms - health_status.gps_last_ms) > GPS_TIMEOUT_MS) {
        health_status.gps_ok = false;
    }
}

const SystemHealth* health_get_status(void) {
    return &health_status;
}
