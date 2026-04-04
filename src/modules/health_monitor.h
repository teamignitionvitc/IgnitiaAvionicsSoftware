/**
 * @file health_monitor.h
 * @brief System health monitoring for sensor fault detection
 */

#ifndef HEALTH_MONITOR_H
#define HEALTH_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief System health status structure
 */
typedef struct {
    bool imu_ok;              // IMU sensor health
    bool baro_ok;             // Barometer sensor health
    bool gps_ok;              // GPS sensor health
    uint32_t imu_last_ms;     // Last successful IMU read timestamp
    uint32_t baro_last_ms;    // Last successful baro read timestamp
    uint32_t gps_last_ms;     // Last successful GPS read timestamp
} SystemHealth;

/**
 * @brief Initialize health monitoring system
 */
void health_init(void);

/**
 * @brief Update IMU health status
 * @param ok True if IMU read was successful
 * @param timestamp_ms Current timestamp in milliseconds
 */
void health_update_imu(bool ok, uint32_t timestamp_ms);

/**
 * @brief Update barometer health status
 * @param ok True if barometer read was successful
 * @param timestamp_ms Current timestamp in milliseconds
 */
void health_update_baro(bool ok, uint32_t timestamp_ms);

/**
 * @brief Update GPS health status
 * @param ok True if GPS read was successful
 * @param timestamp_ms Current timestamp in milliseconds
 */
void health_update_gps(bool ok, uint32_t timestamp_ms);

/**
 * @brief Check for sensor timeouts
 * @param timestamp_ms Current timestamp in milliseconds
 */
void health_check_timeouts(uint32_t timestamp_ms);

/**
 * @brief Get current system health status
 * @return Pointer to SystemHealth structure
 */
const SystemHealth* health_get_status(void);

#endif // HEALTH_MONITOR_H
