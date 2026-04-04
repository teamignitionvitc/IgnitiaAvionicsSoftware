/**
 * @file flight_data.h
 * @brief Shared flight data structure passed through the control loop.
 */

#ifndef FLIGHT_DATA_H
#define FLIGHT_DATA_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t timestamp_ms;
    float dt_s;

    float accel_mag_g;
    float accel_z_g;

    float baro_altitude_m;
    float filtered_altitude_m;
    float baro_velocity_mps;
    float fused_velocity_mps;
    
    // CRITICAL FIX (Issue 1): Fixed-point velocity for deterministic timing
    int16_t velocity_cmps;  // Velocity in cm/s (range: ±327.68 m/s)

    float temperature_c;
    bool gps_valid;
    uint8_t gps_sats;

    bool imu_ok;
    bool bme_ok;
    bool altitude_spike_rejected;
    bool sensor_fault;
    bool high_noise;  // IMPROVEMENT (Issue 2): High noise detection for adaptive filtering
} SensorData;

#endif // FLIGHT_DATA_H
