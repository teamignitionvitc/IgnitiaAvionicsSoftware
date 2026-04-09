#!/usr/bin/env python3
"""
Generate a synthetic payload sensor dataset for SIL testing.

Profile:
- Ground idle
- Drone lift to 350 m
- Hold at altitude
- Recovery ejection event
- Short freefall
- Parachute descent to landing
"""

from __future__ import annotations

import argparse
import csv
import math
import random
from pathlib import Path


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(value, hi))


def baro_pressure_from_altitude(altitude_m: float) -> float:
    # Standard barometric formula approximation at low altitude.
    p0 = 101325.0
    return p0 * (1.0 - altitude_m / 44330.0) ** 5.255


def build_profile(time_s: float) -> tuple[str, float, float, float, int, str]:
    """
    Returns:
      phase, true_altitude_m, true_velocity_mps, true_accel_mps2, deployed, event
    """
    # Timeline (seconds)
    t_idle_end = 10.0
    t_ascent_end = 150.0
    t_hold_end = 155.0
    t_freefall_end = 160.0

    if time_s < t_idle_end:
        phase = "idle"
        altitude = 0.0
        velocity = 0.0
        acceleration = 0.0
        deployed = 0
        event = "ground_idle"
    elif time_s < t_ascent_end:
        phase = "drone_ascent"
        elapsed = time_s - t_idle_end
        ascent_rate = 2.5  # m/s
        altitude = ascent_rate * elapsed
        velocity = ascent_rate
        acceleration = 0.0
        deployed = 0
        event = "lift"
    elif time_s < t_hold_end:
        phase = "hover_hold"
        altitude = 350.0
        velocity = 0.0
        acceleration = 0.0
        deployed = 0
        event = "altitude_hold"
    elif time_s < t_freefall_end:
        phase = "freefall"
        elapsed = time_s - t_hold_end
        # Quadratic drop from 350 m for brief freefall segment.
        acceleration = -8.5
        velocity = acceleration * elapsed
        altitude = 350.0 + 0.5 * acceleration * elapsed * elapsed
        altitude = max(0.0, altitude)
        deployed = 0
        event = "recovery_ejection"
    else:
        phase = "parachute_descent"
        # Start from last freefall altitude, then descend at controlled speed.
        freefall_elapsed = t_freefall_end - t_hold_end
        freefall_alt = 350.0 + 0.5 * (-8.5) * freefall_elapsed * freefall_elapsed
        descent_elapsed = time_s - t_freefall_end
        descent_rate = -4.8
        altitude = freefall_alt + descent_rate * descent_elapsed
        altitude = max(0.0, altitude)
        velocity = 0.0 if altitude <= 0.0 else descent_rate
        acceleration = 0.0
        deployed = 1
        event = "chute_descent"
        if altitude <= 0.0:
            phase = "landed"
            event = "touchdown"

    return phase, altitude, velocity, acceleration, deployed, event


def generate_dataset(output_path: Path, sample_hz: float, duration_s: float, seed: int) -> int:
    random.seed(seed)

    dt = 1.0 / sample_hz
    steps = int(duration_s * sample_hz) + 1

    base_lat = 28.572900
    base_lon = -80.649000

    # Sensor model settings
    baro_noise_std = 0.35
    accel_noise_std_g = 0.03
    gyro_noise_std_dps = 0.4
    gps_noise_std_m = 1.8
    temp_ground_c = 30.0

    # Slow drift terms
    baro_drift_m = 0.0
    gyro_bias_x = 0.0
    gyro_bias_y = 0.0
    gyro_bias_z = 0.0

    output_path.parent.mkdir(parents=True, exist_ok=True)

    fieldnames = [
        "sample",
        "time_s",
        "time_ms",
        "phase",
        "event",
        "deployed",
        "true_altitude_m",
        "true_velocity_mps",
        "true_accel_mps2",
        "baro_altitude_m",
        "baro_pressure_pa",
        "baro_temp_c",
        "accel_x_g",
        "accel_y_g",
        "accel_z_g",
        "accel_magnitude_g",
        "gyro_x_dps",
        "gyro_y_dps",
        "gyro_z_dps",
        "gps_valid",
        "gps_sats",
        "gps_lat",
        "gps_lon",
        "gps_alt_m",
        "battery_v",
    ]

    with output_path.open("w", newline="", encoding="ascii") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for i in range(steps):
            t = i * dt
            phase, true_alt, true_vel, true_accel, deployed, event = build_profile(t)

            # Atmospheric/temperature trends
            temp_c = temp_ground_c - 0.0065 * true_alt + random.gauss(0.0, 0.12)

            # Barometer with drift/noise
            baro_drift_m += random.gauss(0.0, 0.002)
            baro_alt = max(0.0, true_alt + baro_drift_m + random.gauss(0.0, baro_noise_std))
            baro_pressure = baro_pressure_from_altitude(baro_alt) + random.gauss(0.0, 0.7)

            # IMU approximation: mostly vertical axis + gravity
            az_g_true = (true_accel + 9.81) / 9.81
            ax_g = random.gauss(0.0, accel_noise_std_g)
            ay_g = random.gauss(0.0, accel_noise_std_g)
            az_g = az_g_true + random.gauss(0.0, accel_noise_std_g)
            accel_mag_g = math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g)

            # Gyro with tiny bias random walk
            gyro_bias_x += random.gauss(0.0, 0.002)
            gyro_bias_y += random.gauss(0.0, 0.002)
            gyro_bias_z += random.gauss(0.0, 0.002)
            gyro_x = gyro_bias_x + random.gauss(0.0, gyro_noise_std_dps)
            gyro_y = gyro_bias_y + random.gauss(0.0, gyro_noise_std_dps)
            gyro_z = gyro_bias_z + random.gauss(0.0, gyro_noise_std_dps)

            # GPS validity after lock delay and at 5 Hz equivalent
            gps_valid = 1 if (t >= 20.0 and (i % max(1, int(sample_hz / 5.0)) == 0)) else 0
            if gps_valid:
                dlat = random.gauss(0.0, gps_noise_std_m / 111000.0)
                dlon = random.gauss(0.0, gps_noise_std_m / 111000.0)
                gps_lat = base_lat + dlat
                gps_lon = base_lon + dlon
                gps_alt = max(0.0, true_alt + random.gauss(0.0, gps_noise_std_m * 1.8))
                gps_sats = random.randint(7, 12)
            else:
                gps_lat = 0.0
                gps_lon = 0.0
                gps_alt = 0.0
                gps_sats = random.randint(0, 4)

            # Basic battery model under load over time
            battery_v = clamp(8.40 - 0.0009 * t + random.gauss(0.0, 0.01), 7.2, 8.4)

            writer.writerow(
                {
                    "sample": i,
                    "time_s": f"{t:.2f}",
                    "time_ms": int(round(t * 1000.0)),
                    "phase": phase,
                    "event": event,
                    "deployed": deployed,
                    "true_altitude_m": f"{true_alt:.3f}",
                    "true_velocity_mps": f"{true_vel:.3f}",
                    "true_accel_mps2": f"{true_accel:.3f}",
                    "baro_altitude_m": f"{baro_alt:.3f}",
                    "baro_pressure_pa": f"{baro_pressure:.2f}",
                    "baro_temp_c": f"{temp_c:.3f}",
                    "accel_x_g": f"{ax_g:.4f}",
                    "accel_y_g": f"{ay_g:.4f}",
                    "accel_z_g": f"{az_g:.4f}",
                    "accel_magnitude_g": f"{accel_mag_g:.4f}",
                    "gyro_x_dps": f"{gyro_x:.4f}",
                    "gyro_y_dps": f"{gyro_y:.4f}",
                    "gyro_z_dps": f"{gyro_z:.4f}",
                    "gps_valid": gps_valid,
                    "gps_sats": gps_sats,
                    "gps_lat": f"{gps_lat:.7f}",
                    "gps_lon": f"{gps_lon:.7f}",
                    "gps_alt_m": f"{gps_alt:.3f}",
                    "battery_v": f"{battery_v:.3f}",
                }
            )

    return steps


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate dummy payload sensor data")
    parser.add_argument(
        "--output",
        default="test_data/payload_dummy_350m.csv",
        help="Output CSV file path",
    )
    parser.add_argument("--hz", type=float, default=10.0, help="Sample rate in Hz")
    parser.add_argument(
        "--duration",
        type=float,
        default=220.0,
        help="Duration in seconds",
    )
    parser.add_argument("--seed", type=int, default=35042, help="Random seed")
    args = parser.parse_args()

    output = Path(args.output)
    count = generate_dataset(output, args.hz, args.duration, args.seed)

    print(f"Generated {count} samples at {args.hz:.2f} Hz")
    print(f"Output: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
