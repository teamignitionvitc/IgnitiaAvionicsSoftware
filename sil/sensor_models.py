#!/usr/bin/env python3
"""
Sensor Models - Simulates sensor noise and behavior for SIL testing
"""

import math
import random
from dataclasses import dataclass
from typing import Dict, Any, Optional


@dataclass
class SensorConfig:
    """Sensor simulation configuration"""
    # BME280 barometer
    baro_noise_std: float = 0.5      # meters altitude noise
    baro_drift_rate: float = 0.01    # m/s drift rate
    baro_offset: float = 0.0         # initial offset
    
    # MPU6050 IMU
    accel_noise_std: float = 0.02    # g noise
    accel_offset: tuple = (0.0, 0.0, 0.0)
    gyro_noise_std: float = 0.5      # deg/s noise
    gyro_drift_rate: float = 0.1     # deg/s/s drift
    
    # NEO-M8M GPS
    gps_noise_std: float = 2.5       # meters position noise
    gps_update_rate: float = 5.0     # Hz
    gps_fix_delay: float = 30.0      # seconds to first fix
    
    # Failure modes
    sensor_failure_prob: float = 0.0  # probability of failure per second


class SensorModels:
    """Sensor simulation with noise and failure modes"""
    
    GRAVITY = 9.81
    
    def __init__(self):
        self.config = SensorConfig()
        self.reset()
    
    def reset(self):
        """Reset sensor state"""
        self.time = 0.0
        self.baro_drift = 0.0
        self.gyro_drift = [0.0, 0.0, 0.0]
        self.gps_has_fix = False
        self.last_gps_update = 0.0
        self.failed_sensors = set()
    
    def configure(self, config: Dict[str, Any]):
        """Configure from dictionary"""
        for key, value in config.items():
            if hasattr(self.config, key):
                setattr(self.config, key, value)
        self.reset()
    
    def generate(self, true_state: Dict[str, Any]) -> Dict[str, Any]:
        """Generate noisy sensor readings from true state"""
        dt = 0.01  # Assume 100Hz
        self.time += dt
        
        # Check for sensor failures
        self._update_failures(dt)
        
        # Generate readings
        sensor_data = {
            'baro_altitude': self._baro_reading(true_state),
            'baro_pressure': self._baro_pressure(true_state),
            'baro_temp': self._baro_temperature(true_state),
            'accel_x': 0.0,
            'accel_y': 0.0,
            'accel_z': 0.0,
            'accel_magnitude': 0.0,
            'gyro_x': 0.0,
            'gyro_y': 0.0,
            'gyro_z': 0.0,
            'gps_lat': 0.0,
            'gps_lon': 0.0,
            'gps_alt': 0.0,
            'gps_valid': False,
            'gps_sats': 0
        }
        
        # IMU readings
        accel = self._accel_reading(true_state)
        sensor_data['accel_x'] = accel[0]
        sensor_data['accel_y'] = accel[1]
        sensor_data['accel_z'] = accel[2]
        sensor_data['accel_magnitude'] = math.sqrt(sum(a**2 for a in accel))
        
        gyro = self._gyro_reading(true_state)
        sensor_data['gyro_x'] = gyro[0]
        sensor_data['gyro_y'] = gyro[1]
        sensor_data['gyro_z'] = gyro[2]
        
        # GPS readings
        gps = self._gps_reading(true_state)
        sensor_data.update(gps)
        
        return sensor_data
    
    def _baro_reading(self, state: Dict[str, Any]) -> float:
        """Generate barometric altitude reading"""
        if 'baro' in self.failed_sensors:
            return float('nan')
        
        true_alt = state.get('altitude', 0.0)
        
        # Add noise
        noise = random.gauss(0, self.config.baro_noise_std)
        
        # Update drift
        self.baro_drift += random.gauss(0, self.config.baro_drift_rate * 0.01)
        
        return true_alt + noise + self.baro_drift + self.config.baro_offset
    
    def _baro_pressure(self, state: Dict[str, Any]) -> float:
        """Generate pressure reading from altitude"""
        if 'baro' in self.failed_sensors:
            return float('nan')
        
        alt = state.get('altitude', 0.0)
        # Barometric formula: P = P0 * (1 - h/44330)^5.255
        p0 = 101325  # Sea level Pa
        pressure = p0 * (1 - alt / 44330) ** 5.255
        
        # Add noise (about 0.5 Pa)
        return pressure + random.gauss(0, 0.5)
    
    def _baro_temperature(self, state: Dict[str, Any]) -> float:
        """Generate temperature reading"""
        if 'baro' in self.failed_sensors:
            return float('nan')
        
        # Lapse rate: -6.5°C per 1000m
        alt = state.get('altitude', 0.0)
        base_temp = 20.0  # Ground temperature
        temp = base_temp - (alt * 0.0065)
        
        return temp + random.gauss(0, 0.1)
    
    def _accel_reading(self, state: Dict[str, Any]) -> tuple:
        """Generate accelerometer readings"""
        if 'imu' in self.failed_sensors:
            return (float('nan'), float('nan'), float('nan'))
        
        accel = state.get('acceleration', 0.0)
        
        # In flight, Z-axis sees acceleration + gravity
        # Simplified: assume vertical orientation
        az = (accel + self.GRAVITY) / self.GRAVITY  # Convert to g
        ax = 0.0
        ay = 0.0
        
        # Add noise and offset
        ax += random.gauss(0, self.config.accel_noise_std) + self.config.accel_offset[0]
        ay += random.gauss(0, self.config.accel_noise_std) + self.config.accel_offset[1]
        az += random.gauss(0, self.config.accel_noise_std) + self.config.accel_offset[2]
        
        return (ax, ay, az)
    
    def _gyro_reading(self, state: Dict[str, Any]) -> tuple:
        """Generate gyroscope readings"""
        if 'imu' in self.failed_sensors:
            return (float('nan'), float('nan'), float('nan'))
        
        # Assume no rotation in simulation (vertical flight)
        gx = gy = gz = 0.0
        
        # Add noise and drift
        for i, drift in enumerate(self.gyro_drift):
            self.gyro_drift[i] += random.gauss(0, self.config.gyro_drift_rate * 0.01)
        
        gx += random.gauss(0, self.config.gyro_noise_std) + self.gyro_drift[0]
        gy += random.gauss(0, self.config.gyro_noise_std) + self.gyro_drift[1]
        gz += random.gauss(0, self.config.gyro_noise_std) + self.gyro_drift[2]
        
        return (gx, gy, gz)
    
    def _gps_reading(self, state: Dict[str, Any]) -> Dict[str, Any]:
        """Generate GPS readings"""
        result = {
            'gps_lat': 0.0,
            'gps_lon': 0.0,
            'gps_alt': 0.0,
            'gps_valid': False,
            'gps_sats': 0
        }
        
        if 'gps' in self.failed_sensors:
            return result
        
        # Simulate fix acquisition
        if self.time < self.config.gps_fix_delay:
            result['gps_sats'] = min(int(self.time / 5), 3)
            return result
        
        self.gps_has_fix = True
        
        # Check update rate
        if (self.time - self.last_gps_update) < (1.0 / self.config.gps_update_rate):
            return result
        
        self.last_gps_update = self.time
        
        # Generate position (simple: assume ground coordinates)
        # Base position with noise
        result['gps_lat'] = 28.5729 + random.gauss(0, self.config.gps_noise_std / 111000)
        result['gps_lon'] = -80.6490 + random.gauss(0, self.config.gps_noise_std / 111000)
        result['gps_alt'] = state.get('altitude', 0.0) + random.gauss(0, self.config.gps_noise_std * 2)
        result['gps_valid'] = True
        result['gps_sats'] = random.randint(6, 12)
        
        return result
    
    def _update_failures(self, dt: float):
        """Check for sensor failures"""
        if self.config.sensor_failure_prob <= 0:
            return
        
        sensors = ['baro', 'imu', 'gps']
        for sensor in sensors:
            if sensor not in self.failed_sensors:
                if random.random() < self.config.sensor_failure_prob * dt:
                    self.failed_sensors.add(sensor)
                    print(f"WARNING: {sensor} sensor failed at t={self.time:.2f}s")
    
    def inject_failure(self, sensor: str):
        """Manually inject sensor failure"""
        self.failed_sensors.add(sensor)
    
    def recover_failure(self, sensor: str):
        """Recover from sensor failure"""
        self.failed_sensors.discard(sensor)


# Quick test
if __name__ == '__main__':
    sensors = SensorModels()
    
    # Test with sample state
    state = {
        'altitude': 100.0,
        'velocity': 50.0,
        'acceleration': 10.0
    }
    
    print("Sensor readings test:")
    for i in range(10):
        data = sensors.generate(state)
        print(f"  Baro alt: {data['baro_altitude']:.2f}m, Accel: {data['accel_magnitude']:.3f}g")
