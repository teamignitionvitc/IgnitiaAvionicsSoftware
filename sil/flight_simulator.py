#!/usr/bin/env python3
"""
Flight Simulator - Simulates CanSat flight dynamics
"""

import math
from dataclasses import dataclass, field
from typing import Dict, Any
import random


@dataclass
class FlightParams:
    """Flight simulation parameters"""
    launch_accel: float = 50.0      # m/s² initial acceleration
    burn_time: float = 2.0          # seconds
    drag_coeff: float = 0.5         # drag coefficient
    mass: float = 0.35              # kg
    area: float = 0.008             # m² cross-section
    chute_drag: float = 2.0         # parachute drag coefficient
    chute_area: float = 0.25        # m² parachute area
    target_apogee: float = 400.0    # m target altitude


class FlightSimulator:
    """Physics-based flight simulator"""
    
    GRAVITY = 9.81
    AIR_DENSITY = 1.225  # kg/m³ at sea level
    
    def __init__(self):
        self.params = FlightParams()
        self.reset()
    
    def reset(self):
        """Reset simulation state"""
        self.time = 0.0
        self.altitude = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
        self.phase = 'idle'
        self.deployed = False
        self.landed = False
        self.max_altitude = 0.0
        self.apogee_time = None
        self.deploy_time = None
        self.land_time = None
    
    def configure(self, scenario: Dict[str, Any]):
        """Configure simulator from scenario"""
        params = scenario.get('flight_params', {})
        
        if 'target_apogee' in params:
            self.params.target_apogee = params['target_apogee']
        if 'mass' in params:
            self.params.mass = params['mass']
        if 'burn_time' in params:
            self.params.burn_time = params['burn_time']
        
        # Calculate launch acceleration for target apogee
        # Using energy: 0.5*m*v² = m*g*h, v = sqrt(2*g*h)
        # v = a*t, so a = sqrt(2*g*h)/t
        target_v = math.sqrt(2 * self.GRAVITY * self.params.target_apogee * 1.3)  # 30% margin for drag
        self.params.launch_accel = target_v / self.params.burn_time
        
        self.reset()
        self.phase = 'armed' if scenario.get('armed', True) else 'idle'
    
    def step(self, dt: float) -> Dict[str, Any]:
        """Advance simulation by dt seconds"""
        self.time += dt
        
        # Phase transitions
        self._update_phase()
        
        # Compute forces
        thrust = self._get_thrust()
        drag = self._get_drag()
        gravity = self.GRAVITY * self.params.mass
        
        net_force = thrust - drag - gravity
        self.acceleration = net_force / self.params.mass
        
        # Integrate
        self.velocity += self.acceleration * dt
        self.altitude += self.velocity * dt
        
        # Ground constraint
        if self.altitude < 0:
            self.altitude = 0
            self.velocity = 0
            if self.phase in ['descent', 'apogee']:
                self.phase = 'landed'
                self.landed = True
                self.land_time = self.time
        
        # Track max altitude
        if self.altitude > self.max_altitude:
            self.max_altitude = self.altitude
        
        return self.get_state()
    
    def _update_phase(self):
        """Update flight phase based on conditions"""
        if self.phase == 'armed' and self.time > 1.0:
            self.phase = 'ascent'
        
        elif self.phase == 'ascent':
            # Apogee detection: velocity becomes negative
            if self.velocity <= 0:
                self.phase = 'apogee'
                self.apogee_time = self.time
        
        elif self.phase == 'apogee':
            # Deploy after brief delay
            if not self.deployed and self.apogee_time and (self.time - self.apogee_time) > 0.5:
                self.deployed = True
                self.deploy_time = self.time
                self.phase = 'descent'
        
        elif self.phase == 'descent':
            if self.altitude < 10 and abs(self.velocity) < 0.5:
                self.phase = 'landed'
                self.landed = True
                self.land_time = self.time
    
    def _get_thrust(self) -> float:
        """Get current thrust force"""
        if self.phase == 'ascent' and self.time < self.params.burn_time + 1.0:
            return self.params.launch_accel * self.params.mass
        return 0.0
    
    def _get_drag(self) -> float:
        """Get aerodynamic drag force"""
        if self.velocity == 0:
            return 0
        
        # Select drag parameters based on parachute state
        if self.deployed:
            cd = self.params.chute_drag
            area = self.params.chute_area
        else:
            cd = self.params.drag_coeff
            area = self.params.area
        
        # Drag = 0.5 * rho * v² * Cd * A
        drag = 0.5 * self.AIR_DENSITY * (self.velocity ** 2) * cd * area
        
        # Drag opposes motion
        return drag if self.velocity > 0 else -drag
    
    def get_state(self) -> Dict[str, Any]:
        """Get current simulation state"""
        return {
            'time': self.time,
            'altitude': self.altitude,
            'velocity': self.velocity,
            'acceleration': self.acceleration,
            'phase': self.phase,
            'deployed': self.deployed,
            'landed': self.landed,
            'max_altitude': self.max_altitude
        }


# Quick test
if __name__ == '__main__':
    sim = FlightSimulator()
    sim.configure({'armed': True, 'flight_params': {'target_apogee': 400}})
    
    print("Time(s)  Alt(m)   Vel(m/s) Accel    Phase")
    print("-" * 50)
    
    dt = 0.1
    while not sim.landed and sim.time < 120:
        state = sim.step(dt)
        if int(sim.time * 10) % 10 == 0:  # Print every second
            print(f"{state['time']:6.1f}  {state['altitude']:7.1f}  {state['velocity']:7.1f}  {state['acceleration']:7.1f}  {state['phase']}")
    
    print(f"\nMax altitude: {sim.max_altitude:.1f}m")
    print(f"Apogee time: {sim.apogee_time:.1f}s")
    print(f"Deploy time: {sim.deploy_time:.1f}s")
    print(f"Land time: {sim.land_time:.1f}s")
