#!/usr/bin/env python3
"""
SIL Runner - Software-in-the-Loop test framework for Ignitia CanSat Avionics
Simulates flight profiles and validates flight computer behavior
"""

import json
import time
import argparse
import logging
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

from flight_simulator import FlightSimulator
from sensor_models import SensorModels

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


@dataclass
class SILConfig:
    """SIL test configuration"""
    scenario_file: str
    time_scale: float = 10.0  # Speed up simulation
    log_interval: float = 0.1
    output_dir: str = "results"


@dataclass
class FlightEvent:
    """Recorded flight event"""
    timestamp: float
    event_type: str
    details: dict


class SILRunner:
    """Main SIL test runner"""
    
    def __init__(self, config: SILConfig):
        self.config = config
        self.simulator = FlightSimulator()
        self.sensors = SensorModels()
        self.events: list[FlightEvent] = []
        self.telemetry_log: list[dict] = []
        self._last_phase: Optional[str] = None
        
    def load_scenario(self, scenario_file: str) -> dict:
        """Load flight scenario from JSON file"""
        with open(scenario_file, 'r') as f:
            scenario = json.load(f)
        logger.info(f"Loaded scenario: {scenario.get('name', 'unnamed')}")
        return scenario
    
    def run(self) -> bool:
        """Run the SIL test"""
        scenario = self.load_scenario(self.config.scenario_file)
        
        # Initialize simulator
        self.simulator.configure(scenario)
        self.sensors.configure(scenario.get('sensor_config', {}))
        
        # Expected events from scenario
        expected_events = scenario.get('expected_events', [])
        
        logger.info("Starting SIL simulation...")
        start_time = time.time()
        sim_time = 0.0
        dt = 0.01  # 100Hz internal simulation
        
        while sim_time < scenario.get('duration', 120.0):
            # Update simulator
            state = self.simulator.step(dt)
            
            # Generate sensor readings with noise
            sensor_data = self.sensors.generate(state)
            
            # Log telemetry
            if sim_time % self.config.log_interval < dt:
                self.telemetry_log.append({
                    'time': sim_time,
                    'altitude': state['altitude'],
                    'velocity': state['velocity'],
                    'accel': state['acceleration'],
                    'phase': state['phase'],
                    'baro_alt': sensor_data['baro_altitude'],
                    'accel_mag': sensor_data['accel_magnitude']
                })
            
            # Check for events
            self._check_events(sim_time, state)
            
            sim_time += dt
        
        # Validate results
        elapsed = time.time() - start_time
        logger.info(f"Simulation complete in {elapsed:.2f}s (simulated {sim_time:.1f}s)")
        
        return self._validate_results(expected_events)
    
    def _check_events(self, sim_time: float, state: dict):
        """Check and record flight events"""
        phase = state['phase']
        
        # Record phase transitions
        if self._last_phase != phase:
            self.events.append(FlightEvent(
                timestamp=sim_time,
                event_type='phase_change',
                details={'phase': phase}
            ))
            self._last_phase = phase
            logger.info(f"t={sim_time:.2f}s: Phase -> {phase}")
        
        # Check for deployment
        if state.get('deployed') and not any(e.event_type == 'deployment' for e in self.events):
            self.events.append(FlightEvent(
                timestamp=sim_time,
                event_type='deployment',
                details={'altitude': state['altitude']}
            ))
            logger.info(f"t={sim_time:.2f}s: DEPLOYMENT at {state['altitude']:.1f}m")
    
    def _validate_results(self, expected_events: list) -> bool:
        """Validate simulation results against expected events"""
        logger.info("\n=== Validation Results ===")
        
        all_pass = True
        for expected in expected_events:
            event_type = expected['type']
            expected_details = expected.get('details', {})
            
            # Find matching events, including optional detail filters.
            matching = [e for e in self.events if e.event_type == event_type]
            if expected_details:
                matching = [
                    e for e in matching
                    if all(e.details.get(k) == v for k, v in expected_details.items())
                ]
            
            if not matching:
                logger.error(f"FAIL: Expected event '{event_type}' with details {expected_details} not found")
                all_pass = False
                continue

            min_t = expected.get('min_time', float('-inf'))
            max_t = expected.get('max_time', float('inf'))
            in_window = [e for e in matching if min_t <= e.timestamp <= max_t]
            if in_window:
                event = in_window[0]
                logger.info(f"PASS: {event_type} {expected_details} at t={event.timestamp:.2f}s")
            else:
                closest = min(
                    matching,
                    key=lambda e: 0 if (min_t <= e.timestamp <= max_t) else min(abs(e.timestamp - min_t), abs(e.timestamp - max_t))
                )
                logger.error(
                    f"FAIL: {event_type} {expected_details} outside timing window "
                    f"[{min_t:.2f}, {max_t:.2f}]s, observed at {closest.timestamp:.2f}s"
                )
                all_pass = False
        
        # Check deployment altitude if applicable
        deploy_events = [e for e in self.events if e.event_type == 'deployment']
        if deploy_events:
            deploy_alt = deploy_events[0].details['altitude']
            if deploy_alt > 500:
                logger.warning(f"WARNING: Deployment above 500m ({deploy_alt:.1f}m)")
        
        return all_pass
    
    def save_results(self, output_file: str):
        """Save telemetry and events to file"""
        results = {
            'events': [{'time': e.timestamp, 'type': e.event_type, **e.details} for e in self.events],
            'telemetry': self.telemetry_log
        }

        Path(output_file).parent.mkdir(parents=True, exist_ok=True)
        with open(output_file, 'w') as f:
            json.dump(results, f, indent=2)
        logger.info(f"Results saved to {output_file}")


def main():
    parser = argparse.ArgumentParser(description='Ignitia SIL Test Runner')
    parser.add_argument('scenario', help='Path to scenario JSON file')
    parser.add_argument('--output', '-o', default='results/sil_output.json', help='Output file')
    parser.add_argument('--time-scale', type=float, default=10.0, help='Time acceleration factor')
    args = parser.parse_args()
    
    config = SILConfig(
        scenario_file=args.scenario,
        time_scale=args.time_scale
    )
    
    runner = SILRunner(config)
    success = runner.run()
    runner.save_results(args.output)
    
    if success:
        logger.info("\n✓ SIL test PASSED")
        return 0
    else:
        logger.error("\n✗ SIL test FAILED")
        return 1


if __name__ == '__main__':
    exit(main())
