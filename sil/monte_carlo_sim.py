#!/usr/bin/env python3
"""
Monte Carlo simulation framework for flight control validation.

Runs 1000+ randomized flights to validate robustness and measure success rates.
"""

import numpy as np
import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple
from dataclasses import dataclass, asdict

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from sil_runner import SILRunner
from sensor_models import SensorNoise


@dataclass
class SimulationResult:
    """Result of a single simulation run"""
    run_id: int
    success: bool
    freefall_detected: bool
    apogee_detected: bool
    deployed: bool
    false_deployment: bool
    max_altitude: float
    deployment_altitude: float
    landing_velocity: float
    failure_reason: str = ""


class MonteCarloSimulator:
    """Monte Carlo simulation framework"""
    
    def __init__(self, num_runs: int = 1000):
        self.num_runs = num_runs
        self.results: List[SimulationResult] = []
        
    def randomize_initial_conditions(self) -> Dict:
        """Generate randomized initial conditions"""
        return {
            "initial_altitude": np.random.uniform(80, 120),  # 80-120m drop altitude
            "initial_velocity": np.random.uniform(-0.5, 0.5),  # Small initial velocity
            "ground_pressure": np.random.uniform(98000, 103000),  # Pressure variation
            "temperature": np.random.uniform(15, 30),  # Temperature 15-30°C
            "wind_speed": np.random.uniform(0, 5),  # Wind 0-5 m/s
        }
    
    def randomize_sensor_noise(self) -> Dict:
        """Generate randomized sensor noise parameters"""
        return {
            "imu_noise_scale": np.random.uniform(0.8, 1.2),  # ±20% noise variation
            "baro_noise_scale": np.random.uniform(0.8, 1.2),
            "imu_bias": np.random.uniform(-0.05, 0.05),  # Small bias
            "baro_bias": np.random.uniform(-2, 2),  # ±2m altitude bias
        }
    
    def run_single_simulation(self, run_id: int) -> SimulationResult:
        """Run a single randomized simulation"""
        
        # Randomize conditions
        initial_conditions = self.randomize_initial_conditions()
        noise_params = self.randomize_sensor_noise()
        
        # Create scenario
        scenario = {
            "name": f"monte_carlo_run_{run_id}",
            "description": "Randomized Monte Carlo test",
            "duration": 30.0,
            "initial_altitude": initial_conditions["initial_altitude"],
            "initial_velocity": initial_conditions["initial_velocity"],
            "ground_pressure": initial_conditions["ground_pressure"],
            "sensor_noise": {
                "imu_accel_std": 0.02 * noise_params["imu_noise_scale"],
                "baro_alt_std": 0.5 * noise_params["baro_noise_scale"],
                "imu_bias": noise_params["imu_bias"],
                "baro_bias": noise_params["baro_bias"],
            }
        }
        
        # Run simulation
        runner = SILRunner()
        try:
            result = runner.run_scenario(scenario)
            
            # Analyze results
            freefall_detected = any(s["state"] == "FREEFALL" for s in result["states"])
            apogee_detected = any(s["state"] == "APOGEE" for s in result["states"])
            deployed = any(s["state"] == "DEPLOYED" for s in result["states"])
            
            # Check for false deployment (deployment before freefall)
            false_deployment = False
            for i, state in enumerate(result["states"]):
                if state["state"] == "DEPLOYED":
                    # Check if freefall was detected before deployment
                    prev_states = [s["state"] for s in result["states"][:i]]
                    if "FREEFALL" not in prev_states:
                        false_deployment = True
                    break
            
            # Get metrics
            max_altitude = max(s["altitude"] for s in result["states"])
            deployment_altitude = next(
                (s["altitude"] for s in result["states"] if s["state"] == "DEPLOYED"),
                0.0
            )
            landing_velocity = result["states"][-1]["velocity"] if result["states"] else 0.0
            
            # Determine success
            success = (
                freefall_detected and
                apogee_detected and
                deployed and
                not false_deployment and
                deployment_altitude > 10  # Deployed above 10m
            )
            
            failure_reason = ""
            if not freefall_detected:
                failure_reason = "Freefall not detected"
            elif not apogee_detected:
                failure_reason = "Apogee not detected"
            elif not deployed:
                failure_reason = "Deployment failed"
            elif false_deployment:
                failure_reason = "False deployment"
            elif deployment_altitude <= 10:
                failure_reason = f"Deployed too low ({deployment_altitude:.1f}m)"
            
            return SimulationResult(
                run_id=run_id,
                success=success,
                freefall_detected=freefall_detected,
                apogee_detected=apogee_detected,
                deployed=deployed,
                false_deployment=false_deployment,
                max_altitude=max_altitude,
                deployment_altitude=deployment_altitude,
                landing_velocity=landing_velocity,
                failure_reason=failure_reason
            )
            
        except Exception as e:
            return SimulationResult(
                run_id=run_id,
                success=False,
                freefall_detected=False,
                apogee_detected=False,
                deployed=False,
                false_deployment=False,
                max_altitude=0.0,
                deployment_altitude=0.0,
                landing_velocity=0.0,
                failure_reason=f"Simulation error: {str(e)}"
            )
    
    def run_all(self) -> Dict:
        """Run all Monte Carlo simulations"""
        print(f"Running {self.num_runs} Monte Carlo simulations...")
        
        for i in range(self.num_runs):
            if (i + 1) % 100 == 0:
                print(f"  Progress: {i + 1}/{self.num_runs}")
            
            result = self.run_single_simulation(i)
            self.results.append(result)
        
        return self.analyze_results()
    
    def analyze_results(self) -> Dict:
        """Analyze simulation results and compute statistics"""
        total = len(self.results)
        
        if total == 0:
            return {"error": "No results to analyze"}
        
        successes = sum(1 for r in self.results if r.success)
        freefall_detections = sum(1 for r in self.results if r.freefall_detected)
        apogee_detections = sum(1 for r in self.results if r.apogee_detected)
        deployments = sum(1 for r in self.results if r.deployed)
        false_deployments = sum(1 for r in self.results if r.false_deployment)
        
        # Compute statistics
        stats = {
            "total_runs": total,
            "success_rate": successes / total * 100,
            "freefall_detection_rate": freefall_detections / total * 100,
            "apogee_detection_rate": apogee_detections / total * 100,
            "deployment_rate": deployments / total * 100,
            "false_deployment_rate": false_deployments / total * 100,
            "success_criteria": {
                "freefall_detection": f"{freefall_detections / total * 100:.1f}% (target: 99%+)",
                "apogee_detection": f"{apogee_detections / total * 100:.1f}% (target: 95%+)",
                "deployment": f"{deployments / total * 100:.1f}% (target: 100%)",
                "false_deployment": f"{false_deployments / total * 100:.1f}% (target: 0%)",
            },
            "failure_reasons": {}
        }
        
        # Count failure reasons
        for result in self.results:
            if not result.success and result.failure_reason:
                reason = result.failure_reason
                stats["failure_reasons"][reason] = stats["failure_reasons"].get(reason, 0) + 1
        
        return stats
    
    def save_results(self, filename: str = "monte_carlo_results.json"):
        """Save results to JSON file"""
        output = {
            "statistics": self.analyze_results(),
            "results": [asdict(r) for r in self.results]
        }
        
        with open(filename, 'w') as f:
            json.dump(output, f, indent=2)
        
        print(f"\nResults saved to {filename}")


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Monte Carlo simulation for flight control")
    parser.add_argument("--runs", type=int, default=1000, help="Number of simulation runs")
    parser.add_argument("--output", type=str, default="monte_carlo_results.json", 
                       help="Output file for results")
    
    args = parser.parse_args()
    
    # Run Monte Carlo simulation
    simulator = MonteCarloSimulator(num_runs=args.runs)
    stats = simulator.run_all()
    
    # Print summary
    print("\n" + "="*60)
    print("MONTE CARLO SIMULATION RESULTS")
    print("="*60)
    print(f"Total runs: {stats['total_runs']}")
    print(f"Success rate: {stats['success_rate']:.1f}%")
    print(f"\nDetection rates:")
    print(f"  Freefall: {stats['freefall_detection_rate']:.1f}%")
    print(f"  Apogee: {stats['apogee_detection_rate']:.1f}%")
    print(f"  Deployment: {stats['deployment_rate']:.1f}%")
    print(f"  False deployment: {stats['false_deployment_rate']:.1f}%")
    
    if stats['failure_reasons']:
        print(f"\nFailure reasons:")
        for reason, count in sorted(stats['failure_reasons'].items(), 
                                   key=lambda x: x[1], reverse=True):
            print(f"  {reason}: {count} ({count/stats['total_runs']*100:.1f}%)")
    
    print("\nSuccess criteria:")
    for criterion, value in stats['success_criteria'].items():
        print(f"  {criterion}: {value}")
    
    # Save results
    simulator.save_results(args.output)
    
    # Exit with appropriate code
    if stats['success_rate'] >= 95.0:
        print("\n✓ SUCCESS: System meets robustness criteria (≥95% success rate)")
        return 0
    else:
        print(f"\n✗ FAILURE: System below robustness criteria ({stats['success_rate']:.1f}% < 95%)")
        return 1


if __name__ == "__main__":
    sys.exit(main())
