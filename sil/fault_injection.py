#!/usr/bin/env python3
"""
Fault injection testing framework for flight control validation.

Tests system behavior under various fault conditions:
- IMU failure
- Barometer spikes
- GPS loss
- Servo failure
"""

import numpy as np
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional
from dataclasses import dataclass, asdict

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from sil_runner import SILRunner


@dataclass
class FaultConfig:
    """Configuration for a fault injection"""
    fault_type: str
    start_time: float
    duration: float
    severity: float = 1.0
    description: str = ""


@dataclass
class FaultTestResult:
    """Result of a fault injection test"""
    test_name: str
    fault_config: FaultConfig
    success: bool
    deployed: bool
    deployment_altitude: float
    failure_reason: str = ""
    recovery_time: float = 0.0


class FaultInjector:
    """Fault injection testing framework"""
    
    def __init__(self):
        self.results: List[FaultTestResult] = []
    
    def create_imu_failure_scenario(self, failure_start: float = 2.0, 
                                   failure_duration: float = 1.0) -> Dict:
        """Create scenario with IMU failure"""
        return {
            "name": "imu_failure",
            "description": f"IMU fails at t={failure_start}s for {failure_duration}s",
            "duration": 30.0,
            "initial_altitude": 100.0,
            "faults": [{
                "type": "imu_failure",
                "start_time": failure_start,
                "duration": failure_duration,
                "severity": 1.0  # Complete failure
            }]
        }
    
    def create_baro_spike_scenario(self, spike_time: float = 5.0,
                                   spike_magnitude: float = 50.0) -> Dict:
        """Create scenario with barometer spike"""
        return {
            "name": "baro_spike",
            "description": f"Baro spike of {spike_magnitude}m at t={spike_time}s",
            "duration": 30.0,
            "initial_altitude": 100.0,
            "faults": [{
                "type": "baro_spike",
                "start_time": spike_time,
                "duration": 0.1,  # Brief spike
                "severity": spike_magnitude
            }]
        }
    
    def create_gps_loss_scenario(self, loss_start: float = 3.0,
                                 loss_duration: float = 5.0) -> Dict:
        """Create scenario with GPS loss"""
        return {
            "name": "gps_loss",
            "description": f"GPS lost at t={loss_start}s for {loss_duration}s",
            "duration": 30.0,
            "initial_altitude": 100.0,
            "faults": [{
                "type": "gps_loss",
                "start_time": loss_start,
                "duration": loss_duration,
                "severity": 1.0
            }]
        }
    
    def create_servo_failure_scenario(self, failure_time: float = 8.0) -> Dict:
        """Create scenario with servo failure at deployment"""
        return {
            "name": "servo_failure",
            "description": f"Servo fails at t={failure_time}s",
            "duration": 30.0,
            "initial_altitude": 100.0,
            "faults": [{
                "type": "servo_failure",
                "start_time": failure_time,
                "duration": 999.0,  # Permanent failure
                "severity": 1.0
            }]
        }
    
    def create_multiple_faults_scenario(self) -> Dict:
        """Create scenario with multiple simultaneous faults"""
        return {
            "name": "multiple_faults",
            "description": "IMU + GPS failure simultaneously",
            "duration": 30.0,
            "initial_altitude": 100.0,
            "faults": [
                {
                    "type": "imu_failure",
                    "start_time": 3.0,
                    "duration": 2.0,
                    "severity": 1.0
                },
                {
                    "type": "gps_loss",
                    "start_time": 3.0,
                    "duration": 5.0,
                    "severity": 1.0
                }
            ]
        }
    
    def run_fault_test(self, scenario: Dict) -> FaultTestResult:
        """Run a single fault injection test"""
        
        runner = SILRunner()
        fault_config = FaultConfig(
            fault_type=scenario["faults"][0]["type"] if scenario.get("faults") else "none",
            start_time=scenario["faults"][0]["start_time"] if scenario.get("faults") else 0.0,
            duration=scenario["faults"][0]["duration"] if scenario.get("faults") else 0.0,
            severity=scenario["faults"][0]["severity"] if scenario.get("faults") else 0.0,
            description=scenario["description"]
        )
        
        try:
            result = runner.run_scenario(scenario)
            
            # Analyze results
            deployed = any(s["state"] == "DEPLOYED" for s in result["states"])
            deployment_altitude = next(
                (s["altitude"] for s in result["states"] if s["state"] == "DEPLOYED"),
                0.0
            )
            
            # Check if system recovered from fault
            recovery_time = 0.0
            if scenario.get("faults"):
                fault_end = fault_config.start_time + fault_config.duration
                # System recovered if it deployed after fault ended
                if deployed:
                    deploy_time = next(
                        (s["time"] for s in result["states"] if s["state"] == "DEPLOYED"),
                        0.0
                    )
                    if deploy_time > fault_end:
                        recovery_time = deploy_time - fault_end
            
            # Determine success
            # Success = system still deployed despite fault
            success = deployed and deployment_altitude > 10
            
            failure_reason = ""
            if not deployed:
                failure_reason = "Deployment failed due to fault"
            elif deployment_altitude <= 10:
                failure_reason = f"Deployed too low ({deployment_altitude:.1f}m)"
            
            return FaultTestResult(
                test_name=scenario["name"],
                fault_config=fault_config,
                success=success,
                deployed=deployed,
                deployment_altitude=deployment_altitude,
                failure_reason=failure_reason,
                recovery_time=recovery_time
            )
            
        except Exception as e:
            return FaultTestResult(
                test_name=scenario["name"],
                fault_config=fault_config,
                success=False,
                deployed=False,
                deployment_altitude=0.0,
                failure_reason=f"Test error: {str(e)}",
                recovery_time=0.0
            )
    
    def run_all_tests(self) -> Dict:
        """Run all fault injection tests"""
        
        test_scenarios = [
            # IMU failures at different times
            self.create_imu_failure_scenario(failure_start=1.0, failure_duration=0.5),
            self.create_imu_failure_scenario(failure_start=3.0, failure_duration=1.0),
            self.create_imu_failure_scenario(failure_start=5.0, failure_duration=2.0),
            
            # Barometer spikes of varying magnitude
            self.create_baro_spike_scenario(spike_time=2.0, spike_magnitude=20.0),
            self.create_baro_spike_scenario(spike_time=5.0, spike_magnitude=50.0),
            self.create_baro_spike_scenario(spike_time=7.0, spike_magnitude=100.0),
            
            # GPS loss at different times
            self.create_gps_loss_scenario(loss_start=1.0, loss_duration=3.0),
            self.create_gps_loss_scenario(loss_start=5.0, loss_duration=10.0),
            
            # Servo failure
            self.create_servo_failure_scenario(failure_time=8.0),
            
            # Multiple faults
            self.create_multiple_faults_scenario(),
        ]
        
        print(f"Running {len(test_scenarios)} fault injection tests...")
        
        for i, scenario in enumerate(test_scenarios):
            print(f"  Test {i+1}/{len(test_scenarios)}: {scenario['description']}")
            result = self.run_fault_test(scenario)
            self.results.append(result)
            
            status = "✓ PASS" if result.success else "✗ FAIL"
            print(f"    {status}: {result.failure_reason if not result.success else 'System recovered'}")
        
        return self.analyze_results()
    
    def analyze_results(self) -> Dict:
        """Analyze fault injection test results"""
        total = len(self.results)
        
        if total == 0:
            return {"error": "No results to analyze"}
        
        successes = sum(1 for r in self.results if r.success)
        
        # Group by fault type
        by_fault_type = {}
        for result in self.results:
            fault_type = result.fault_config.fault_type
            if fault_type not in by_fault_type:
                by_fault_type[fault_type] = {"total": 0, "success": 0}
            by_fault_type[fault_type]["total"] += 1
            if result.success:
                by_fault_type[fault_type]["success"] += 1
        
        stats = {
            "total_tests": total,
            "success_rate": successes / total * 100,
            "by_fault_type": {
                fault_type: {
                    "success_rate": data["success"] / data["total"] * 100,
                    "tests": data["total"]
                }
                for fault_type, data in by_fault_type.items()
            },
            "failures": [
                {
                    "test": r.test_name,
                    "fault": r.fault_config.fault_type,
                    "reason": r.failure_reason
                }
                for r in self.results if not r.success
            ]
        }
        
        return stats
    
    def save_results(self, filename: str = "fault_injection_results.json"):
        """Save results to JSON file"""
        output = {
            "statistics": self.analyze_results(),
            "results": [
                {
                    **asdict(r),
                    "fault_config": asdict(r.fault_config)
                }
                for r in self.results
            ]
        }
        
        with open(filename, 'w') as f:
            json.dump(output, f, indent=2)
        
        print(f"\nResults saved to {filename}")


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Fault injection testing for flight control")
    parser.add_argument("--output", type=str, default="fault_injection_results.json",
                       help="Output file for results")
    
    args = parser.parse_args()
    
    # Run fault injection tests
    injector = FaultInjector()
    stats = injector.run_all_tests()
    
    # Print summary
    print("\n" + "="*60)
    print("FAULT INJECTION TEST RESULTS")
    print("="*60)
    print(f"Total tests: {stats['total_tests']}")
    print(f"Success rate: {stats['success_rate']:.1f}%")
    
    print(f"\nResults by fault type:")
    for fault_type, data in stats['by_fault_type'].items():
        print(f"  {fault_type}: {data['success_rate']:.1f}% ({data['tests']} tests)")
    
    if stats['failures']:
        print(f"\nFailures ({len(stats['failures'])}):")
        for failure in stats['failures']:
            print(f"  {failure['test']}: {failure['reason']}")
    
    # Save results
    injector.save_results(args.output)
    
    # Exit with appropriate code
    if stats['success_rate'] >= 80.0:
        print("\n✓ SUCCESS: System demonstrates good fault tolerance (≥80% success rate)")
        return 0
    else:
        print(f"\n✗ WARNING: System fault tolerance below target ({stats['success_rate']:.1f}% < 80%)")
        return 1


if __name__ == "__main__":
    sys.exit(main())
