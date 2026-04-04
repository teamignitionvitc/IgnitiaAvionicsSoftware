#!/usr/bin/env python3
"""
Log replay capability for regression testing.

Parses CSV log files from real flights and replays them through the SIL
to verify behavior matches expected results.
"""

import csv
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, asdict

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from sil_runner import SILRunner


@dataclass
class LogEntry:
    """Single log entry from CSV"""
    timestamp_ms: int
    altitude_m: float
    velocity_mps: float
    accel_g: float
    state: str


@dataclass
class ReplayResult:
    """Result of log replay"""
    log_file: str
    success: bool
    total_entries: int
    state_matches: int
    state_mismatches: int
    max_altitude_error: float
    max_velocity_error: float
    mismatched_states: List[Dict]
    failure_reason: str = ""


class LogReplay:
    """Log replay framework for regression testing"""
    
    def __init__(self):
        self.results: List[ReplayResult] = []
    
    def parse_csv_log(self, log_file: str) -> List[LogEntry]:
        """Parse CSV log file into LogEntry objects"""
        entries = []
        
        try:
            with open(log_file, 'r') as f:
                # Try to detect format
                first_line = f.readline().strip()
                f.seek(0)
                
                # Check if it's the $L format from logger.c
                if first_line.startswith('$L,'):
                    for line in f:
                        if not line.startswith('$L,'):
                            continue
                        
                        parts = line.strip().split(',')
                        if len(parts) >= 5:
                            entry = LogEntry(
                                timestamp_ms=int(parts[1]),
                                altitude_m=float(parts[2]),
                                velocity_mps=float(parts[3]),
                                accel_g=float(parts[4]),
                                state=parts[5] if len(parts) > 5 else "UNKNOWN"
                            )
                            entries.append(entry)
                
                # Otherwise try standard CSV with header
                else:
                    reader = csv.DictReader(f)
                    for row in reader:
                        entry = LogEntry(
                            timestamp_ms=int(row.get('timestamp_ms', row.get('time', 0))),
                            altitude_m=float(row.get('altitude_m', row.get('altitude', 0.0))),
                            velocity_mps=float(row.get('velocity_mps', row.get('velocity', 0.0))),
                            accel_g=float(row.get('accel_g', row.get('accel', 0.0))),
                            state=row.get('state', 'UNKNOWN')
                        )
                        entries.append(entry)
        
        except Exception as e:
            print(f"Error parsing log file {log_file}: {e}")
            return []
        
        return entries
    
    def create_scenario_from_log(self, entries: List[LogEntry]) -> Dict:
        """Create SIL scenario from log entries"""
        if not entries:
            return {}
        
        # Extract initial conditions from first entry
        first = entries[0]
        last = entries[-1]
        
        duration = (last.timestamp_ms - first.timestamp_ms) / 1000.0
        
        return {
            "name": "log_replay",
            "description": "Replay from log file",
            "duration": duration,
            "initial_altitude": first.altitude_m,
            "initial_velocity": first.velocity_mps,
            "replay_mode": True,
            "replay_data": [
                {
                    "time": (e.timestamp_ms - first.timestamp_ms) / 1000.0,
                    "altitude": e.altitude_m,
                    "velocity": e.velocity_mps,
                    "accel": e.accel_g,
                    "expected_state": e.state
                }
                for e in entries
            ]
        }
    
    def compare_states(self, expected: List[LogEntry], 
                      actual: List[Dict]) -> Tuple[int, int, List[Dict]]:
        """Compare expected states from log with actual SIL states"""
        matches = 0
        mismatches = 0
        mismatched_states = []
        
        # Create time-indexed lookup for actual states
        actual_by_time = {s["time"]: s for s in actual}
        
        for entry in expected:
            time_s = entry.timestamp_ms / 1000.0
            
            # Find closest actual state
            closest_time = min(actual_by_time.keys(), 
                             key=lambda t: abs(t - time_s),
                             default=None)
            
            if closest_time is None:
                continue
            
            actual_state = actual_by_time[closest_time]
            
            if actual_state["state"] == entry.state:
                matches += 1
            else:
                mismatches += 1
                mismatched_states.append({
                    "time": time_s,
                    "expected": entry.state,
                    "actual": actual_state["state"],
                    "altitude": entry.altitude_m,
                    "velocity": entry.velocity_mps
                })
        
        return matches, mismatches, mismatched_states
    
    def replay_log(self, log_file: str) -> ReplayResult:
        """Replay a log file through SIL"""
        
        print(f"Replaying log: {log_file}")
        
        # Parse log file
        entries = self.parse_csv_log(log_file)
        
        if not entries:
            return ReplayResult(
                log_file=log_file,
                success=False,
                total_entries=0,
                state_matches=0,
                state_mismatches=0,
                max_altitude_error=0.0,
                max_velocity_error=0.0,
                mismatched_states=[],
                failure_reason="Failed to parse log file"
            )
        
        print(f"  Parsed {len(entries)} log entries")
        
        # Create scenario
        scenario = self.create_scenario_from_log(entries)
        
        # Run SIL
        runner = SILRunner()
        try:
            result = runner.run_scenario(scenario)
            
            # Compare states
            matches, mismatches, mismatched_states = self.compare_states(
                entries, result["states"]
            )
            
            # Calculate errors
            max_altitude_error = 0.0
            max_velocity_error = 0.0
            
            for i, entry in enumerate(entries):
                if i < len(result["states"]):
                    actual = result["states"][i]
                    alt_error = abs(actual["altitude"] - entry.altitude_m)
                    vel_error = abs(actual["velocity"] - entry.velocity_mps)
                    max_altitude_error = max(max_altitude_error, alt_error)
                    max_velocity_error = max(max_velocity_error, vel_error)
            
            # Determine success
            # Success if >95% state matches and errors are reasonable
            match_rate = matches / len(entries) if entries else 0.0
            success = (
                match_rate >= 0.95 and
                max_altitude_error < 5.0 and  # Within 5m
                max_velocity_error < 2.0      # Within 2 m/s
            )
            
            failure_reason = ""
            if match_rate < 0.95:
                failure_reason = f"State match rate too low ({match_rate*100:.1f}%)"
            elif max_altitude_error >= 5.0:
                failure_reason = f"Altitude error too high ({max_altitude_error:.1f}m)"
            elif max_velocity_error >= 2.0:
                failure_reason = f"Velocity error too high ({max_velocity_error:.1f} m/s)"
            
            return ReplayResult(
                log_file=log_file,
                success=success,
                total_entries=len(entries),
                state_matches=matches,
                state_mismatches=mismatches,
                max_altitude_error=max_altitude_error,
                max_velocity_error=max_velocity_error,
                mismatched_states=mismatched_states[:10],  # Keep first 10
                failure_reason=failure_reason
            )
            
        except Exception as e:
            return ReplayResult(
                log_file=log_file,
                success=False,
                total_entries=len(entries),
                state_matches=0,
                state_mismatches=len(entries),
                max_altitude_error=0.0,
                max_velocity_error=0.0,
                mismatched_states=[],
                failure_reason=f"Replay error: {str(e)}"
            )
    
    def replay_all(self, log_files: List[str]) -> Dict:
        """Replay multiple log files"""
        
        print(f"Replaying {len(log_files)} log files...")
        
        for log_file in log_files:
            result = self.replay_log(log_file)
            self.results.append(result)
            
            status = "✓ PASS" if result.success else "✗ FAIL"
            print(f"  {status}: {Path(log_file).name}")
            if not result.success:
                print(f"    Reason: {result.failure_reason}")
        
        return self.analyze_results()
    
    def analyze_results(self) -> Dict:
        """Analyze replay results"""
        total = len(self.results)
        
        if total == 0:
            return {"error": "No results to analyze"}
        
        successes = sum(1 for r in self.results if r.success)
        total_entries = sum(r.total_entries for r in self.results)
        total_matches = sum(r.state_matches for r in self.results)
        total_mismatches = sum(r.state_mismatches for r in self.results)
        
        stats = {
            "total_logs": total,
            "success_rate": successes / total * 100,
            "total_entries": total_entries,
            "overall_match_rate": total_matches / total_entries * 100 if total_entries > 0 else 0.0,
            "failures": [
                {
                    "log": r.log_file,
                    "reason": r.failure_reason,
                    "match_rate": r.state_matches / r.total_entries * 100 if r.total_entries > 0 else 0.0
                }
                for r in self.results if not r.success
            ]
        }
        
        return stats
    
    def save_results(self, filename: str = "replay_results.json"):
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
    
    parser = argparse.ArgumentParser(description="Log replay for regression testing")
    parser.add_argument("logs", nargs="+", help="Log files to replay")
    parser.add_argument("--output", type=str, default="replay_results.json",
                       help="Output file for results")
    
    args = parser.parse_args()
    
    # Run log replay
    replayer = LogReplay()
    stats = replayer.replay_all(args.logs)
    
    # Print summary
    print("\n" + "="*60)
    print("LOG REPLAY RESULTS")
    print("="*60)
    print(f"Total logs: {stats['total_logs']}")
    print(f"Success rate: {stats['success_rate']:.1f}%")
    print(f"Total entries: {stats['total_entries']}")
    print(f"Overall match rate: {stats['overall_match_rate']:.1f}%")
    
    if stats['failures']:
        print(f"\nFailures ({len(stats['failures'])}):")
        for failure in stats['failures']:
            print(f"  {Path(failure['log']).name}: {failure['reason']}")
            print(f"    Match rate: {failure['match_rate']:.1f}%")
    
    # Save results
    replayer.save_results(args.output)
    
    # Exit with appropriate code
    if stats['success_rate'] >= 95.0:
        print("\n✓ SUCCESS: All logs replayed successfully")
        return 0
    else:
        print(f"\n✗ FAILURE: Some logs failed replay ({stats['success_rate']:.1f}% < 95%)")
        return 1


if __name__ == "__main__":
    sys.exit(main())
