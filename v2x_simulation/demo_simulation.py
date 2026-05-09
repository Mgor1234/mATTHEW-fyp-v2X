"""
V2X Ambulance Simulation - Demo Mode (No CARLA Required)
Generates realistic simulation data for testing the analysis framework
"""

import json
import os
from datetime import datetime
import random
import math
from config import *


class DemoAmbulanceSimulation:
    """Demo simulation that generates realistic data without CARLA"""
    
    def __init__(self):
        self.run_results = []
    
    def generate_realistic_results(self, num_runs=NUM_RUNS, v2x_enabled=True):
        """
        Generate realistic trip duration data
        
        With V2X: ~40-50 seconds (vehicles clear path)
        Without V2X: ~60-75 seconds (stuck in traffic)
        """
        results = []
        
        if v2x_enabled:
            # V2X scenario - faster response
            base_time = 45.0
            variance = 3.5
            distance_base = 800
        else:
            # Baseline - slower response
            base_time = 68.0
            variance = 5.0
            distance_base = 850
        
        for run in range(1, num_runs + 1):
            # Add realistic variation to each run
            trip_duration = base_time + random.gauss(0, variance)
            trip_duration = max(trip_duration, base_time - 10)  # Minimum realistic time
            
            trip_distance = distance_base + random.gauss(0, 50)
            trip_distance = max(trip_distance, 750)  # Minimum distance
            
            average_speed = trip_distance / trip_duration if trip_duration > 0 else 0
            
            result = {
                'run_number': run,
                'trip_duration': trip_duration,
                'trip_distance': trip_distance,
                'average_speed': average_speed,
                'vehicles_spawned': TRAFFIC_DENSITY,
                'real_time_elapsed': trip_duration * 0.7,  # Simulation typically 70% real-time
                'timestamp': datetime.now().isoformat()
            }
            
            results.append(result)
        
        return results
    
    def run_v2x_simulation(self, num_runs=NUM_RUNS):
        """Run V2X simulation with path clearing"""
        print(f"\n{'='*60}")
        print(f"V2X AMBULANCE SIMULATION (DEMO MODE)")
        print(f"Configuration:")
        print(f"  - Runs: {num_runs}")
        print(f"  - Traffic Density: {TRAFFIC_DENSITY} vehicles")
        print(f"  - V2X Range: {V2X_DETECTION_RANGE}m")
        print(f"  - Map: {MAP_NAME}")
        print(f"{'='*60}\n")
        
        self.run_results = self.generate_realistic_results(num_runs, v2x_enabled=True)
        
        # Print progress
        for i, result in enumerate(self.run_results, 1):
            print(f"[RUN {i}] Complete - Duration: {result['trip_duration']:.2f}s, "
                  f"Distance: {result['trip_distance']:.1f}m")
        
        self.print_summary("V2X ENABLED")
        
        if RECORD_RESULTS:
            self.save_results("v2x")
    
    def run_baseline_simulation(self, num_runs=NUM_RUNS):
        """Run baseline simulation without V2X"""
        print(f"\n{'='*60}")
        print(f"BASELINE AMBULANCE SIMULATION (DEMO MODE)")
        print(f"Configuration:")
        print(f"  - Runs: {num_runs}")
        print(f"  - Traffic Density: {TRAFFIC_DENSITY} vehicles")
        print(f"  - V2X Enabled: NO")
        print(f"  - Map: {MAP_NAME}")
        print(f"{'='*60}\n")
        
        self.run_results = self.generate_realistic_results(num_runs, v2x_enabled=False)
        
        # Print progress
        for i, result in enumerate(self.run_results, 1):
            print(f"[BASELINE RUN {i}] Complete - Duration: {result['trip_duration']:.2f}s, "
                  f"Distance: {result['trip_distance']:.1f}m")
        
        self.print_summary("BASELINE (NO V2X)")
        
        if RECORD_RESULTS:
            self.save_results("baseline")
    
    def print_summary(self, title="SIMULATION SUMMARY"):
        """Print summary statistics"""
        if not self.run_results:
            print("[ERROR] No results to summarize")
            return
        
        durations = [r['trip_duration'] for r in self.run_results]
        distances = [r['trip_distance'] for r in self.run_results]
        speeds = [r['average_speed'] for r in self.run_results]
        
        avg_duration = sum(durations) / len(durations)
        min_duration = min(durations)
        max_duration = max(durations)
        std_dev = self._std_dev(durations)
        
        avg_distance = sum(distances) / len(distances)
        avg_speed = sum(speeds) / len(speeds)
        
        print(f"\n{'='*60}")
        print(f"{title}")
        print(f"{'='*60}")
        print(f"Trip Duration:")
        print(f"  Average: {avg_duration:.2f} seconds")
        print(f"  Min: {min_duration:.2f} seconds")
        print(f"  Max: {max_duration:.2f} seconds")
        print(f"  Std Dev: {std_dev:.2f} seconds")
        print(f"\nTrip Distance:")
        print(f"  Average: {avg_distance:.1f} meters")
        print(f"\nAverage Speed:")
        print(f"  Mean: {avg_speed:.2f} m/s")
        print(f"\nRuns Completed: {len(durations)}/{len(self.run_results)}")
        print(f"{'='*60}\n")
    
    def _std_dev(self, values):
        """Calculate standard deviation"""
        if not values or len(values) < 2:
            return 0
        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        return variance ** 0.5
    
    def save_results(self, scenario_type="v2x"):
        """Save results to JSON file"""
        try:
            if not os.path.exists(RESULTS_DIR):
                os.makedirs(RESULTS_DIR)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            if scenario_type == "v2x":
                filename = os.path.join(RESULTS_DIR, f"v2x_simulation_{timestamp}.json")
                v2x_enabled = True
                v2x_range = V2X_DETECTION_RANGE
            else:
                filename = os.path.join(RESULTS_DIR, f"baseline_simulation_{timestamp}.json")
                v2x_enabled = False
                v2x_range = 0
            
            summary = {
                'configuration': {
                    'map': MAP_NAME,
                    'traffic_density': TRAFFIC_DENSITY,
                    'v2x_enabled': v2x_enabled,
                    'v2x_range': v2x_range,
                    'num_runs': NUM_RUNS,
                    'timestamp': datetime.now().isoformat(),
                    'simulation_mode': 'DEMO (Generated Data)'
                },
                'results': self.run_results
            }
            
            with open(filename, 'w') as f:
                json.dump(summary, f, indent=2, default=str)
            
            print(f"[SAVE] Results saved to {filename}")
            
        except Exception as e:
            print(f"[ERROR] Failed to save results: {e}")


def main():
    """Main entry point"""
    simulation = DemoAmbulanceSimulation()
    
    try:
        # Run both simulations
        simulation.run_baseline_simulation(NUM_RUNS)
        print("\n" + "="*60)
        print("PAUSING BETWEEN TESTS (5 seconds)...")
        print("="*60)
        import time
        time.sleep(5)
        
        simulation.run_v2x_simulation(NUM_RUNS)
        
        print(f"\n{'='*60}")
        print(f"BOTH SIMULATIONS COMPLETE!")
        print(f"Results saved to: {RESULTS_DIR}/")
        print(f"Run 'python analysis.py' to compare results")
        print(f"{'='*60}\n")
        
    except KeyboardInterrupt:
        print("\n[INFO] Simulation interrupted by user")
    except Exception as e:
        print(f"[FATAL] {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
