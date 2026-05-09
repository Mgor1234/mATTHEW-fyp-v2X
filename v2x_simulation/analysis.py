"""
Results Analysis - Analyze and compare V2X simulation results
"""

import json
import os
from datetime import datetime
import statistics


class ResultsAnalyzer:
    """Analyze V2X simulation results"""
    
    def __init__(self, results_dir="./results"):
        self.results_dir = results_dir
        self.results = []
    
    def load_results(self, filename=None):
        """
        Load results from JSON file
        
        Args:
            filename: Specific file to load, or None to load all files
        """
        if not os.path.exists(self.results_dir):
            print(f"[ERROR] Results directory not found: {self.results_dir}")
            return False
        
        if filename:
            filepath = os.path.join(self.results_dir, filename)
            if os.path.exists(filepath):
                with open(filepath, 'r') as f:
                    self.results.append(json.load(f))
            else:
                print(f"[ERROR] File not found: {filepath}")
                return False
        else:
            # Load all JSON files
            for file in sorted(os.listdir(self.results_dir)):
                if file.endswith('.json'):
                    filepath = os.path.join(self.results_dir, file)
                    try:
                        with open(filepath, 'r') as f:
                            self.results.append(json.load(f))
                        print(f"[LOAD] Loaded: {file}")
                    except Exception as e:
                        print(f"[WARN] Failed to load {file}: {e}")
        
        return len(self.results) > 0
    
    def analyze_single_result(self, result_data):
        """Analyze a single simulation result set"""
        runs = result_data.get('results', [])
        
        if not runs:
            print("[ERROR] No runs in result data")
            return None
        
        durations = [r['trip_duration'] for r in runs if r.get('trip_duration')]
        distances = [r['trip_distance'] for r in runs if r.get('trip_distance')]
        speeds = [r['average_speed'] for r in runs if r.get('average_speed')]
        
        analysis = {
            'num_runs': len(runs),
            'successful_runs': len(durations),
            'configuration': result_data.get('configuration', {}),
            'metrics': {}
        }
        
        if durations:
            analysis['metrics']['duration'] = {
                'mean': statistics.mean(durations),
                'median': statistics.median(durations),
                'min': min(durations),
                'max': max(durations),
                'stdev': statistics.stdev(durations) if len(durations) > 1 else 0,
                'all_values': durations
            }
        
        if distances:
            analysis['metrics']['distance'] = {
                'mean': statistics.mean(distances),
                'median': statistics.median(distances),
                'min': min(distances),
                'max': max(distances),
                'stdev': statistics.stdev(distances) if len(distances) > 1 else 0
            }
        
        if speeds:
            analysis['metrics']['speed'] = {
                'mean': statistics.mean(speeds),
                'median': statistics.median(speeds),
                'min': min(speeds),
                'max': max(speeds),
                'stdev': statistics.stdev(speeds) if len(speeds) > 1 else 0
            }
        
        return analysis
    
    def print_analysis(self, analysis, title="Simulation Analysis"):
        """Print formatted analysis"""
        print(f"\n{'='*70}")
        print(f"{title}")
        print(f"{'='*70}")
        
        config = analysis.get('configuration', {})
        print(f"\nConfiguration:")
        print(f"  Map: {config.get('map', 'N/A')}")
        print(f"  Traffic Density: {config.get('traffic_density', 'N/A')} vehicles")
        print(f"  V2X Range: {config.get('v2x_range', 'N/A')}m")
        print(f"  Test Date: {config.get('timestamp', 'N/A')}")
        
        print(f"\nRun Statistics:")
        print(f"  Total Runs: {analysis.get('num_runs')}")
        print(f"  Successful Runs: {analysis.get('successful_runs')}")
        
        metrics = analysis.get('metrics', {})
        
        if 'duration' in metrics:
            dur = metrics['duration']
            print(f"\nTrip Duration (seconds):")
            print(f"  Mean: {dur['mean']:.2f}s")
            print(f"  Median: {dur['median']:.2f}s")
            print(f"  Min: {dur['min']:.2f}s")
            print(f"  Max: {dur['max']:.2f}s")
            print(f"  Std Dev: {dur['stdev']:.2f}s")
        
        if 'distance' in metrics:
            dist = metrics['distance']
            print(f"\nTrip Distance (meters):")
            print(f"  Mean: {dist['mean']:.1f}m")
            print(f"  Median: {dist['median']:.1f}m")
            print(f"  Min: {dist['min']:.1f}m")
            print(f"  Max: {dist['max']:.1f}m")
        
        if 'speed' in metrics:
            spd = metrics['speed']
            print(f"\nAverage Speed (m/s):")
            print(f"  Mean: {spd['mean']:.2f} m/s")
            print(f"  Median: {spd['median']:.2f} m/s")
            print(f"  Min: {spd['min']:.2f} m/s")
            print(f"  Max: {spd['max']:.2f} m/s")
        
        print(f"\n{'='*70}\n")
    
    def compare_results(self):
        """Compare multiple simulation results (e.g., with/without V2X)"""
        if len(self.results) < 2:
            print("[WARN] Need at least 2 result sets to compare")
            return
        
        analyses = []
        for i, result in enumerate(self.results):
            analysis = self.analyze_single_result(result)
            if analysis:
                analyses.append(analysis)
        
        if len(analyses) < 2:
            print("[ERROR] Not enough valid results to compare")
            return
        
        print(f"\n{'='*70}")
        print(f"COMPARATIVE ANALYSIS ({len(analyses)} test sets)")
        print(f"{'='*70}\n")
        
        # Print individual analyses
        for i, analysis in enumerate(analyses):
            title = f"Result Set {i+1}"
            config = analysis.get('configuration', {})
            if 'map' in config:
                title += f" - {config.get('map')}"
            self.print_analysis(analysis, title)
        
        # Print comparison
        if 'duration' in analyses[0].get('metrics', {}) and \
           'duration' in analyses[1].get('metrics', {}):
            
            dur1 = analyses[0]['metrics']['duration']['mean']
            dur2 = analyses[1]['metrics']['duration']['mean']
            improvement = ((dur1 - dur2) / dur1 * 100) if dur1 != 0 else 0
            
            print(f"{'='*70}")
            print(f"IMPROVEMENT SUMMARY")
            print(f"{'='*70}")
            print(f"\nTrip Duration:")
            print(f"  Result Set 1: {dur1:.2f}s")
            print(f"  Result Set 2: {dur2:.2f}s")
            print(f"  Improvement: {improvement:+.1f}% {'(faster)' if improvement > 0 else '(slower)'}")
            print(f"\n{'='*70}\n")
    
    def export_comparison_csv(self, output_file="comparison.csv"):
        """Export results to CSV format"""
        try:
            with open(output_file, 'w') as f:
                f.write("Result Set,Metric,Mean,Median,Min,Max,StdDev\n")
                
                for i, result in enumerate(self.results):
                    analysis = self.analyze_single_result(result)
                    if not analysis:
                        continue
                    
                    metrics = analysis.get('metrics', {})
                    for metric_name, metric_data in metrics.items():
                        f.write(f"{i+1},{metric_name},"
                               f"{metric_data['mean']:.2f},"
                               f"{metric_data['median']:.2f},"
                               f"{metric_data['min']:.2f},"
                               f"{metric_data['max']:.2f},"
                               f"{metric_data['stdev']:.2f}\n")
            
            print(f"[EXPORT] Results exported to {output_file}")
        except Exception as e:
            print(f"[ERROR] Failed to export: {e}")


def main():
    """Main analysis entry point"""
    analyzer = ResultsAnalyzer()
    
    print("[INFO] V2X Ambulance Simulation - Results Analyzer")
    print("[INFO] Loading results from ./results/...\n")
    
    if analyzer.load_results():
        if len(analyzer.results) == 1:
            analysis = analyzer.analyze_single_result(analyzer.results[0])
            if analysis:
                analyzer.print_analysis(analysis, "V2X Ambulance Simulation - Results")
        else:
            analyzer.compare_results()
            analyzer.export_comparison_csv()
    else:
        print("[ERROR] No results found")


if __name__ == "__main__":
    main()
