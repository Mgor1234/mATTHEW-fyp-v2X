# V2X Ambulance Emergency Response Simulation

A CARLA-based simulation demonstrating Vehicle-to-Everything (V2X) technology for autonomous ambulance emergency response in crowded urban environments.

## Overview

This simulation demonstrates how V2X communication allows regular vehicles to detect the presence of an emergency ambulance and automatically clear a path, reducing response times in emergency scenarios.

### Key Features

- **V2X Communication**: Vehicles detect ambulance broadcasts within configurable range (default 150m)
- **Automatic Path Clearing**: NPC vehicles automatically move out of the ambulance's way when detection occurs
- **Multi-run Analysis**: Run multiple simulations and collect average trip times
- **Performance Tracking**: Measure trip duration, distance, and average speed
- **Configurable Parameters**: Easily adjust traffic density, detection range, and vehicle behavior

## Project Structure

```
v2x_simulation/
├── config.py                      # Configuration parameters
├── ambulance_controller.py          # Ambulance vehicle + V2X broadcast logic
├── vehicle_behavior.py              # NPC vehicle behavior + V2X detection
├── simulation.py                    # Main simulation runner
├── analysis.py                      # Results analysis and comparison
├── results/                         # Output directory for simulation results
└── README.md                        # This file
```

## Prerequisites

1. **CARLA Simulator** (v0.9.16) running and listening on localhost:2000
   ```bash
   # Start CARLA (from CARLA directory)
   CarlaUE4\Binaries\Win64\CarlaUE4.exe
   ```

2. **Python 3.7+** with CARLA Python API installed
   ```bash
   pip install carla
   ```

3. **CARLA PythonAPI** agents module must be available
   - The agents module is located in `PythonAPI/carla/agents/`
   - Add to Python path or ensure CARLA Python package includes it

## Configuration

Edit `config.py` to customize simulation parameters:

### Key Parameters

- `NUM_RUNS`: Number of simulation iterations (default: 5)
- `TRAFFIC_DENSITY`: Number of NPC vehicles (default: 50)
- `V2X_DETECTION_RANGE`: Distance vehicles can detect ambulance (default: 150m)
- `MAP_NAME`: CARLA map to use (default: "Town04")
- `AMBULANCE_SPEED`: Max speed for ambulance (default: 30 m/s)
- `PATH_CLEARANCE_URGENCY`: Speed multiplier when clearing path (default: 2.0x)

## Running the Simulation

### 1. Start CARLA Server

```bash
cd E:\CARLA_0.9.16
CarlaUE4\Binaries\Win64\CarlaUE4.exe
```

Wait for the Unreal Engine window to fully load.

### 2. Run the Simulation

```bash
cd E:\CARLA_0.9.16\v2x_simulation
python simulation.py
```

Or launch the scenario UI app:

```bash
python simulation_ui.py
```

The UI allows users to set before each run:
- Type A/B/C vehicle counts
- Number of runs
- Ambulance start and destination waypoint indices
- Storm weather toggle
- Map name

The simulation will:
1. Connect to CARLA
2. Load the specified map
3. Spawn ambulance and NPCs
4. Run the configured number of simulations
5. Print results summary
6. Save results to `results/` directory

Each batch also exports a result-sheet CSV:
- `results/v2x_result_sheet_YYYYMMDD_HHMMSS.csv`

### 3. Analyze Results

```bash
python analysis.py
```

This will:
- Load all results from the `results/` directory
- Print statistics for each run
- Show comparative analysis if multiple result sets exist
- Export results to CSV

## Understanding the Results

### Key Metrics

- **Trip Duration**: Time for ambulance to reach destination
- **Trip Distance**: Actual distance traveled (may differ due to traffic/routing)
- **Average Speed**: Mean speed maintained during trip
- **Standard Deviation**: Variation between runs (lower = more consistent)

### Sample Output

```
============================================================
SIMULATION SUMMARY
============================================================
Trip Duration (with V2X path clearing):
  Average: 45.23 seconds
  Min: 42.15 seconds
  Max: 48.91 seconds
  Std Dev: 2.45 seconds

Average Trip Distance: 850.5 meters
Average Trip Speed: 18.79 m/s
Runs Completed: 5/5
============================================================
```

## How V2X Communication Works in This Simulation

1. **Ambulance Broadcast**: Every time step, the ambulance broadcasts its location and velocity via V2X
2. **Vehicle Detection**: NPC vehicles check if they're within V2X range of the ambulance
3. **Path Clearing**: When detected:
   - Vehicles calculate a clearance vector (perpendicular to ambulance path)
   - Vehicles increase speed and move away from the ambulance
   - Path clearing continues until ambulance exits detection range
4. **Resume Normal Behavior**: Once ambulance is out of range, vehicles return to normal navigation

## Advanced Usage

### Comparing Scenarios

To compare V2X-assisted vs. non-assisted scenarios:

1. Run simulation with current settings: `python simulation.py`
2. Modify `config.py` to set `V2X_DETECTION_RANGE = 0` (disable V2X)
3. Run again: `python simulation.py`
4. Run analysis: `python analysis.py` (will compare both result sets)

### Different Map Scenarios

Edit `config.py` and change `MAP_NAME` to test different urban environments:
- Town01 - Residential area
- Town02 - Highway
- Town03 - Urban with pedestrians
- Town04 - Large city (default)
- Town05 - Highway with traffic
- Town10HD - High-definition urban

### Adjusting Traffic Density

Change `TRAFFIC_DENSITY` to test different congestion levels:
- 20-30: Light traffic
- 40-60: Moderate traffic
- 80+: Heavy congestion

## Troubleshooting

### Connection Error
```
Error connecting to CARLA server
```
- Ensure CARLA is running and fully loaded
- Check that port 2000 is not blocked
- Try: `netstat -ano | findstr :2000`

### No Spawn Points
```
No spawn points available
```
- CARLA may not be fully initialized
- Wait 10+ seconds after launching CARLA
- Try restarting CARLA

### Vehicle Behavior Issues
- If vehicles don't respond to V2X, check `V2X_DETECTION_RANGE` in config
- If ambulance doesn't reach destination, verify map has spawn points
- Enable `DEBUG_PRINT = True` for detailed logs

## Performance Tips

- Reduce `TRAFFIC_DENSITY` for faster simulations
- Use smaller maps (Town02) for quicker tests
- Disable `DRAW_V2X_RANGE` for slightly better performance
- Set `DEBUG_PRINT = False` to reduce console output

## Next Steps

1. **Test Multiple Configurations**: Compare different V2X ranges, traffic densities
2. **Add Visualization**: Use CARLA client recorder to playback scenarios
3. **Extend to Baseline**: Implement non-V2X ambulance behavior for comparison
4. **Add Metrics**: Include fuel/energy consumption, crash avoidance stats
5. **ITS Integration**: Connect to real traffic management systems

## References

- CARLA Documentation: https://carla.readthedocs.io/
- V2X Technology: https://en.wikipedia.org/wiki/Vehicle-to-everything
- Emergency Response Optimization: IEEE papers on emergency vehicle routing

## License

This simulation is provided as-is for research and development purposes.
