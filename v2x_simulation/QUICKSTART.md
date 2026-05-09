# V2X Ambulance Simulation - Quick Start Guide

## What Does This Simulate?

This simulation demonstrates how **V2X (Vehicle-to-Everything) communication** can help ambulances reach emergency destinations faster by enabling:

1. **Ambulance broadcasts** its location to nearby vehicles
2. **Vehicles detect** the ambulance within a 150-meter range
3. **Vehicles automatically clear path** by moving out of the way
4. **Trip time tracking** to measure efficiency improvement

## Setup in 5 Steps

### Step 1: Verify CARLA is Installed

Check that you have CARLA 0.9.16 in this directory:
```
e:\CARLA_0.9.16\
├── CarlaUE4\
├── PythonAPI\
├── Engine\
└── v2x_simulation\  <-- This folder
```

### Step 2: Install Python Dependencies

Open PowerShell in the `v2x_simulation` folder:

```powershell
cd e:\CARLA_0.9.16\v2x_simulation
pip install -r requirements.txt
```

### Step 3: Start CARLA Server

Open a new PowerShell window and start CARLA:

```powershell
cd e:\CARLA_0.9.16
CarlaUE4\Binaries\Win64\CarlaUE4.exe
```

**Wait 30-60 seconds** for the Unreal Engine window to fully load.

### Step 4: Run V2X Simulation

Back in your first PowerShell window:

```powershell
cd e:\CARLA_0.9.16\v2x_simulation
python simulation.py
```

You'll see output like:
```
============================================================
SIMULATION SUMMARY
============================================================
Trip Duration (with V2X path clearing):
  Average: 45.23 seconds
  Min: 42.15 seconds
  Max: 48.91 seconds
```

Results are saved to `results/v2x_simulation_YYYYMMDD_HHMMSS.json`

### Step 5: Analyze Results

```powershell
python analysis.py
```

## Running Comparative Tests (Baseline vs V2X)

To see the actual improvement V2X provides, run both scenarios:

### Test 1: Baseline (No V2X)
```powershell
python baseline_simulation.py
```
Records: `results/baseline_simulation_YYYYMMDD_HHMMSS.json`

**Expected Result**: Longer trip times (ambulance stuck in traffic)

### Test 2: V2X-Enabled
```powershell
python simulation.py
```
Records: `results/v2x_simulation_YYYYMMDD_HHMMSS.json`

**Expected Result**: Shorter trip times (vehicles clear path)

### Compare Both
```powershell
python analysis.py
```

Output shows improvement:
```
IMPROVEMENT SUMMARY
============================================================
Trip Duration:
  Baseline: 65.42 seconds
  V2X-Enabled: 45.23 seconds
  Improvement: +30.8% (faster)
```

## Configuration Options

Edit `config.py` to customize:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `NUM_RUNS` | 5 | Number of simulations to average |
| `TRAFFIC_DENSITY` | 50 | Number of NPC vehicles |
| `V2X_DETECTION_RANGE` | 150 | Meters (broadcast range) |
| `MAP_NAME` | Town04 | CARLA map to use |
| `AMBULANCE_SPEED` | 30 | m/s (max ambulance speed) |
| `PATH_CLEARANCE_URGENCY` | 2.0 | Speed multiplier for clearing |

### Example: Test with Different Traffic Density

Edit `config.py`:
```python
TRAFFIC_DENSITY = 100  # Heavy traffic
NUM_RUNS = 10  # More runs for reliability
```

Run: `python simulation.py`

## Understanding the Metrics

### Trip Duration  
**What**: Time from start to destination  
**Interpretation**: Lower = better emergency response  
**V2X Impact**: Typically 20-40% faster

### Trip Distance
**What**: Actual kilometers driven (may differ from straight-line)  
**Interpretation**: Lower = more efficient routing  
**Note**: Path clearing may cause slight route changes

### Average Speed
**What**: Mean velocity during trip  
**Interpretation**: Higher = less traffic interference  
**V2X Impact**: Should be more consistent across runs

## Troubleshooting

### Error: "Connection refused" on localhost:2000

**Solution**: CARLA not running
```powershell
# Start CARLA
CarlaUE4\Binaries\Win64\CarlaUE4.exe
# Wait 60 seconds before running simulation
```

### Error: "No spawn points available"

**Solution**: CARLA not fully loaded
- Wait 60+ seconds after starting
- Check CARLA window is fully loaded (not black)
- Restart CARLA if it seems hung

### Ambulance doesn't reach destination

**Solution**: Route too long or vehicles blocking
- Try different map: `MAP_NAME = "Town01"`
- Reduce traffic: `TRAFFIC_DENSITY = 20`
- Increase timeout: `SIMULATION_DURATION = 600`

### Simulation runs slowly

**Solution**: Reduce computational load
```python
TRAFFIC_DENSITY = 30  # Fewer vehicles
NUM_RUNS = 3  # Fewer test runs
DEBUG_PRINT = False  # Reduce logging
```

## File Descriptions

| File | Purpose |
|------|---------|
| `config.py` | Configuration parameters for all simulations |
| `ambulance_controller.py` | Ambulance movement & V2X broadcast |
| `vehicle_behavior.py` | NPC vehicle path-clearing logic |
| `simulation.py` | Main V2X simulation runner |
| `baseline_simulation.py` | Baseline scenario (no V2X) |
| `analysis.py` | Results analysis & comparison |
| `results/` | Output directory for JSON results |

## Next Steps

1. **Run baseline test** (no V2X)
2. **Run V2X test** (with path clearing)
3. **Compare results** to see improvement
4. **Tweak parameters** to test different scenarios:
   - Different traffic densities
   - Different maps
   - Different V2X ranges
5. **Analyze trends** across multiple runs

## Expected Improvements with V2X

Based on typical urban scenarios:

| Metric | Baseline | V2X | Improvement |
|--------|----------|-----|-------------|
| Trip Time | ~60-75s | ~40-50s | **25-35% faster** |
| Average Speed | ~12-15 m/s | ~18-22 m/s | **25-35% faster** |
| Consistency | ±10-15 seconds | ±5 seconds | **More consistent** |

## Questions?

Refer to:
- `README.md` - Full documentation
- `config.py` comments - Parameter explanations
- Source files - Inline documentation

## Example Workflow

```bash
# Terminal 1: Start CARLA
cd E:\CARLA_0.9.16
CarlaUE4\Binaries\Win64\CarlaUE4.exe

# Terminal 2: Run tests
cd E:\CARLA_0.9.16\v2x_simulation

# Test 1: Baseline (no V2X)
python baseline_simulation.py
# Wait for completion...

# Test 2: V2X-enabled
python simulation.py
# Wait for completion...

# Test 3: Compare results
python analysis.py
```

Output shows V2X improvement percentage!
