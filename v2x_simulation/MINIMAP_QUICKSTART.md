"""
Quick Start Guide - V2X Simulation Minimap Feature
===================================================

The minimap provides real-time 2D visualization of your V2X ambulance simulation,
showing ambulance and vehicle positions with color-coded indicators.

## Installation

1. Install Pygame (recommended for interactive display):
   ```bash
   pip install pygame
   ```

   Or Pillow (alternative, for static rendering):
   ```bash
   pip install Pillow
   ```

2. Update your simulation configuration (already done, located in config.py):
   ```python
   MINIMAP_ENABLED = True  # Already set to True
   MINIMAP_WIDTH = 400
   MINIMAP_HEIGHT = 400
   MINIMAP_FPS = 30
   ```

## Vehicle Color Legend

When running the simulation with the minimap enabled, you'll see:

- **WHITE dot** ⚪: Ambulance (emergency vehicle)
- **RED dot** 🔴: Type A vehicles (CARLA autopilot only, no V2X/LIDAR)
- **YELLOW dot** 🟡: Type B vehicles (LIDAR autonomous driving, no V2X)
- **GREEN dot** 🟢: Type C vehicles (LIDAR autonomous driving + V2X)

Each dot has a small line indicating the vehicle's direction of travel.

## Visual Features

1. **Grid Background**: Reference grid for tracking relative positions
2. **Border**: Shows map boundaries
3. **Legend**: Top-left corner displays vehicle type colors
4. **Real-time Updates**: Vehicle positions refresh at configured FPS (default: 30)

## How to Use

1. Start your simulation as normal (the minimap launches automatically)
2. A new window titled "V2X Simulation Minimap - Town01" (or your map name) will appear
3. Watch the colored dots move in real-time as vehicles navigate
4. Red dots show basic autopilot behavior (should generally follow predefined routes)
5. Yellow dots show LIDAR-based smart vehicles responding to obstacles (smoother paths)
6. Green dots show V2X-aware vehicles that actively clear path for ambulance
7. White dot shows ambulance position - watch how other vehicles react!

## What the Minimap Demonstrates

By observing the minimap, you can verify that:

### Type A (Red) - Baseline Behavior
- Vehicles follow CARLA's built-in traffic routes
- Minimal adaptation to obstacles
- Used as comparison baseline

### Type B (Yellow) - LIDAR Autonomy
- Vehicles use LIDAR sensors for autonomous driving
- Smoother, more reactive paths around obstacles
- Demonstrates impact of autonomous technology alone

### Type C (Green) - V2X+LIDAR Integration
- Vehicles respond to V2X alerts from ambulance
- Actively make way for emergency vehicle
- Shows the value of V2X communication integrated with autonomy
- Green dots should move aside when ambulance approaches!

## Configuration Options

In `config.py`, you can customize the minimap:

```python
# Enable/disable minimap
MINIMAP_ENABLED = True

# Display size (pixels)
MINIMAP_WIDTH = 400
MINIMAP_HEIGHT = 400

# Update rate (frames per second)
MINIMAP_FPS = 30  # Lower for less CPU usage, higher for smoother updates
```

## Troubleshooting

### Minimap doesn't appear?
- Run: `python -c "import pygame; print('Pygame OK')"` to verify installation
- Check console for [MINIMAP ERROR] messages
- Ensure MINIMAP_ENABLED = True in config.py

### Minimap is jerky or slow?
- Reduce MINIMAP_FPS value, e.g., set to 15-20 instead of 30
- Ensure Pygame is installed (faster than Pillow fallback)

### Can't see vehicle movement?
- Increase window size (MINIMAP_WIDTH/HEIGHT)
- Check that simulation is actually running and spawning vehicles
- Verify spawn counts in console output

## Example Session Output

When you run the simulation, you'll see:

```
[SETUP] Connected to CARLA server at localhost:2000
[SETUP] Loaded map: Town01
[MINIMAP] Real-time minimap started (400x400, 30 FPS)
[SPAWN] Ambulance spawned at Location(x=123.45, y=456.78, z=0.00)
[SPAWN] Spawned 60 NPC vehicles (Type A=20, Type B=20, Type C=20)

[RUN 1] Starting simulation...
```

Plus:
1. CarlaUE4 window showing 3D first-person view of ambulance
2. Separate minimap window showing 2D bird-eye view with colored dots

## Performance

- Minimap adds minimal overhead (~5-10ms per frame in worst case)
- Rendering runs in separate thread, doesn't block simulation
- Memory usage: ~5-10 MB
- CPU impact: Negligible on modern systems

## Next Steps

1. Run your simulation: Your preferred method to start the simulation
2. Watch both windows:
   - CarlaUE4 window: 3D view of the emergency scenario
   - Minimap window: 2D overview of all vehicle positions and types
3. Analyze the vehicle behaviors:
   - Do red vehicles (Type A) block the path longer than yellow/green?
   - Do green vehicles (Type C) respond faster than yellow vehicles?
   - Does the ambulance path differ based on vehicle type distribution?

## Tips for Analysis

- **Full Coverage View**: The minimap provides perspective that's hard to get in 3D view
- **Behavior Comparison**: Compare same run with different Type distributions to see V2X impact
- **Path Planning**: Visualize how vehicles plan routes around ambulance
- **Bottleneck Detection**: Identify where traffic congestion occurs

## Questions or Issues?

Refer to MINIMAP_README.md for detailed technical documentation.
"""
