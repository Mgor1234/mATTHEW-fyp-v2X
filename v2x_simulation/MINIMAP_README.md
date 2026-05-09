"""
V2X Ambulance Simulation - Real-Time Minimap Visualization
===========================================================

This module provides a real-time 2D bird-eye view minimap of the CARLA simulation,
showing vehicle positions and types with color-coded indicators.

## Features

1. **Real-Time Vehicle Tracking**: Displays ambulance and all NPC vehicles with 
   live position updates during simulation

2. **Vehicle Type Color Coding**:
   - WHITE: Ambulance (emergency vehicle)
   - RED: Type A vehicles (CARLA autopilot only - no V2X/LIDAR)
   - YELLOW: Type B vehicles (LIDAR autonomous driving - no V2X communication)
   - GREEN: Type C vehicles (LIDAR autonomous driving + V2X communication)

3. **Visual Elements**:
   - Vehicle dots sized appropriately (ambulance slightly larger)
   - Direction indicators (lines showing vehicle heading)
   - Grid background for reference
   - Legend showing vehicle types and their colors
   - Automatic world coordinate mapping

4. **Separate Window Display**: Opens as a standalone Pygame window, separate from
   CarlaUE4, allowing easy monitoring during simulation

## Configuration

In `config.py`, configure the minimap with these settings:

```python
# Real-Time Minimap Configuration
MINIMAP_ENABLED = True  # Enable/disable real-time minimap display
MINIMAP_WIDTH = 400  # Width in pixels
MINIMAP_HEIGHT = 400  # Height in pixels
MINIMAP_FPS = 30  # Update frequency in frames per second
```

## Usage

The minimap starts automatically when a simulation run begins, assuming:
1. Pygame or PIL is installed (Pygame preferred for interactive features)
2. `MINIMAP_ENABLED = True` in config.py

During simulation, the minimap window will display:
- Ambulance position (white dot)
- All Type A vehicles (red dots) - following CARLA autopilot routes
- All Type B vehicles (yellow dots) - using LIDAR-based autonomous driving
- All Type C vehicles (green dots) - using LIDAR + V2X communication
- Real-time vehicle movements and rotations

## Dependencies

### Required:
- Python 3.7+
- CARLA Python API
- threading (standard library)
- queue (standard library)

### Optional (choose one for rendering):
- **Pygame**: For interactive minimap with live window display
  ```bash
  pip install pygame
  ```
- **PIL (Pillow)**: For static image rendering (fallback if Pygame unavailable)
  ```bash
  pip install Pillow
  ```

## Implementation Details

### Architecture

The minimap uses a multi-threaded architecture:
1. **Main Simulation Thread**: Collects vehicle positions and adds them to update queue
2. **Minimap Thread**: Independently renders the minimap at configured FPS rate

This separation ensures the minimap rendering doesn't block the simulation loop.

### How to Extend

To add custom features to the minimap:

1. **Modify MinimapRenderer class**: Edit rendering logic in `minimap.py`
2. **Add vehicle metadata**: Pass additional fields through MinimapUpdateQueue
3. **Change colors or symbols**: Modify the COLORS dictionary or rendering code

### Integration with Simulation

The following methods integrate minimap into the simulation:

1. `_setup_minimap()`: Initializes minimap thread at simulation start
2. `_update_minimap()`: Updates vehicle positions every simulation frame
3. `_cleanup_minimap()`: Stops minimap thread on simulation end

## Troubleshooting

### Minimap doesn't appear
1. Check if Pygame/PIL is installed: `pip list | grep -E "pygame|Pillow"`
2. Verify `MINIMAP_ENABLED = True` in config.py
3. Check console for error messages starting with [MINIMAP ERROR]

### Minimap is slow
1. Reduce MINIMAP_FPS if it's consuming too much CPU
2. Check if Pygame is properly installed (faster than PIL)

### Vehicle positions not updating
1. Verify CARLA simulation is running normally
2. Check that vehicles are spawning correctly
3. Review simulation console for spawn errors

## Example Output

When running a simulation with minimap enabled, you'll see:

```
[SETUP] Connected to CARLA server at localhost:2000
[SETUP] Loaded map: Town01
[MINIMAP] Real-time minimap started (400x400, 30 FPS)
[SPAWN] Ambulance spawned at Location(x=123.45, y=456.78, z=0.00)
[SPAWN] Spawned 60 NPC vehicles (Type A=20, Type B=20, Type C=20)
```

And a Pygame window will open showing the 2D bird-eye view with colored vehicle dots.

## Performance Impact

- Minimap adds ~5-10ms overhead per simulation frame (rendering in separate thread)
- Memory overhead: ~5-10 MB for minimap display
- CPU usage: Varies based on Pygame efficiency and number of vehicles

To minimize impact, reduce MINIMAP_FPS or disable with `MINIMAP_ENABLED = False`.
"""
