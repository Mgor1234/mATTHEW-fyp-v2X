"""
Enhanced CARLA texture glow effect with animated fire stages.
Creates a procedural "burning" effect by rapidly cycling through
different emissive fire textures to simulate flames and damage.
"""

import carla
import time
import random

# Configuration
CARLA_HOST = "localhost"
CARLA_PORT = 2000
MAP_NAME = "Town01"
GLOW_DURATION = 180  # seconds to display the effect
FIRE_ANIMATION_SPEED = 0.15  # seconds between fire stage changes (0.1-0.3 recommended)

# Fire stages: (brightness, orange_intensity, red_content)
# These create different "stages" of burning intensity
FIRE_STAGES = [
    # Stage 1: Starting fire - dim orange
    {"emissive": 2.0, "r_base": 200, "g_base": 80, "b_base": 10, "intensity": 0.4},
    # Stage 2: Building intensity - bright orange
    {"emissive": 4.0, "r_base": 255, "g_base": 120, "b_base": 20, "intensity": 0.6},
    # Stage 3: Intense orange-red - high heat
    {"emissive": 6.0, "r_base": 255, "g_base": 100, "b_base": 30, "intensity": 0.8},
    # Stage 4: Bright red-orange - peak flame
    {"emissive": 8.0, "r_base": 255, "g_base": 80, "b_base": 40, "intensity": 1.0},
    # Stage 5: Deep red - intense core
    {"emissive": 7.0, "r_base": 220, "g_base": 60, "b_base": 30, "intensity": 0.9},
    # Stage 6: Yellow-white hot spots
    {"emissive": 9.0, "r_base": 255, "g_base": 150, "b_base": 50, "intensity": 1.0},
]

def create_fire_texture(stage_params):
    """Create a fire-stage texture based on parameters."""
    diffuse = carla.TextureColor(2, 2)
    normal = carla.TextureFloatColor(2, 2)
    ao_r_m_e = carla.TextureFloatColor(2, 2)
    
    # Apply stage parameters with slight variation per pixel
    for x in range(2):
        for y in range(2):
            # Add randomness to create flickering effect
            var = random.uniform(0.8, 1.2)
            
            # Apply intensity variation (opacity-like effect through color variation)
            r = int(stage_params["r_base"] * var * stage_params["intensity"])
            g = int(stage_params["g_base"] * var * stage_params["intensity"])
            b = int(stage_params["b_base"] * var * stage_params["intensity"])
            
            # Clamp to valid range
            r = min(255, max(0, r))
            g = min(255, max(0, g))
            b = min(255, max(0, b))
            
            # Diffuse color: Fire colors
            diffuse.set(x, y, carla.Color(r, g, b, 255))
            
            # Normal map: Unchanged
            normal.set(x, y, carla.FloatColor(0.5, 0.5, 1.0, 1.0))
            
            # AO_R_M_E: High emissive for glow, roughness varies with stage
            roughness = 0.9 - (stage_params["emissive"] / 10.0 * 0.3)  # Less rough at higher temps
            ao_r_m_e.set(x, y, carla.FloatColor(1.0, roughness, 0.0, stage_params["emissive"]))
    
    return diffuse, normal, ao_r_m_e

def apply_glow_to_buildings(world, num_buildings=8):
    """Apply orange/emissive glow texture to buildings."""
    print("\n[FIRE] Querying scene objects...")
    
    try:
        object_names = world.get_names_of_all_objects()
        print(f"[FIRE] Found {len(object_names)} total objects in scene")
    except Exception as e:
        print(f"[ERROR] Could not query objects: {e}")
        return []
    
    # Filter for buildings
    keywords = ["building", "house", "facade", "wall"]
    candidates = [name for name in object_names if any(word in name.lower() for word in keywords)]
    
    print(f"[FIRE] Found {len(candidates)} building-related objects")
    print(f"[FIRE] Selecting {min(num_buildings, len(candidates))} for burning effect\n")
    
    if not candidates:
        print("[ERROR] No buildings found to apply effect")
        return []
    
    # Randomly select buildings for visual variety
    selected = random.sample(candidates, min(num_buildings, len(candidates)))
    
    print("[FIRE] Selected buildings to burn:")
    for name in selected:
        print(f"  [FIRE] {name}")
    print()
    
    return selected

def position_camera_for_viewing(world, selected_buildings):
    """Position spectator camera to view the burning buildings."""
    print("[CAMERA] Positioning spectator for optimal viewing...")
    
    spectator = world.get_spectator()
    
    # Get a good viewing location in Town01
    view_location = carla.Location(x=150, y=50, z=20)
    view_rotation = carla.Rotation(pitch=-15, yaw=45, roll=0)
    
    spectator.set_transform(carla.Transform(view_location, view_rotation))
    print(f"[CAMERA] Spectator positioned at {view_location}")
    print(f"[CAMERA] Looking at yaw={view_rotation.yaw}, pitch={view_rotation.pitch}\n")

def animate_fire_effect(world, buildings, duration):
    """Animate fire effect by cycling through fire stages."""
    print("=" * 60)
    print("ANIMATED FIRE EFFECT ACTIVE")
    print("=" * 60)
    print(f"\nFire Stages: {len(FIRE_STAGES)}")
    print(f"Update Speed: Every {FIRE_ANIMATION_SPEED} seconds")
    print(f"Duration: {duration} seconds")
    print(f"Buildings Burning: {len(buildings)}\n")
    
    start_time = time.time()
    last_update = 0
    stage_index = 0
    
    try:
        while time.time() - start_time < duration:
            elapsed = time.time() - start_time
            
            # Update fire stage at animation speed
            if elapsed - last_update >= FIRE_ANIMATION_SPEED:
                # Create texture for current stage
                stage_params = FIRE_STAGES[stage_index % len(FIRE_STAGES)]
                diffuse, normal, ao_r_m_e = create_fire_texture(stage_params)
                empty_specular = carla.TextureFloatColor(0, 0)
                
                # Apply to all buildings
                for building in buildings:
                    try:
                        world.apply_textures_to_object(building, diffuse, empty_specular, normal, ao_r_m_e)
                    except Exception as e:
                        pass  # Building may be destroyed, silent fail
                
                # Print stage info
                stage_name = [
                    "Starting Fire",
                    "Building Intensity",
                    "Intense Orange-Red",
                    "Peak Flame",
                    "Deep Red Core",
                    "Yellow-White Hot"
                ][stage_index % len(FIRE_STAGES)]
                
                print(f"[FIRE] Stage {stage_index % len(FIRE_STAGES) + 1}/6: {stage_name:20s} | " +
                      f"Emissive: {stage_params['emissive']:.1f} | " +
                      f"Time: {elapsed:.1f}s / {duration}s")
                
                stage_index += 1
                last_update = elapsed
            
            # Minimal tick
            world.tick()
            time.sleep(0.01)  # 100Hz update for smooth simulation
    
    except KeyboardInterrupt:
        print("\n\n[INFO] Interrupted by user")
    
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()

def main():
    print("=" * 60)
    print("CARLA ANIMATED FIRE EFFECT VISUALIZER")
    print("=" * 60)
    print("\nFeatures:")
    print("  • Rapidly cycling fire stages (6 stages)")
    print("  • Procedural burning effect with color variations")
    print("  • Emissive glow increasing with fire intensity")
    print("  • Flickering simulation using per-pixel randomness")
    print("  • Independent animation loop per frame\n")
    
    client = None
    world = None
    original_weather = None
    
    try:
        # Connect to CARLA
        print(f"[SETUP] Connecting to CARLA at {CARLA_HOST}:{CARLA_PORT}...")
        client = carla.Client(CARLA_HOST, CARLA_PORT)
        client.set_timeout(60.0)
        
        # Load map
        print(f"[SETUP] Loading map: {MAP_NAME}...")
        world = client.load_world(MAP_NAME)
        
        # Set synchronous mode
        print("[SETUP] Setting synchronous mode...")
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        
        # Store original weather
        original_weather = world.get_weather()
        
        # Set dark/night weather to make fire glow extremely visible
        print("[SETUP] Setting dark weather for maximum fire visibility...")
        dark_weather = carla.WeatherParameters(
            cloudiness=85.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=5.0,  # Some wind to simulate fire movement
            sun_altitude_angle=-50.0,  # Deep night
            fog_density=30.0,
            fog_distance=80.0,
            wetness=0.0
        )
        world.set_weather(dark_weather)
        world.tick()
        
        print("[SETUP] [OK] Night time with fog for dramatic effect\n")
        
        # Get buildings
        buildings = apply_glow_to_buildings(world, num_buildings=8)
        
        if not buildings:
            print("\n[ERROR] No buildings to animate!")
            return
        
        # Position camera
        position_camera_for_viewing(world, buildings)
        
        # Run animation
        print("[INFO] Press Ctrl+C to exit early...\n")
        animate_fire_effect(world, buildings, GLOW_DURATION)
        
        print("\n[INFO] Animation duration completed")
        
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        print("\n[CLEANUP] Restoring settings...")
        if world:
            try:
                # Restore original weather
                if original_weather:
                    world.set_weather(original_weather)
                
                # Restore asynchronous mode
                settings = world.get_settings()
                settings.synchronous_mode = False
                world.apply_settings(settings)
                
                print("[CLEANUP] Settings restored")
            except Exception as e:
                print(f"[WARN] Cleanup error: {e}")
        
        print("[INFO] Fire animation ended")

if __name__ == "__main__":
    main()
