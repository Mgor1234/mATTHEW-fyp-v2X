"""
Standalone script to visualize runtime texture glow effect on buildings.
This script applies orange/emissive glow to nearby buildings and positions
the spectator camera to clearly see the effect.
"""

import carla
import time
import random

# Configuration
CARLA_HOST = "localhost"
CARLA_PORT = 2000
MAP_NAME = "Town01"
GLOW_DURATION = 120  # seconds to display the effect

def apply_glow_to_buildings(world, num_buildings=5):
    """Apply orange/emissive glow texture to buildings."""
    print("\n[GLOW] Querying scene objects...")
    
    try:
        object_names = world.get_names_of_all_objects()
        print(f"[GLOW] Found {len(object_names)} total objects in scene")
    except Exception as e:
        print(f"[ERROR] Could not query objects: {e}")
        return []
    
    # Filter for buildings
    keywords = ["building", "house", "facade", "wall"]
    candidates = [name for name in object_names if any(word in name.lower() for word in keywords)]
    
    print(f"[GLOW] Found {len(candidates)} building-related objects")
    
    if not candidates:
        print("[ERROR] No buildings found to apply glow effect")
        return []
    
    # Show first few candidates
    print(f"[GLOW] Sample building objects:")
    for name in candidates[:10]:
        print(f"  - {name}")
    
    # Create glow textures
    print("\n[GLOW] Creating orange/emissive textures...")
    
    # 2x2 texture for efficiency
    diffuse = carla.TextureColor(2, 2)
    normal = carla.TextureFloatColor(2, 2)
    ao_r_m_e = carla.TextureFloatColor(2, 2)
    empty_specular = carla.TextureFloatColor(0, 0)
    
    # Set bright orange color with high emissive
    for x in range(2):
        for y in range(2):
            # Diffuse: Bright orange
            diffuse.set(x, y, carla.Color(255, 100, 20, 255))
            
            # Normal: Unchanged (neutral normal map)
            normal.set(x, y, carla.FloatColor(0.5, 0.5, 1.0, 1.0))
            
            # AO_R_M_E: (AO, Roughness, Metallic, Emissive)
            # High emissive (8.0) for strong glow
            ao_r_m_e.set(x, y, carla.FloatColor(1.0, 0.7, 0.0, 8.0))
    
    # Apply to buildings
    print(f"\n[GLOW] Applying glow effect to {min(num_buildings, len(candidates))} buildings...")
    applied = []
    
    for i, name in enumerate(candidates[:num_buildings]):
        try:
            world.apply_textures_to_object(name, diffuse, empty_specular, normal, ao_r_m_e)
            applied.append(name)
            print(f"  ✓ Applied to: {name}")
        except Exception as e:
            print(f"  ✗ Failed on {name}: {e}")
            continue
    
    return applied

def position_camera_for_viewing(world, applied_objects):
    """Position spectator camera to view the glowing buildings."""
    print("\n[CAMERA] Positioning spectator for optimal viewing...")
    
    spectator = world.get_spectator()
    
    # Get a good viewing location in Town01
    # Town01 center area around spawn point 1
    view_location = carla.Location(x=150, y=50, z=15)
    view_rotation = carla.Rotation(pitch=-10, yaw=45, roll=0)
    
    spectator.set_transform(carla.Transform(view_location, view_rotation))
    print(f"[CAMERA] Spectator positioned at {view_location}")
    print(f"[CAMERA] Looking at yaw={view_rotation.yaw}, pitch={view_rotation.pitch}")

def main():
    print("=" * 60)
    print("CARLA TEXTURE GLOW EFFECT VISUALIZER")
    print("=" * 60)
    
    client = None
    world = None
    original_weather = None
    
    try:
        # Connect to CARLA
        print(f"\n[SETUP] Connecting to CARLA at {CARLA_HOST}:{CARLA_PORT}...")
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
        
        # Set dark/night weather to make glow more visible
        print("[SETUP] Setting dark weather for better glow visibility...")
        dark_weather = carla.WeatherParameters(
            cloudiness=80.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=0.0,
            sun_altitude_angle=-45.0,  # Night time
            fog_density=20.0,
            fog_distance=50.0,
            wetness=0.0
        )
        world.set_weather(dark_weather)
        world.tick()
        
        print("\n[INFO] Weather set to night time for better glow effect visibility")
        
        # Apply glow effect
        applied = apply_glow_to_buildings(world, num_buildings=10)
        
        if not applied:
            print("\n[ERROR] No glow effects were applied!")
            return
        
        print(f"\n[SUCCESS] Applied glow effect to {len(applied)} objects!")
        
        # Position camera
        position_camera_for_viewing(world, applied)
        
        # Keep scene visible
        print("\n" + "=" * 60)
        print(f"GLOW EFFECT ACTIVE - Displaying for {GLOW_DURATION} seconds")
        print("=" * 60)
        print("\nObjects with glow effect:")
        for obj in applied:
            print(f"  • {obj}")
        print("\n[INFO] Look around the scene to see the orange/emissive glow on buildings")
        print("[INFO] The effect is most visible in dark areas and at night")
        print("[INFO] Move the spectator camera (if in manual mode) to explore")
        print(f"\nPress Ctrl+C to exit early...\n")
        
        # Display the effect
        start_time = time.time()
        while time.time() - start_time < GLOW_DURATION:
            world.tick()
            time.sleep(0.05)
            
            # Print progress every 10 seconds
            elapsed = int(time.time() - start_time)
            if elapsed % 10 == 0 and elapsed > 0:
                remaining = GLOW_DURATION - elapsed
                print(f"[INFO] Time remaining: {remaining} seconds...")
        
        print("\n[INFO] Display duration completed")
        
    except KeyboardInterrupt:
        print("\n\n[INFO] Interrupted by user")
        
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
        
        print("[INFO] Visualization ended")

if __name__ == "__main__":
    main()
