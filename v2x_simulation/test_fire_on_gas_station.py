"""
Test script to verify fire effect on the gas station building
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'PythonAPI', 'carla'))

import carla
import time

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    # Target building identified from logs
    building_name = "Bl_CityBuilding_GasStation_43"
    
    # Create very bright orange emissive texture
    diffuse = carla.TextureColor(2, 2)
    normal = carla.TextureFloatColor(2, 2)
    ao_r_m_e = carla.TextureFloatColor(2, 2)
    empty_specular = carla.TextureFloatColor(0, 0)
    
    for x in range(2):
        for y in range(2):
            diffuse.set(x, y, carla.Color(255, 120, 40, 255))  # Bright orange
            normal.set(x, y, carla.FloatColor(0.5, 0.5, 1.0, 1.0))
            ao_r_m_e.set(x, y, carla.FloatColor(1.0, 0.8, 0.0, 25.0))  # Very high emissive
    
    print(f"[TEST] Applying extreme emissive texture (25.0) to '{building_name}'...")
    try:
        world.apply_textures_to_object(building_name, diffuse, empty_specular, normal, ao_r_m_e)
        print(f"[TEST] SUCCESS - Texture applied to '{building_name}'")
    except Exception as e:
        print(f"[TEST] FAILED - Error: {e}")
        return
    
    # Get all object names to see what buildings exist
    print("\n[TEST] Checking all building names...")
    try:
        object_names = world.get_names_of_all_objects()
        keywords = ["gasstation", "gas", "building"]
        matches = [name for name in object_names if any(word in name.lower() for word in keywords)]
        print(f"[TEST] Found {len(matches)} matching buildings:")
        for name in matches[:10]:  # Show first 10
            print(f"  - {name}")
    except Exception as e:
        print(f"[TEST] Could not query scene objects: {e}")
    
    # Position spectator to look at destination area
    destination = carla.Location(x=299.4, y=133.5, z=10.0)
    spectator = world.get_spectator()
    
    # Position camera to look at destination
    camera_transform = carla.Transform(
        carla.Location(x=destination.x - 30, y=destination.y - 30, z=destination.z + 20),
        carla.Rotation(pitch=-30, yaw=45)
    )
    spectator.set_transform(camera_transform)
    
    print(f"\n[TEST] Camera positioned to view destination at ({destination.x:.1f}, {destination.y:.1f})")
    print(f"[TEST] Look for bright orange glowing building...")
    print(f"[TEST] Keeping effect active for 60 seconds...")
    
    time.sleep(60)
    
    print("[TEST] Test complete")

if __name__ == "__main__":
    main()
