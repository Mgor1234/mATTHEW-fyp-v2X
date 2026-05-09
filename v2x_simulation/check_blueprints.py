#!/usr/bin/env python3
"""Check available vehicle blueprints in CARLA"""

import carla

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    
    print("\n" + "="*60)
    print("AMBULANCE BLUEPRINTS:")
    print("="*60)
    ambulances = blueprint_library.filter("vehicle.*ambulance*")
    if ambulances:
        for bp in ambulances:
            print(f"  - {bp.id}")
    else:
        print("  No ambulance blueprints found!")
    
    print("\n" + "="*60)
    print("POLICE BLUEPRINTS:")
    print("="*60)
    police = blueprint_library.filter("vehicle.*police*")
    if police:
        for bp in police:
            print(f"  - {bp.id}")
    else:
        print("  No police blueprints found!")
    
    print("\n" + "="*60)
    print("ALL VEHICLE BLUEPRINTS (first 20):")
    print("="*60)
    all_vehicles = blueprint_library.filter("vehicle.*")
    for i, bp in enumerate(all_vehicles[:20]):
        print(f"  {i+1}. {bp.id}")
    print(f"\n  Total vehicles: {len(all_vehicles)}")
    print("="*60 + "\n")

if __name__ == '__main__':
    main()
