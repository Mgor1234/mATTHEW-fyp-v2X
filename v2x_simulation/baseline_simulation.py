"""
Baseline Simulation - Ambulance with proximity-based path clearing (no V2X)
Vehicles detect ambulance through proximity (simulating visual/audio detection)
Use for comparison against V2X-enabled simulation
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'PythonAPI', 'carla'))

import carla
import time
import traceback
from datetime import datetime
from ambulance_controller import AmbulanceController
from baseline_vehicle_behavior import BaselineVehicleBehavior
from config import *
import json


class BaselineAmbulanceSimulation:
    def __init__(self):
        self.client = None
        self.world = None
        self.traffic_manager = None
        self.vehicles = []
        self.vehicle_behaviors = []  # Behavior controllers for NPCs
        self.ambulance = None
        self.ambulance_controller = None
        self.run_results = []
        self.original_weather = None
        # Fire animation state
        self.animated_buildings = []
        self.fire_animation_index = 0
        self.fire_animation_last_update = 0.0

    def _clear_existing_actors(self):
        """Remove lingering vehicles or walkers from previous runs."""
        try:
            actors = self.world.get_actors()
            for actor in actors.filter("vehicle.*"):
                actor.destroy()
            for actor in actors.filter("walker.*"):
                actor.destroy()
        except Exception as e:
            if DEBUG_PRINT:
                print(f"[WARN] Failed to clear existing actors: {e}")
    
    def _disable_all_traffic_lights(self):
        """Freeze all traffic lights to green."""
        try:
            actors = self.world.get_actors()
            traffic_lights = actors.filter("traffic.traffic_light")
            for light in traffic_lights:
                light.set_state(carla.TrafficLightState.Green)
                light.set_green_time(999999.0)
                light.freeze(True)
            if traffic_lights:
                print(f"[SETUP] Disabled {len(traffic_lights)} traffic lights (all green)")
        except Exception as e:
            if DEBUG_PRINT:
                print(f"[WARN] Failed to disable traffic lights: {e}")

    def _get_driving_waypoint(self, location):
        """Return the nearest drivable waypoint for a location."""
        world_map = self.world.get_map()
        try:
            return world_map.get_waypoint(
                location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
        except TypeError:
            return world_map.get_waypoint(location)

    def _snap_transform_to_driving_lane(self, transform):
        """Snap a transform to the center of a drivable lane when possible."""
        waypoint = self._get_driving_waypoint(transform.location)
        return waypoint.transform if waypoint else transform

    def _select_ambulance_blueprint(self):
        """Select a vehicle blueprint suitable for emergency driving."""
        blueprint_library = self.world.get_blueprint_library()

        # Prioritize actual ambulance models first
        preferred = []
        preferred.extend(blueprint_library.filter("vehicle.*ambulance*"))
        preferred.extend(blueprint_library.filter("vehicle.ford.ambulance"))
        preferred.extend(blueprint_library.filter("vehicle.mercedes.ambulance"))
        preferred.extend(blueprint_library.filter("vehicle.*police*"))
        preferred.extend(blueprint_library.filter("vehicle.tesla.model3"))
        preferred.extend(blueprint_library.filter("vehicle.car.*"))
        preferred.extend(blueprint_library.filter("vehicle"))

        return preferred[0] if preferred else None

    def _try_spawn_vehicle(self, blueprint, spawn_points, label="vehicle"):
        """Attempt to spawn a vehicle at multiple spawn points."""
        for transform in spawn_points:
            snapped = self._snap_transform_to_driving_lane(transform)
            actor = self.world.try_spawn_actor(blueprint, snapped)
            if actor is None:
                adjusted = carla.Transform(
                    carla.Location(
                        x=snapped.location.x,
                        y=snapped.location.y,
                        z=snapped.location.z + 0.5
                    ),
                    snapped.rotation
                )
                actor = self.world.try_spawn_actor(blueprint, adjusted)
            if actor is None:
                adjusted = carla.Transform(
                    carla.Location(
                        x=snapped.location.x,
                        y=snapped.location.y,
                        z=snapped.location.z + 1.0
                    ),
                    snapped.rotation
                )
                actor = self.world.try_spawn_actor(blueprint, adjusted)
            if actor is not None:
                if DEBUG_PRINT:
                    print(f"[SPAWN] {label} spawned at {actor.get_location()}")
                return actor
        return None

    def _select_destination(self, spawn_points, start_location):
        """Pick a drivable destination that is not too close to the start."""
        if not spawn_points:
            return None

        if 0 <= AMBULANCE_END_WAYPOINT < len(spawn_points):
            candidate = spawn_points[AMBULANCE_END_WAYPOINT]
            waypoint = self._get_driving_waypoint(candidate.location)
            if waypoint and start_location.distance(waypoint.transform.location) > 50.0:
                return waypoint.transform.location

        candidates = []
        for transform in spawn_points:
            waypoint = self._get_driving_waypoint(transform.location)
            if not waypoint:
                continue
            distance = start_location.distance(waypoint.transform.location)
            if distance > 50.0:
                candidates.append((distance, waypoint.transform.location))

        if not candidates:
            fallback = spawn_points[min(len(spawn_points) - 1, 10)].location
            waypoint = self._get_driving_waypoint(fallback)
            return waypoint.transform.location if waypoint else fallback

        import random
        return random.choice(candidates)[1]

    def _safe_tick(self, retries=TICK_RETRY_COUNT):
        """Tick the world with retry on transient simulator timeout."""
        for attempt in range(1, retries + 1):
            try:
                return self.world.tick()
            except RuntimeError as e:
                if "time-out" not in str(e).lower() or attempt == retries:
                    raise
                print(f"[WARN] world.tick timeout (attempt {attempt}/{retries}), retrying...")
                time.sleep(TICK_RETRY_DELAY)

    def _apply_disaster_weather(self, mode):
        """Apply configurable weather to mimic disaster conditions."""
        weather = self.world.get_weather()

        presets = {
            "storm": {
                "cloudiness": 95.0,
                "precipitation": 95.0,
                "precipitation_deposits": 100.0,
                "wind_intensity": 95.0,
                "fog_density": 55.0,
                "fog_distance": 35.0,
                "fog_falloff": 0.7,
                "wetness": 100.0,
                "sun_altitude_angle": -5.0,
            },
            "flood": {
                "cloudiness": 90.0,
                "precipitation": 80.0,
                "precipitation_deposits": 100.0,
                "wind_intensity": 60.0,
                "fog_density": 40.0,
                "fog_distance": 45.0,
                "fog_falloff": 0.5,
                "wetness": 100.0,
                "sun_altitude_angle": 5.0,
            },
            "fire_glow": {
                "cloudiness": 70.0,
                "precipitation": 0.0,
                "precipitation_deposits": 15.0,
                "wind_intensity": 25.0,
                "fog_density": 30.0,
                "fog_distance": 30.0,
                "fog_falloff": 0.8,
                "wetness": 20.0,
                "sun_altitude_angle": -8.0,
            },
        }

        values = presets.get(mode, presets["storm"])
        weather.cloudiness = values["cloudiness"]
        weather.precipitation = values["precipitation"]
        weather.precipitation_deposits = values["precipitation_deposits"]
        weather.wind_intensity = values["wind_intensity"]
        weather.fog_density = values["fog_density"]
        weather.fog_distance = values["fog_distance"]
        weather.fog_falloff = values["fog_falloff"]
        weather.wetness = values["wetness"]
        weather.sun_altitude_angle = values["sun_altitude_angle"]
        self.world.set_weather(weather)
        print(f"[VISUAL] Weather preset applied: {mode}")

    def _apply_runtime_glow_textures(self, destination=None, max_targets=1):
        """Apply runtime texture tinting to a building near the destination.
        Returns tuple: (list of buildings that were tinted, location of selected building if available)."""
        if not hasattr(self.world, "get_names_of_all_objects") or not hasattr(self.world, "apply_textures_to_object"):
            print("[VISUAL] Runtime texture API unavailable in this CARLA build")
            return []

        try:
            object_names = self.world.get_names_of_all_objects()
        except Exception as e:
            if DEBUG_PRINT:
                print(f"[VISUAL] Could not query scene objects: {e}")
            return []

        keywords = ["building", "house", "facade", "wall"]
        candidates = [name for name in object_names if any(word in name.lower() for word in keywords)]

        if not candidates:
            print("[VISUAL] No scene objects matched for runtime texture tinting")
            return [], None

        # Select one building (prefer a named target or house-like object).
        selected = []
        target_name = (FIRE_TARGET_BUILDING_NAME or "").strip()
        if target_name:
            matched = next((name for name in candidates if name.lower() == target_name.lower()), None)
            if matched:
                selected = [matched]
                print(f"[VISUAL] Selected target building '{matched}' for animated fire effect")
            else:
                print(f"[VISUAL] Requested target building not found: {target_name}")

        if not selected and destination:
            keyword_candidates = [
                name for name in candidates
                if any(keyword in name.lower() for keyword in FIRE_TARGET_BUILDING_KEYWORDS)
            ]
            if FIRE_TARGET_DEBUG_LIST and keyword_candidates:
                ordered_candidates = sorted(keyword_candidates)[:FIRE_TARGET_DEBUG_LIMIT]
                print("[VISUAL] House-like candidate objects:")
                for index, name in enumerate(ordered_candidates, start=1):
                    print(f"  [VISUAL] {index:02d}. {name}")
                if FIRE_TARGET_PROMPT:
                    try:
                        selection = input("Select house index for fire target (blank to skip): ").strip()
                        if selection:
                            choice = int(selection)
                            if 1 <= choice <= len(ordered_candidates):
                                selected = [ordered_candidates[choice - 1]]
                                print(f"[VISUAL] Selected house candidate '{selected[0]}' for animated fire effect")
                    except (ValueError, EOFError):
                        print("[VISUAL] Invalid selection; continuing with default target.")
            if keyword_candidates and not selected:
                import random
                # Group buildings by base name (e.g., 'BP_House12' or 'Bl_House_AmerSuburb009')
                building_groups = {}
                for name in keyword_candidates:
                    # Extract base name (remove trailing numbers/suffixes)
                    import re
                    base = re.sub(r'[_N]?\d+$', '', name)
                    if base not in building_groups:
                        building_groups[base] = []
                    building_groups[base].append(name)
                
                # Select a group with multiple buildings (for street effect)
                valid_groups = {k: v for k, v in building_groups.items() if len(v) >= FIRE_TARGET_BUILDING_COUNT}
                if valid_groups:
                    selected_group = random.choice(list(valid_groups.keys()))
                    selected = random.sample(building_groups[selected_group], min(FIRE_TARGET_BUILDING_COUNT, len(building_groups[selected_group])))
                    print(f"[VISUAL] Selected {len(selected)} buildings from '{selected_group}' series for street fire effect")
                else:
                    # Fallback: just pick random buildings
                    selected = random.sample(keyword_candidates, min(FIRE_TARGET_BUILDING_COUNT, len(keyword_candidates)))
                    print(f"[VISUAL] Selected {len(selected)} random buildings for fire effect")

        if not selected:
            selected = candidates[:max_targets]

        # Create base orange glow texture
        diffuse = carla.TextureColor(2, 2)
        normal = carla.TextureFloatColor(2, 2)
        ao_r_m_e = carla.TextureFloatColor(2, 2)
        empty_specular = carla.TextureFloatColor(0, 0)

        for x in range(2):
            for y in range(2):
                diffuse.set(x, y, carla.Color(255, 60, 10, 255))  # Brighter red-orange
                normal.set(x, y, carla.FloatColor(0.5, 0.5, 1.0, 1.0))
                ao_r_m_e.set(x, y, carla.FloatColor(1.0, 0.85, 0.0, 10.0))  # Higher emissive

        tinted_buildings = []
        for name in selected:
            try:
                self.world.apply_textures_to_object(name, diffuse, empty_specular, normal, ao_r_m_e)
                tinted_buildings.append(name)
                print(f"[VISUAL] Applied base orange glow to '{name}' (emissive: 10.0)")
            except Exception as e:
                print(f"[VISUAL] Failed to apply texture to '{name}': {e}")
                continue

        print(f"[VISUAL] Runtime texture tint applied to {len(tinted_buildings)} destination building(s)")
        
        # Try to get a rough location near the selected building (using spawn points as proxy)
        building_location = destination  # Default to original destination
        return tinted_buildings, building_location

    def _create_fire_texture(self, stage_params):
        """Create fire-stage texture based on parameters."""
        import random
        
        diffuse = carla.TextureColor(2, 2)
        normal = carla.TextureFloatColor(2, 2)
        ao_r_m_e = carla.TextureFloatColor(2, 2)
        
        for x in range(2):
            for y in range(2):
                # Add randomness for flickering
                var = random.uniform(0.8, 1.2)
                
                r = int(stage_params["r_base"] * var * stage_params["intensity"])
                g = int(stage_params["g_base"] * var * stage_params["intensity"])
                b = int(stage_params["b_base"] * var * stage_params["intensity"])
                
                r = min(255, max(0, r))
                g = min(255, max(0, g))
                b = min(255, max(0, b))
                
                diffuse.set(x, y, carla.Color(r, g, b, 255))
                normal.set(x, y, carla.FloatColor(0.5, 0.5, 1.0, 1.0))
                
                roughness = 0.9 - (stage_params["emissive"] / 10.0 * 0.3)
                ao_r_m_e.set(x, y, carla.FloatColor(1.0, roughness, 0.0, stage_params["emissive"]))
        
        return diffuse, normal, ao_r_m_e

    def _update_fire_animation(self, elapsed_time):
        """Update fire animation on buildings for fire_glow preset."""
        if DISASTER_VISUAL_MODE != "fire_glow" or not self.animated_buildings:
            return
        
        # Update fire stage based on animation speed
        if elapsed_time - self.fire_animation_last_update >= FIRE_ANIMATION_SPEED:
            stage_params = FIRE_STAGES[self.fire_animation_index % len(FIRE_STAGES)]
            diffuse, normal, ao_r_m_e = self._create_fire_texture(stage_params)
            empty_specular = carla.TextureFloatColor(0, 0)
            
            # Apply to all animated buildings
            for building in self.animated_buildings:
                try:
                    self.world.apply_textures_to_object(building, diffuse, empty_specular, normal, ao_r_m_e)
                    if DEBUG_PRINT and self.fire_animation_index < 5:
                        print(f"[FIRE] Applied stage {self.fire_animation_index % len(FIRE_STAGES)}/6 to '{building}' (emissive: {stage_params['emissive']})")
                except Exception as e:
                    if DEBUG_PRINT:
                        print(f"[FIRE] Failed to update '{building}': {e}")
            
            self.fire_animation_index += 1
            self.fire_animation_last_update = elapsed_time

    def _spawn_disaster_markers(self, location):
        """Spawn a small, strategic emergency marker scene near destination."""
        import math

        blueprint_library = self.world.get_blueprint_library()
        spawned = 0

        cone_bp = blueprint_library.find("static.prop.constructioncone")
        if cone_bp:
            for i in range(8):
                angle = i * (2.0 * math.pi / 8.0)
                transform = carla.Transform(
                    carla.Location(
                        x=location.x + 4.0 * math.cos(angle),
                        y=location.y + 4.0 * math.sin(angle),
                        z=location.z + 0.2,
                    ),
                    carla.Rotation(),
                )
                actor = self.world.try_spawn_actor(cone_bp, transform)
                if actor:
                    self.vehicles.append(actor)
                    spawned += 1

        warning_bp = blueprint_library.find("static.prop.trafficwarning")
        if warning_bp:
            for yaw, radius in [(0, 6.0), (90, 6.0), (180, 6.0), (270, 6.0)]:
                angle = math.radians(yaw)
                transform = carla.Transform(
                    carla.Location(
                        x=location.x + radius * math.cos(angle),
                        y=location.y + radius * math.sin(angle),
                        z=location.z + 0.5,
                    ),
                    carla.Rotation(yaw=yaw),
                )
                actor = self.world.try_spawn_actor(warning_bp, transform)
                if actor:
                    self.vehicles.append(actor)
                    spawned += 1

        chain_bp = blueprint_library.find("static.prop.chainbarrier")
        if chain_bp:
            for yaw in [45, 225]:
                angle = math.radians(yaw)
                transform = carla.Transform(
                    carla.Location(
                        x=location.x + 7.5 * math.cos(angle),
                        y=location.y + 7.5 * math.sin(angle),
                        z=location.z,
                    ),
                    carla.Rotation(yaw=yaw + 45),
                )
                actor = self.world.try_spawn_actor(chain_bp, transform)
                if actor:
                    self.vehicles.append(actor)
                    spawned += 1

        print(f"[VISUAL] Disaster markers spawned: {spawned}")
        return spawned

    def _apply_disaster_visuals(self, destination):
        """Apply all visual-disaster effects supported at runtime.
        
        Note: Buildings are selected randomly for fire effects but their exact world 
        locations cannot be queried from CARLA's static mesh API. The destination 
        parameter is used as a hint but fire buildings may appear anywhere on the map.
        """
        mode = str(DISASTER_VISUAL_MODE).lower()

        if mode == "off":
            print("[VISUAL] Disaster visuals disabled (mode=off)")
            return

        print(f"[VISUAL] Applying disaster visuals (mode={mode})...")
        self._apply_disaster_weather(mode)

        if DISASTER_RUNTIME_TEXTURE_ENABLED:
            # Apply fire animation to random building (cosmetic effect)
            self.animated_buildings, _ = self._apply_runtime_glow_textures(destination=destination)
            if mode == "fire_glow" and self.animated_buildings:
                building_list = "', '".join(self.animated_buildings[:3])
                if len(self.animated_buildings) > 3:
                    building_list += f"' + {len(self.animated_buildings)-3} more"
                print(f"[VISUAL] Fire animation enabled on {len(self.animated_buildings)} buildings: '{building_list}'")
                print(f"[VISUAL] Note: Building locations not queryable - fire may not be at exact destination")
        
    def setup(self):
        """Connect to CARLA and setup world"""
        try:
            self.client = carla.Client(CARLA_HOST, CARLA_PORT)
            self.client.set_timeout(CARLA_TIMEOUT)
            
            print(f"[SETUP] Connected to CARLA server at {CARLA_HOST}:{CARLA_PORT}")
            
            # Load map
            self.world = self.client.load_world(MAP_NAME)
            self.traffic_manager = self.client.get_trafficmanager()
            self.traffic_manager.set_global_distance_to_leading_vehicle(5.0)
            self.original_weather = self.world.get_weather()
            
            print(f"[SETUP] Loaded map: {MAP_NAME}")

            self._clear_existing_actors()
            if DISABLE_TRAFFIC_LIGHTS:
                self._disable_all_traffic_lights()
            
            # Set synchronous mode
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            self.world.apply_settings(settings)
            
            print("[SETUP] Synchronous mode enabled (50ms tick)")

            if DEBUG_PRINT:
                spawn_points = self.world.get_map().get_spawn_points()
                print(f"[SETUP] Spawn points available: {len(spawn_points)}")

            # Give the world time to settle after loading
            for _ in range(5):
                self._safe_tick()
            
        except Exception as e:
            print(f"[ERROR] Setup failed: {e}")
            traceback.print_exc()
            raise
    
    def spawn_ambulance(self):
        """Spawn ambulance vehicle"""
        try:
            world_map = self.world.get_map()
            spawn_points = world_map.get_spawn_points()
            
            if not spawn_points:
                print("[ERROR] No spawn points available")
                return False
            
            ambulance_bp = self._select_ambulance_blueprint()

            if ambulance_bp is None:
                print("[ERROR] No vehicle blueprints available for ambulance")
                return False

            self.ambulance = self._try_spawn_vehicle(ambulance_bp, spawn_points, label="Ambulance")
            if self.ambulance is None:
                print("[ERROR] Failed to spawn ambulance at any spawn point")
                return False
            self.ambulance_controller = AmbulanceController(self.ambulance, world_map, self.world)

            # Ensure ambulance is snapped to the nearest drivable lane
            try:
                current = self.ambulance.get_location()
                waypoint = self._get_driving_waypoint(current)
                if waypoint:
                    self.ambulance.set_transform(waypoint.transform)
            except Exception as e:
                if DEBUG_PRINT:
                    print(f"[WARN] Failed to snap ambulance to road: {e}")
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to spawn ambulance: {e}")
            return False
    
    def spawn_traffic(self, num_vehicles=TRAFFIC_DENSITY):
        """Spawn NPC vehicles with proximity-based ambulance detection"""
        try:
            world_map = self.world.get_map()
            spawn_points = world_map.get_spawn_points()
            
            if len(spawn_points) < 2:
                print("[ERROR] Not enough spawn points for traffic")
                return 0
            
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bps = blueprint_library.filter("vehicle.car.*")
            
            if not vehicle_bps:
                print("[WARN] No vehicle blueprints found, trying alternative filter")
                vehicle_bps = blueprint_library.filter("vehicle")
            
            if not vehicle_bps:
                print("[ERROR] Could not find any vehicle blueprints")
                return 0
            
            spawned = 0
            for i in range(min(num_vehicles, len(spawn_points) - 1)):
                try:
                    spawn_point = spawn_points[i + 1]
                    vehicle_bp = vehicle_bps[i % len(vehicle_bps)]
                    
                    vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
                    self.vehicles.append(vehicle)
                    
                    # Create behavior controller with proximity detection
                    behavior = BaselineVehicleBehavior(vehicle, world_map, self.world)
                    self.vehicle_behaviors.append(behavior)
                    
                    spawned += 1
                except Exception as e:
                    if DEBUG_PRINT:
                        pass  # Silently skip failed spawns
                    continue
            
            print(f"[SPAWN] Spawned {spawned} NPC vehicles (proximity-based detection)")
            return spawned
            
        except Exception as e:
            print(f"[ERROR] Failed to spawn traffic: {e}")
            traceback.print_exc()
            return 0

    def _update_spectator(self):
        """Move the spectator camera to follow the ambulance."""
        if not CAMERA_FOLLOW_ENABLED or not self.ambulance or not self.world:
            return

        try:
            spectator = self.world.get_spectator()
            transform = self.ambulance.get_transform()
            forward = transform.get_forward_vector()

            location = transform.location - (forward * CAMERA_FOLLOW_DISTANCE)
            location.z += CAMERA_FOLLOW_HEIGHT

            rotation = carla.Rotation(
                pitch=CAMERA_FOLLOW_PITCH,
                yaw=transform.rotation.yaw,
                roll=0.0
            )

            spectator.set_transform(carla.Transform(location, rotation))
        except Exception as e:
            if DEBUG_PRINT:
                print(f"[WARN] Failed to update camera: {e}")
    
    def run_simulation(self, run_number):
        """Run single simulation iteration (baseline without V2X)"""
        print(f"\n{'='*60}")
        print(f"[BASELINE RUN {run_number}] Starting simulation (proximity detection)...")
        print(f"{'='*60}")
        
        try:
            # Clear fire animation state from previous run
            self.animated_buildings.clear()
            self.fire_animation_index = 0
            self.fire_animation_last_update = 0.0
            
            if not self.spawn_ambulance():
                return None
            
            self.spawn_traffic(TRAFFIC_DENSITY)
            
            world_map = self.world.get_map()
            spawn_points = world_map.get_spawn_points()
            
            current_loc = self.ambulance.get_location()
            start_waypoint = self._get_driving_waypoint(current_loc)
            start_loc = start_waypoint.transform.location if start_waypoint else current_loc

            end_loc = self._select_destination(spawn_points, start_loc)

            if end_loc is None:
                print("[ERROR] Could not find a valid destination")
                return None
            
            # Set ambulance destination
            self.ambulance_controller.set_destination(start_loc, end_loc)
            print(f"[ROUTE] Ambulance destination set: {start_loc.distance(end_loc):.1f}m")
            
            # Apply disaster visuals (fire effects on random building - cosmetic only)
            # Note: CARLA doesn't expose building world locations via get_names_of_all_objects(),
            # so fire buildings are selected randomly and may not be exactly at the destination.
            # The ambulance destination remains at the selected spawn point (valid road location).
            self._apply_disaster_visuals(end_loc)
            start_time = time.time()
            frame = 0  # Initialize frame counter
            
            print(f"[DEBUG] Starting simulation loop, max frames: {int(SIMULATION_DURATION / 0.05)}")
            
            try:
                while frame < int(SIMULATION_DURATION / 0.05):
                    self._safe_tick()
                    
                    timestamp = frame * 0.05
                    
                    # Update fire animation if active
                    self._update_fire_animation(timestamp)
                    
                    # Update ambulance
                    try:
                        ambulance_moving = self.ambulance_controller.update(timestamp)
                    except Exception as e:
                        print(f"[ERROR] Ambulance controller update failed at frame {frame}: {e}")
                        import traceback
                        traceback.print_exc()
                        raise
                    
                    # Update all NPC vehicles with proximity-based detection
                    ambulance_location = self.ambulance.get_location() if self.ambulance else None
                    for behavior in self.vehicle_behaviors:
                        try:
                            behavior.detect_ambulance_proximity(ambulance_location)
                            behavior.update(timestamp)
                        except Exception as e:
                            if DEBUG_PRINT:
                                print(f"[WARN] Vehicle behavior update failed: {e}")

                    # Update spectator camera to follow ambulance
                    self._update_spectator()
                    
                    if not ambulance_moving:
                        break
                    
                    if frame % 100 == 0 and DEBUG_PRINT:
                        trip_info = self.ambulance_controller.get_trip_info()
                        if trip_info['duration']:
                            print(f"[FRAME {frame}] Ambulance progress - Time: {trip_info['duration']:.1f}s, "
                                  f"Distance: {trip_info['distance']:.1f}m")
                    
                    frame += 1
                    
            except KeyboardInterrupt:
                print(f"\n[INFO] Simulation interrupted at frame {frame}")
                raise
            except Exception as e:
                print(f"\n[FATAL] Simulation crashed at frame {frame}: {e}")
                import traceback
                traceback.print_exc()
                raise
            
            trip_info = self.ambulance_controller.get_trip_info()
            elapsed_real_time = time.time() - start_time
            
            # Handle cases where trip didn't complete
            trip_duration = trip_info['duration'] if trip_info['duration'] else SIMULATION_DURATION
            trip_distance = trip_info['distance'] if trip_info['distance'] > 0 else 0
            avg_speed = trip_info['average_speed'] if trip_info['average_speed'] > 0 else 0
            
            result = {
                'run_number': run_number,
                'trip_duration': trip_duration,
                'trip_distance': trip_distance,
                'average_speed': avg_speed,
                'vehicles_spawned': len(self.vehicles),
                'real_time_elapsed': elapsed_real_time,
                'trip_complete': trip_info['complete'],
                'timestamp': datetime.now().isoformat()
            }
            
            if trip_info['complete']:
                print(f"[BASELINE RUN {run_number}] Complete - Duration: {trip_duration:.2f}s, "
                      f"Distance: {trip_distance:.1f}m")
            else:
                print(f"[BASELINE RUN {run_number}] Timeout - Duration: {trip_duration:.2f}s, "
                      f"Distance: {trip_distance:.1f}m (incomplete)")
            
            return result
            
        except Exception as e:
            print(f"[ERROR] Simulation failed: {e}")
            traceback.print_exc()
            return None
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up actors and reset world"""
        try:
            print("[CLEANUP] Destroying actors...")

            if self.world and self.original_weather is not None:
                try:
                    self.world.set_weather(self.original_weather)
                except Exception:
                    pass
            
            # Cleanup NPC behaviors (destroys lidar sensors)
            for behavior in self.vehicle_behaviors:
                try:
                    behavior.cleanup()
                except Exception:
                    pass

            # Destroy ambulance first
            if self.ambulance:
                try:
                    self.ambulance.destroy()
                except:
                    pass
            
            # Destroy all traffic vehicles
            for vehicle in self.vehicles:
                try:
                    vehicle.destroy()
                except:
                    pass
            
            self.vehicles.clear()
            self.vehicle_behaviors.clear()
            
            # Cleanup ambulance controller (destroys lidar sensor)
            if self.ambulance_controller:
                self.ambulance_controller.cleanup()
            
            self.ambulance = None
            self.ambulance_controller = None
            self.animated_buildings.clear()
            self.fire_animation_index = 0
            self.fire_animation_last_update = 0.0
            
            # Give CARLA time to process destructions
            print("[CLEANUP] Waiting for cleanup to settle...")
            for _ in range(10):
                self._safe_tick()
            
            print("[CLEANUP] Complete")
            
        except Exception as e:
            print(f"[WARN] Cleanup error: {e}")
    
    def run_multiple_simulations(self, num_runs=NUM_RUNS):
        """Run multiple baseline simulations"""
        print(f"\n{'='*60}")
        print(f"BASELINE AMBULANCE SIMULATION (Proximity-Based Detection)")
        print(f"Configuration:")
        print(f"  - Runs: {num_runs}")
        print(f"  - Traffic Density: {TRAFFIC_DENSITY} vehicles")
        print(f"  - Detection: Proximity (60m range, simulating sirens)")
        print(f"  - V2X Communication: NO")
        print(f"  - Map: {MAP_NAME}")
        print(f"{'='*60}\n")
        
        self.run_results = []
        
        for run in range(1, num_runs + 1):
            result = self.run_simulation(run)
            if result:
                self.run_results.append(result)
            
            # Add delay between runs to let CARLA stabilize
            if run < num_runs:
                print(f"\n[WAIT] Pausing 3 seconds before next run...\n")
                time.sleep(3)
        
        self.print_summary()
        
        if RECORD_RESULTS:
            self.save_results()
    
    def print_summary(self):
        """Print summary statistics"""
        if not self.run_results:
            print("[ERROR] No results to summarize")
            return
        
        durations = [r['trip_duration'] for r in self.run_results if r['trip_duration']]
        distances = [r['trip_distance'] for r in self.run_results if r['trip_distance']]
        speeds = [r['average_speed'] for r in self.run_results if r['average_speed']]
        
        if durations:
            avg_duration = sum(durations) / len(durations)
            min_duration = min(durations)
            max_duration = max(durations)
            
            print(f"\n{'='*60}")
            print(f"BASELINE SIMULATION SUMMARY")
            print(f"{'='*60}")
            print(f"Trip Duration (proximity-based path clearing):")
            print(f"  Average: {avg_duration:.2f} seconds")
            print(f"  Min: {min_duration:.2f} seconds")
            print(f"  Max: {max_duration:.2f} seconds")
            print(f"  Std Dev: {self._std_dev(durations):.2f} seconds")
            
            if distances:
                avg_distance = sum(distances) / len(distances)
                print(f"\nAverage Trip Distance: {avg_distance:.1f} meters")
            
            if speeds:
                avg_speed = sum(speeds) / len(speeds)
                print(f"Average Trip Speed: {avg_speed:.2f} m/s")
            
            print(f"Runs Completed: {len(durations)}/{len(self.run_results)}")
            print(f"{'='*60}\n")
    
    def _std_dev(self, values):
        """Calculate standard deviation"""
        if not values or len(values) < 2:
            return 0
        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        return variance ** 0.5
    
    def save_results(self):
        """Save results to JSON file"""
        try:
            if not os.path.exists(RESULTS_DIR):
                os.makedirs(RESULTS_DIR)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(RESULTS_DIR, f"baseline_simulation_{timestamp}.json")
            
            summary = {
                'configuration': {
                    'map': MAP_NAME,
                    'traffic_density': TRAFFIC_DENSITY,
                    'v2x_enabled': False,
                    'v2x_range': 0,
                    'num_runs': NUM_RUNS,
                    'timestamp': datetime.now().isoformat()
                },
                'results': self.run_results
            }
            
            with open(filename, 'w') as f:
                json.dump(summary, f, indent=2, default=str)
            
            print(f"[SAVE] Baseline results saved to {filename}")
            
        except Exception as e:
            print(f"[ERROR] Failed to save results: {e}")


def main():
    """Main entry point"""
    simulation = BaselineAmbulanceSimulation()
    
    try:
        simulation.setup()
        simulation.run_multiple_simulations(NUM_RUNS)
        
    except KeyboardInterrupt:
        print("\n[INFO] Simulation interrupted by user")
    except Exception as e:
        print(f"[FATAL] {e}")
        traceback.print_exc()
    finally:
        simulation.cleanup()
        print("[INFO] Simulation ended")


if __name__ == "__main__":
    main()
