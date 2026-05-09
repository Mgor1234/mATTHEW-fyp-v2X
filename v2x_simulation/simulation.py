"""
Main V2X Ambulance Simulation Runner
Simulates vehicles clearing path for emergency ambulance using V2X communication
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'PythonAPI', 'carla'))

import carla
import time
import traceback
from datetime import datetime
import csv
import gc
from ambulance_controller import AmbulanceController
from vehicle_behavior import V2XVehicleBehavior
from minimap import RealTimeMinimapWindow, MinimapUpdateQueue, get_map_road_geometry, get_vehicle_world_bounds
from config import *
import json

# Calculate tick duration from target FPS
SIMULATION_TICK_DURATION = 1.0 / SIMULATION_FPS


# When True, a valid ambulance start index is treated as mandatory.
# If spawning at that exact index fails, simulation run aborts instead of silently
# falling back to another spawn point.
STRICT_AMBULANCE_START_WAYPOINT = False
SELECTED_AMBULANCE_START_LOCATION = None
SELECTED_AMBULANCE_END_LOCATION = None


class V2XAmbulanceSimulation:
    def __init__(self):
        self.client = None
        self.world = None
        self.traffic_manager = None
        self.vehicles = []
        self.ambulance = None
        self.ambulance_controller = None
        self.vehicle_behaviors = {}
        self.vehicle_types = {}  # Track vehicle type: {vehicle_id: 'A'/'B'/'C'}
        self.run_results = []
        self.original_weather = None
        self.spawned_type_counts = {"A": 0, "B": 0, "C": 0}
        self.total_traffic_spawned = 0
        self.ambulance_spawn_index = None
        # Fire animation state
        self.animated_buildings = []
        self.fire_animation_index = 0
        self.fire_animation_last_update = 0.0
        # Minimap state
        self.minimap_window = None
        self.minimap_update_queue = None
        self.minimap_enabled = MINIMAP_ENABLED
        self.minimap_update_every_n_frames = max(1, int(globals().get('MINIMAP_UPDATE_EVERY_N_FRAMES', 2)))
        self.npc_behavior_update_every_n_frames = max(1, int(globals().get('NPC_BEHAVIOR_UPDATE_EVERY_N_FRAMES', 1)))
        self.spectator_update_every_n_frames = max(1, int(globals().get('SPECTATOR_UPDATE_EVERY_N_FRAMES', 1)))
        self._minimap_update_counter = 0

    def _runtime_disaster_mode(self):
        """Return runtime disaster mode currently configured for this process."""
        return str(DISASTER_VISUAL_MODE).lower()

    def _runtime_map_name(self):
        return str(MAP_NAME)

    def _runtime_type_counts(self):
        counts = {
            "A": max(0, int(TYPE_A_COUNT)),
            "B": max(0, int(TYPE_B_COUNT)),
            "C": max(0, int(TYPE_C_COUNT)),
        }
        if sum(counts.values()) == 0:
            # Backward-compatible fallback if type counts are all zero.
            counts["C"] = max(0, int(TRAFFIC_DENSITY))
        return counts

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

    def _try_spawn_vehicle(self, blueprint, spawn_points, label="vehicle", snap_to_lane=True):
        """Attempt to spawn a vehicle at multiple spawn points."""
        for transform in spawn_points:
            target = self._snap_transform_to_driving_lane(transform) if snap_to_lane else transform
            actor = self.world.try_spawn_actor(blueprint, target)
            if actor is None:
                adjusted = carla.Transform(
                    carla.Location(
                        x=target.location.x,
                        y=target.location.y,
                        z=target.location.z + 0.5
                    ),
                    target.rotation
                )
                actor = self.world.try_spawn_actor(blueprint, adjusted)
            if actor is None:
                adjusted = carla.Transform(
                    carla.Location(
                        x=target.location.x,
                        y=target.location.y,
                        z=target.location.z + 1.0
                    ),
                    target.rotation
                )
                actor = self.world.try_spawn_actor(blueprint, adjusted)
            if actor is not None:
                if DEBUG_PRINT:
                    print(f"[SPAWN] {label} spawned at {actor.get_location()}")
                return actor
        return None

    def _pick_random_destination_location(self, spawn_points, origin_location=None, min_distance=25.0):
        """Choose a random destination location with a minimum distance from origin."""
        if not spawn_points:
            return None

        import random
        candidates = list(spawn_points)
        random.shuffle(candidates)

        if origin_location is None:
            return candidates[0].location

        for candidate in candidates:
            if candidate.location.distance(origin_location) >= float(min_distance):
                return candidate.location

        return candidates[0].location

    def _assign_type_a_destination(self, vehicle, destination_location):
        """Assign a destination path to Type A vehicle via Traffic Manager."""
        if destination_location is None:
            return False

        try:
            self.traffic_manager.set_path(vehicle, [destination_location])
            return True
        except Exception:
            return False

    def _select_destination(self, spawn_points, start_location):
        """Pick destination strictly from user selection (location first, then index)."""
        if not spawn_points:
            return None

        # UI-selected end location is authoritative when provided.
        if SELECTED_AMBULANCE_END_LOCATION is not None:
            try:
                return carla.Location(
                    x=float(SELECTED_AMBULANCE_END_LOCATION["x"]),
                    y=float(SELECTED_AMBULANCE_END_LOCATION["y"]),
                    z=float(SELECTED_AMBULANCE_END_LOCATION["z"]),
                )
            except Exception:
                pass

        # UI-selected end waypoint index is authoritative when valid.
        if 0 <= AMBULANCE_END_WAYPOINT < len(spawn_points):
            candidate = spawn_points[AMBULANCE_END_WAYPOINT]
            waypoint = self._get_driving_waypoint(candidate.location)
            if waypoint:
                return waypoint.transform.location

        # CLI/default fallback: support AMBULANCE_END_WAYPOINT == -1 (random destination).
        if int(AMBULANCE_END_WAYPOINT) < 0:
            random_dest = self._pick_random_destination_location(
                spawn_points,
                origin_location=start_location,
                min_distance=80.0,
            )
            if random_dest is not None:
                waypoint = self._get_driving_waypoint(random_dest)
                if waypoint:
                    return waypoint.transform.location

        return None

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
        mode = self._runtime_disaster_mode()

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
    
    def _setup_minimap(self):
        """Initialize real-time minimap display"""
        try:
            if not self.minimap_enabled:
                print("[MINIMAP] Minimap disabled")
                return
            
            # Get config values with defaults
            minimap_width = getattr(sys.modules[__name__], 'MINIMAP_WIDTH', 400)
            minimap_height = getattr(sys.modules[__name__], 'MINIMAP_HEIGHT', 400)
            minimap_fps = getattr(sys.modules[__name__], 'MINIMAP_FPS', 30)
            
            self.minimap_update_queue = MinimapUpdateQueue()
            self.minimap_window = RealTimeMinimapWindow(
                self._runtime_map_name(),
                self.minimap_update_queue,
                width=minimap_width,
                height=minimap_height,
                fps=minimap_fps
            )
            
            # Cache the road network once so the minimap can render the map
            # with a stable scale and road geometry instead of just vehicle dots.
            road_segments, bounds = get_map_road_geometry(self.world)
            min_x, max_x, min_y, max_y = bounds
            self.minimap_window.set_road_geometry(road_segments)

            # Set world bounds for minimap
            self.minimap_window.set_world_bounds(min_x, max_x, min_y, max_y)
            
            # Start minimap window thread
            self.minimap_window.start()
            print(f"[MINIMAP] Real-time minimap started ({minimap_width}x{minimap_height}, {minimap_fps} FPS)")
        except Exception as e:
            print(f"[MINIMAP ERROR] Failed to initialize minimap: {e}")
            import traceback
            traceback.print_exc()
            self.minimap_enabled = False
    
    def _update_minimap(self):
        """Update minimap with current vehicle positions"""
        if not self.minimap_enabled or not self.minimap_update_queue:
            return

        self._minimap_update_counter += 1
        if (self._minimap_update_counter % self.minimap_update_every_n_frames) != 0:
            return
        
        try:
            # Update ambulance
            if self.ambulance and self.ambulance.is_alive:
                transform = self.ambulance.get_transform()
                pos = transform.location
                rot = transform.rotation.yaw
                self.minimap_update_queue.add_update(
                    self.ambulance.id, (pos.x, pos.y), 'ambulance', rot
                )
            elif self.ambulance and self.minimap_window:
                self.minimap_window.remove_vehicle(self.ambulance.id)
            
            # Update regular traffic vehicles
            for vehicle in self.vehicles:
                try:
                    if vehicle.is_alive:
                        vehicle_type = self.vehicle_types.get(vehicle.id, 'A')
                        transform = vehicle.get_transform()
                        pos = transform.location
                        rot = transform.rotation.yaw
                        self.minimap_update_queue.add_update(
                            vehicle.id, (pos.x, pos.y), vehicle_type, rot
                        )
                    elif self.minimap_window:
                        self.minimap_window.remove_vehicle(vehicle.id)
                except:
                    pass
        except Exception as e:
            if DEBUG_PRINT:
                print(f"[MINIMAP ERROR] Update failed: {e}")
    
    def _cleanup_minimap(self):
        """Stop minimap thread and cleanup"""
        if self.minimap_window:
            self.minimap_window.stop()
            self.minimap_window.join(timeout=2.0)
            self.minimap_window = None
            print("[MINIMAP] Minimap window closed")
        self.minimap_update_queue = None
        self._minimap_update_counter = 0
        
    def setup(self):
        """Connect to CARLA and setup world"""
        try:
            self.client = carla.Client(CARLA_HOST, CARLA_PORT)
            self.client.set_timeout(CARLA_TIMEOUT)
            
            print(f"[SETUP] Connected to CARLA server at {CARLA_HOST}:{CARLA_PORT}")
            
            # Load map
            self.world = self.client.load_world(self._runtime_map_name())
            self.traffic_manager = self.client.get_trafficmanager()
            self.traffic_manager.set_global_distance_to_leading_vehicle(5.0)
            self.original_weather = self.world.get_weather()
            
            print(f"[SETUP] Loaded map: {self._runtime_map_name()}")

            self._clear_existing_actors()
            if DISABLE_TRAFFIC_LIGHTS:
                self._disable_all_traffic_lights()
            
            # Set synchronous mode
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = SIMULATION_TICK_DURATION
            self.world.apply_settings(settings)
            
            print(f"[SETUP] Synchronous mode enabled ({SIMULATION_FPS} FPS, {SIMULATION_TICK_DURATION*1000:.2f}ms tick)")

            if DEBUG_PRINT:
                spawn_points = self.world.get_map().get_spawn_points()
                print(f"[SETUP] Spawn points available: {len(spawn_points)}")

            # Give the world time to settle after loading
            for _ in range(5):
                self._safe_tick()
            
            # Initialize minimap if enabled
            self._setup_minimap()
            
        except Exception as e:
            print(f"[ERROR] Setup failed: {e}")
            traceback.print_exc()
            raise
    
    def spawn_ambulance(self):
        """Spawn ambulance vehicle"""
        try:
            world_map = self.world.get_map()
            spawn_points = world_map.get_spawn_points()
            self.ambulance_spawn_index = None
            
            if not spawn_points:
                print("[ERROR] No spawn points available")
                return False
            
            ambulance_bp = self._select_ambulance_blueprint()
            if ambulance_bp is None:
                print("[ERROR] No vehicle blueprints available for ambulance")
                return False
            selected_start = int(AMBULANCE_START_WAYPOINT)
            selected_start_location = None

            if SELECTED_AMBULANCE_START_LOCATION is not None:
                try:
                    selected_start_location = carla.Location(
                        x=float(SELECTED_AMBULANCE_START_LOCATION["x"]),
                        y=float(SELECTED_AMBULANCE_START_LOCATION["y"]),
                        z=float(SELECTED_AMBULANCE_START_LOCATION["z"]),
                    )
                except Exception:
                    selected_start_location = None

            # UI-selected start location is authoritative when provided.
            if selected_start_location is not None:
                base_rotation = carla.Rotation()
                if 0 <= selected_start < len(spawn_points):
                    base_rotation = spawn_points[selected_start].rotation
                else:
                    waypoint = self._get_driving_waypoint(selected_start_location)
                    if waypoint:
                        base_rotation = waypoint.transform.rotation

                requested_transform = carla.Transform(selected_start_location, base_rotation)
                self.ambulance = self._try_spawn_vehicle(
                    ambulance_bp,
                    [requested_transform],
                    label=f"Ambulance (UI selected location index {selected_start})",
                    snap_to_lane=False,
                )
                if self.ambulance is None:
                    print(
                        "[ERROR] Ambulance could not spawn at the exact UI-selected start point. "
                        "Clear nearby blocking actors or choose another start point."
                    )
                    return False
            else:
                # If UI location was not provided, require a valid selected start index.
                if not (0 <= selected_start < len(spawn_points)):
                    print("[ERROR] Invalid ambulance start selection. Choose a valid start point in UI.")
                    return False

                selected_transform = spawn_points[selected_start]
                self.ambulance = self._try_spawn_vehicle(
                    ambulance_bp,
                    [selected_transform],
                    label=f"Ambulance (UI selected index {selected_start})",
                    snap_to_lane=False,
                )

                if self.ambulance is None:
                    print(
                        f"[ERROR] Ambulance could not spawn at selected start index {selected_start}. "
                        "Clear nearby blocking actors or choose another start point."
                    )
                    return False

            if self.ambulance is None:
                print("[ERROR] Failed to spawn ambulance at any spawn point")
                return False

            # Hard-enforce the UI-selected start transform after actor creation.
            # This guarantees the start point is exactly what the UI selected.
            if selected_start_location is not None:
                try:
                    enforced_rotation = self.ambulance.get_transform().rotation
                    if 0 <= selected_start < len(spawn_points):
                        enforced_rotation = spawn_points[selected_start].rotation
                    enforced_transform = carla.Transform(selected_start_location, enforced_rotation)
                    self.ambulance.set_transform(enforced_transform)
                    # Tick the world to apply the transform
                    self.world.tick()
                except Exception as e:
                    print(f"[ERROR] Failed to enforce UI-selected ambulance start point: {e}")
                    import traceback
                    traceback.print_exc()
                    return False

            self.ambulance_controller = AmbulanceController(self.ambulance, world_map, self.world)

            # Track the nearest spawn-point index to where the ambulance actually spawned.
            try:
                spawn_loc = self.ambulance.get_location()
                nearest_idx = min(
                    range(len(spawn_points)),
                    key=lambda idx: spawn_points[idx].location.distance(spawn_loc),
                )
                self.ambulance_spawn_index = int(nearest_idx)
                print(
                    f"[SPAWN] Ambulance requested index={selected_start}, "
                    f"actual nearest index={self.ambulance_spawn_index}, "
                    f"strict_start={'ON' if STRICT_AMBULANCE_START_WAYPOINT else 'OFF'}, "
                    f"requested_loc={'SET' if selected_start_location else 'NONE'}, "
                    f"actual_loc=({spawn_loc.x:.2f}, {spawn_loc.y:.2f}, {spawn_loc.z:.2f})"
                )
            except Exception:
                self.ambulance_spawn_index = None

            # Avoid overriding a user-selected start location by post-spawn snapping.
            if selected_start_location is None:
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
    
    def spawn_traffic(self):
        """Spawn mixed Type A/B/C NPC traffic."""
        try:
            world_map = self.world.get_map()
            spawn_points = world_map.get_spawn_points()
            
            if len(spawn_points) < 2:
                print("[ERROR] Not enough spawn points for traffic")
                return 0
            
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bps = blueprint_library.filter("vehicle.car.*")
            
            if not vehicle_bps:
                vehicle_bps = blueprint_library.filter("vehicle")
            
            self.spawned_type_counts = {"A": 0, "B": 0, "C": 0}

            traffic_mix = self._runtime_type_counts()
            vehicle_plan = (["A"] * traffic_mix["A"]) + (["B"] * traffic_mix["B"]) + (["C"] * traffic_mix["C"])
            target_spawn = min(len(vehicle_plan), len(spawn_points) - 1)

            ambulance_location = self.ambulance.get_location() if self.ambulance else None
            exclusion_radius = max(0.0, float(globals().get('TRAFFIC_SPAWN_EXCLUSION_RADIUS', 0.0)))

            spawn_candidates = [
                (idx, sp) for idx, sp in enumerate(spawn_points)
                if idx != self.ambulance_spawn_index
                and (
                    ambulance_location is None
                    or exclusion_radius <= 0.0
                    or sp.location.distance(ambulance_location) >= exclusion_radius
                )
            ]

            if ambulance_location is not None and exclusion_radius > 0.0:
                print(
                    f"[SPAWN] Traffic exclusion radius around ambulance: {exclusion_radius:.1f}m "
                    f"(candidates left: {len(spawn_candidates)})"
                )

            spawned = 0
            for i in range(min(target_spawn, len(spawn_candidates))):
                try:
                    spawn_point = spawn_candidates[i][1]
                    vehicle_bp = vehicle_bps[i % len(vehicle_bps)]
                    
                    vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)
                    if vehicle is None:
                        continue
                    self.vehicles.append(vehicle)

                    vehicle_type = vehicle_plan[i]
                    self.vehicle_types[vehicle.id] = vehicle_type
                    destination_location = self._pick_random_destination_location(
                        spawn_points,
                        origin_location=spawn_point.location,
                        min_distance=25.0,
                    )

                    # Keep TM light handling enabled for all traffic vehicles.
                    self.traffic_manager.update_vehicle_lights(vehicle, True)

                    if vehicle_type == "A":
                        # Type A: pure CARLA Traffic Manager behavior.
                        try:
                            self.traffic_manager.auto_lane_change(vehicle, False)
                            self.traffic_manager.random_left_lanechange_percentage(vehicle, 0.0)
                            self.traffic_manager.random_right_lanechange_percentage(vehicle, 0.0)
                        except Exception:
                            pass
                        vehicle.set_autopilot(True, self.traffic_manager.get_port())
                        self._assign_type_a_destination(vehicle, destination_location)
                    elif vehicle_type == "B":
                        # Type B: drive to destination with rear lidar detection for left-lane evasion only.
                        behavior = V2XVehicleBehavior(
                            vehicle,
                            world_map,
                            self.world,
                            use_v2x=False,
                            use_lidar=True,
                            destination_location=destination_location,
                        )
                        self.vehicle_behaviors[vehicle.id] = behavior
                        vehicle.set_autopilot(False, self.traffic_manager.get_port())
                    else:
                        # Type C: autonomous + V2X only (no lidar).
                        behavior = V2XVehicleBehavior(
                            vehicle,
                            world_map,
                            self.world,
                            use_v2x=True,
                            use_lidar=False,
                            destination_location=destination_location,
                        )
                        self.vehicle_behaviors[vehicle.id] = behavior
                        vehicle.set_autopilot(False, self.traffic_manager.get_port())

                    self.spawned_type_counts[vehicle_type] += 1
                    
                    spawned += 1
                except Exception as e:
                    if DEBUG_PRINT:
                        print(f"[WARN] Failed to spawn vehicle {i}: {e}")
                    continue

            self.total_traffic_spawned = spawned
            print(
                f"[SPAWN] Spawned {spawned} NPC vehicles "
                f"(Type A={self.spawned_type_counts['A']}, "
                f"Type B={self.spawned_type_counts['B']}, "
                f"Type C={self.spawned_type_counts['C']})"
            )
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
        """Run single simulation iteration"""
        print(f"\n{'='*60}")
        print(f"[RUN {run_number}] Starting simulation...")
        print(f"{'='*60}")
        
        try:
            # Clear fire animation state from previous run
            self.animated_buildings.clear()
            self.fire_animation_index = 0
            self.fire_animation_last_update = 0.0
            
            # Spawn vehicles
            if not self.spawn_ambulance():
                return None
            
            self.spawn_traffic()
            
            # Set ambulance destination
            world_map = self.world.get_map()
            spawn_points = world_map.get_spawn_points()
            
            current_loc = self.ambulance.get_location()
            start_waypoint = self._get_driving_waypoint(current_loc)
            start_loc = start_waypoint.transform.location if start_waypoint else current_loc

            end_loc = self._select_destination(spawn_points, start_loc)

            if end_loc is None:
                print("[ERROR] Could not find a valid destination")
                return None
            
            # Apply disaster visuals (fire effects on random building - cosmetic only)
            # Note: CARLA doesn't expose building world locations via get_names_of_all_objects(),
            # so fire buildings are selected randomly and may not be exactly at the destination.
            # The ambulance destination remains at the selected spawn point (valid road location).
            self._apply_disaster_visuals(end_loc)
            
            if not self.ambulance_controller.set_destination(start_loc, end_loc):
                print("[ERROR] Failed to compute or set ambulance shortest road route")
                return None
            if self.minimap_window:
                self.minimap_window.set_route_path(self.ambulance_controller.get_route_path())
            
            # Simulation loop
            frame = 0
            start_time = time.time()
            
            while frame < int(SIMULATION_DURATION / SIMULATION_TICK_DURATION):
                # Tick world
                self._safe_tick()
                
                # Update fire animation if active
                elapsed_time = frame * SIMULATION_TICK_DURATION
                self._update_fire_animation(elapsed_time)
                
                # Broadcast ambulance V2X signal
                ambulance_broadcast = self.ambulance_controller.get_v2x_broadcast()
                
                # Update all vehicles
                timestamp = frame * SIMULATION_TICK_DURATION
                
                # Filter out destroyed vehicles
                destroyed_ids = []
                for vehicle_id, behavior in self.vehicle_behaviors.items():
                    try:
                        # Check if vehicle still exists
                        if behavior.vehicle.is_alive:
                            should_update_behavior = (
                                self.npc_behavior_update_every_n_frames <= 1
                                or (vehicle_id % self.npc_behavior_update_every_n_frames)
                                == (frame % self.npc_behavior_update_every_n_frames)
                            )
                            if should_update_behavior:
                                # Update V2X detection and behavior on a staggered frame schedule.
                                behavior.detect_v2x_ambulance(ambulance_broadcast, timestamp)
                                behavior.update(timestamp)
                        else:
                            destroyed_ids.append(vehicle_id)
                    except Exception as e:
                        # Vehicle was destroyed, mark for removal
                        destroyed_ids.append(vehicle_id)
                
                # Remove destroyed vehicles from tracking
                for vehicle_id in destroyed_ids:
                    self.vehicle_behaviors.pop(vehicle_id, None)
                
                # Update ambulance
                ambulance_moving = self.ambulance_controller.update(timestamp)

                # Update spectator camera to follow ambulance
                if (frame % self.spectator_update_every_n_frames) == 0:
                    self._update_spectator()
                
                # Update minimap with current positions
                self._update_minimap()
                
                # Check if trip complete
                if not ambulance_moving:
                    break
                
                if frame % 100 == 0 and DEBUG_PRINT:
                    trip_info = self.ambulance_controller.get_trip_info()
                    if trip_info['duration']:
                        print(f"[FRAME {frame}] Ambulance progress - Time: {trip_info['duration']:.1f}s, "
                              f"Distance: {trip_info['distance']:.1f}m")
                
                frame += 1
            
            # Get results
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
                'vehicles_spawned': self.total_traffic_spawned,
                'type_a_spawned': self.spawned_type_counts['A'],
                'type_b_spawned': self.spawned_type_counts['B'],
                'type_c_spawned': self.spawned_type_counts['C'],
                'real_time_elapsed': elapsed_real_time,
                'trip_complete': trip_info['complete'],
                'timestamp': datetime.now().isoformat()
            }
            
            if trip_info['complete']:
                print(f"[RUN {run_number}] Complete - Duration: {trip_duration:.2f}s, "
                      f"Distance: {trip_distance:.1f}m")
            else:
                print(f"[RUN {run_number}] Timeout - Duration: {trip_duration:.2f}s, "
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

            # Cleanup minimap
            self._cleanup_minimap()

            if self.world and self.original_weather is not None:
                try:
                    self.world.set_weather(self.original_weather)
                except Exception:
                    pass
            
            # Cleanup NPC behaviors (destroys lidar sensors)
            for behavior in self.vehicle_behaviors.values():
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
            
            # Cleanup ambulance controller (destroys lidar sensor)
            if self.ambulance_controller:
                self.ambulance_controller.cleanup()
                self.ambulance_controller.route_path = []
            
            self.ambulance = None
            self.ambulance_controller = None
            self.vehicle_behaviors.clear()
            self.minimap_update_queue = None
            self.animated_buildings.clear()
            self.fire_animation_index = 0
            self.fire_animation_last_update = 0.0
            
            # Give CARLA time to process destructions
            print("[CLEANUP] Waiting for cleanup to settle...")
            for _ in range(10):
                self._safe_tick()
            
            # Explicitly request CARLA to free memory
            print("[CLEANUP] Unloading map and freeing memory...")
            try:
                if self.world:
                    self.world = None
                if self.traffic_manager:
                    self.traffic_manager = None
            except Exception:
                pass
            
            # Garbage collect Python objects
            print("[CLEANUP] Running garbage collection...")
            gc.collect()
            
            print("[CLEANUP] Complete")
            
        except Exception as e:
            print(f"[WARN] Cleanup error: {e}")
    
    def run_multiple_simulations(self, num_runs=NUM_RUNS):
        """Run multiple simulations and collect statistics"""
        print(f"\n{'='*60}")
        print(f"V2X AMBULANCE SIMULATION")
        print(f"Configuration:")
        print(f"  - Runs: {num_runs}")
        mix = self._runtime_type_counts()
        print(f"  - Traffic Density: {sum(mix.values())} vehicles")
        print(f"  - Type A/B/C: {mix['A']}/{mix['B']}/{mix['C']}")
        print(f"  - V2X Range: {V2X_DETECTION_RANGE}m")
        print(f"  - Map: {self._runtime_map_name()}")
        print(f"  - Storm Mode: {'ON' if self._runtime_disaster_mode() == 'storm' else 'OFF'}")
        print(f"  - Ambulance Start Waypoint: {AMBULANCE_START_WAYPOINT}")
        print(f"  - Ambulance End Waypoint: {AMBULANCE_END_WAYPOINT}")
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
        
        # Print summary
        self.print_summary()
        
        # Save results
        if RECORD_RESULTS:
            self.save_results()
            self.save_result_sheet()
    
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
            print(f"SIMULATION SUMMARY")
            print(f"{'='*60}")
            print(f"Trip Duration (with V2X path clearing):")
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
            filename = os.path.join(RESULTS_DIR, f"v2x_simulation_{timestamp}.json")
            
            summary = {
                'configuration': {
                    'map': self._runtime_map_name(),
                    'traffic_density': sum(self._runtime_type_counts().values()),
                    'type_a_count': self._runtime_type_counts()['A'],
                    'type_b_count': self._runtime_type_counts()['B'],
                    'type_c_count': self._runtime_type_counts()['C'],
                    'v2x_range': V2X_DETECTION_RANGE,
                    'num_runs': NUM_RUNS,
                    'storm_enabled': self._runtime_disaster_mode() == 'storm',
                    'ambulance_start_waypoint': AMBULANCE_START_WAYPOINT,
                    'ambulance_end_waypoint': AMBULANCE_END_WAYPOINT,
                    'timestamp': datetime.now().isoformat()
                },
                'results': self.run_results
            }
            
            with open(filename, 'w') as f:
                json.dump(summary, f, indent=2, default=str)
            
            print(f"[SAVE] Results saved to {filename}")
            
        except Exception as e:
            print(f"[ERROR] Failed to save results: {e}")

    def save_result_sheet(self):
        """Save a human-friendly CSV result sheet for each simulation batch."""
        try:
            if not self.run_results:
                return

            if not os.path.exists(RESULTS_DIR):
                os.makedirs(RESULTS_DIR)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(RESULTS_DIR, f"v2x_result_sheet_{timestamp}.csv")

            durations = [r['trip_duration'] for r in self.run_results if r['trip_duration']]
            distances = [r['trip_distance'] for r in self.run_results if r['trip_distance']]
            speeds = [r['average_speed'] for r in self.run_results if r['average_speed']]

            with open(filename, 'w', newline='') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(["Section", "Field", "Value"])
                writer.writerow(["Configuration", "Map", self._runtime_map_name()])
                writer.writerow(["Configuration", "Traffic Total", sum(self._runtime_type_counts().values())])
                writer.writerow(["Configuration", "Type A Count", self._runtime_type_counts()['A']])
                writer.writerow(["Configuration", "Type B Count", self._runtime_type_counts()['B']])
                writer.writerow(["Configuration", "Type C Count", self._runtime_type_counts()['C']])
                writer.writerow(["Configuration", "V2X Range (m)", V2X_DETECTION_RANGE])
                writer.writerow(["Configuration", "Storm Enabled", self._runtime_disaster_mode() == 'storm'])
                writer.writerow(["Configuration", "Ambulance Start Waypoint", AMBULANCE_START_WAYPOINT])
                writer.writerow(["Configuration", "Ambulance End Waypoint", AMBULANCE_END_WAYPOINT])
                writer.writerow([])

                if durations:
                    writer.writerow(["Summary", "Average Trip Duration (s)", f"{sum(durations)/len(durations):.3f}"])
                    writer.writerow(["Summary", "Min Trip Duration (s)", f"{min(durations):.3f}"])
                    writer.writerow(["Summary", "Max Trip Duration (s)", f"{max(durations):.3f}"])
                    writer.writerow(["Summary", "Std Dev Trip Duration (s)", f"{self._std_dev(durations):.3f}"])
                if distances:
                    writer.writerow(["Summary", "Average Trip Distance (m)", f"{sum(distances)/len(distances):.3f}"])
                if speeds:
                    writer.writerow(["Summary", "Average Speed (m/s)", f"{sum(speeds)/len(speeds):.3f}"])

                writer.writerow([])
                writer.writerow([
                    "Runs",
                    "Run Number",
                    "Trip Duration (s)",
                    "Trip Distance (m)",
                    "Average Speed (m/s)",
                    "Type A Spawned",
                    "Type B Spawned",
                    "Type C Spawned",
                    "Trip Complete"
                ])
                for run in self.run_results:
                    writer.writerow([
                        "Runs",
                        run.get('run_number'),
                        f"{run.get('trip_duration', 0):.3f}",
                        f"{run.get('trip_distance', 0):.3f}",
                        f"{run.get('average_speed', 0):.3f}",
                        run.get('type_a_spawned', 0),
                        run.get('type_b_spawned', 0),
                        run.get('type_c_spawned', 0),
                        run.get('trip_complete', False)
                    ])

            print(f"[SAVE] Result sheet saved to {filename}")
        except Exception as e:
            print(f"[ERROR] Failed to save result sheet: {e}")


def apply_runtime_overrides(overrides):
    """Apply runtime parameters (used by UI launcher) without editing config.py."""
    global NUM_RUNS
    global TRAFFIC_DENSITY
    global TYPE_A_COUNT
    global TYPE_B_COUNT
    global TYPE_C_COUNT
    global AMBULANCE_START_WAYPOINT
    global AMBULANCE_END_WAYPOINT
    global MAP_NAME
    global DISASTER_VISUAL_MODE
    global STRICT_AMBULANCE_START_WAYPOINT
    global SELECTED_AMBULANCE_START_LOCATION
    global SELECTED_AMBULANCE_END_LOCATION

    if not overrides:
        return

    NUM_RUNS = int(overrides.get('num_runs', NUM_RUNS))
    TYPE_A_COUNT = int(overrides.get('type_a_count', TYPE_A_COUNT))
    TYPE_B_COUNT = int(overrides.get('type_b_count', TYPE_B_COUNT))
    TYPE_C_COUNT = int(overrides.get('type_c_count', TYPE_C_COUNT))
    TRAFFIC_DENSITY = TYPE_A_COUNT + TYPE_B_COUNT + TYPE_C_COUNT
    AMBULANCE_START_WAYPOINT = int(overrides.get('ambulance_start_waypoint', AMBULANCE_START_WAYPOINT))
    AMBULANCE_END_WAYPOINT = int(overrides.get('ambulance_end_waypoint', AMBULANCE_END_WAYPOINT))
    MAP_NAME = str(overrides.get('map_name', MAP_NAME))
    STRICT_AMBULANCE_START_WAYPOINT = bool(overrides.get('strict_ambulance_start_waypoint', STRICT_AMBULANCE_START_WAYPOINT))
    SELECTED_AMBULANCE_START_LOCATION = overrides.get('ambulance_start_location')
    SELECTED_AMBULANCE_END_LOCATION = overrides.get('ambulance_end_location')

    if bool(overrides.get('storm_enabled', False)):
        DISASTER_VISUAL_MODE = "storm"
    else:
        DISASTER_VISUAL_MODE = "off"

    start_loc_text = "NONE"
    end_loc_text = "NONE"
    if isinstance(SELECTED_AMBULANCE_START_LOCATION, dict):
        try:
            start_loc_text = (
                f"({float(SELECTED_AMBULANCE_START_LOCATION['x']):.2f}, "
                f"{float(SELECTED_AMBULANCE_START_LOCATION['y']):.2f}, "
                f"{float(SELECTED_AMBULANCE_START_LOCATION['z']):.2f})"
            )
        except Exception:
            start_loc_text = "SET"
    elif SELECTED_AMBULANCE_START_LOCATION is not None:
        start_loc_text = "SET"

    if isinstance(SELECTED_AMBULANCE_END_LOCATION, dict):
        try:
            end_loc_text = (
                f"({float(SELECTED_AMBULANCE_END_LOCATION['x']):.2f}, "
                f"{float(SELECTED_AMBULANCE_END_LOCATION['y']):.2f}, "
                f"{float(SELECTED_AMBULANCE_END_LOCATION['z']):.2f})"
            )
        except Exception:
            end_loc_text = "SET"
    elif SELECTED_AMBULANCE_END_LOCATION is not None:
        end_loc_text = "SET"

    print(
        f"[OVERRIDES] map={MAP_NAME}, runs={NUM_RUNS}, "
        f"start={AMBULANCE_START_WAYPOINT}, end={AMBULANCE_END_WAYPOINT}, "
        f"strict_start={'ON' if STRICT_AMBULANCE_START_WAYPOINT else 'OFF'}, "
        f"start_loc={start_loc_text}, end_loc={end_loc_text}"
    )


def run_with_overrides(overrides):
    """Convenience entry point for UI launcher."""
    apply_runtime_overrides(overrides)
    main()
    # Final garbage collection after simulation
    gc.collect()


def main():
    """Main entry point"""
    simulation = V2XAmbulanceSimulation()
    
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
        # Final garbage collection
        gc.collect()


if __name__ == "__main__":
    main()
