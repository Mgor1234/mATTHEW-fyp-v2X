"""
Ambulance Controller - Manages ambulance movement and V2X broadcast with Lidar-based collision avoidance
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'PythonAPI', 'carla'))

import carla
import math
import numpy as np
import time
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.global_route_planner import GlobalRoutePlanner
try:
    from agents.navigation.behavior_agent import BehaviorAgent
except Exception:
    BehaviorAgent = None
from config import *

# Calculate tick duration from target FPS
SIMULATION_TICK_DURATION = 1.0 / SIMULATION_FPS


class AmbulanceController:
    def __init__(self, vehicle, world_map, world=None):
        """
        Initialize ambulance controller with lidar-based collision avoidance
        
        Args:
            vehicle: CARLA vehicle actor (ambulance)
            world_map: CARLA map object
            world: CARLA world object (for sensor spawning)
        """
        self.vehicle = vehicle
        self.map = world_map
        self.world = world
        self.agent = self._build_agent()
        self.current_waypoint = None
        self.destination_waypoint = None
        self.destination_location = None
        self.route_path = []
        self.route_trace = []
        self.route_lane_keys = []
        self.route_progress_index = 0
        self.trip_started = False
        self.trip_complete = False
        self.start_time = None
        self.trip_duration = None
        self.total_distance = 0.0
        self.previous_location = vehicle.get_location()
        self.rotation_mode = False
        self.rotation_start_time = None
        self.rotation_duration = 15.0  # Rotate for 15 seconds
        self.rotation_speed = 10.0  # degrees per second
        
        # Lidar-based collision avoidance
        self.lidar_sensor = None
        self.lidar_data = None
        self.obstacle_ahead = False
        self.obstacle_distance = float('inf')
        self.stopped_vehicle_ahead = False
        self.stopped_vehicle_distance = float('inf')
        
        # Stuck detection - hold position when blocked
        self.stuck_frames = 0
        self.stuck_threshold = AMBULANCE_BLOCKED_FRAMES_THRESHOLD
        self.normal_target_speed = float(AMBULANCE_SPEED)
        self.attempting_left_dodge = False
        self.left_dodge_target_waypoint = None
        self.left_dodge_end_time = None
        self.left_dodge_started_at = None
        self.road_lock_tolerance = 1.8
        self._last_route_alignment_check = -999.0
        self.use_default_carla_logic = str(AMBULANCE_AGENT_TYPE).lower() == "basic"
        
        if world and not self.use_default_carla_logic:
            self._setup_lidar_sensor()

        if not self.use_default_carla_logic:
            self._enforce_emergency_priority()

    def _setup_lidar_sensor(self):
        """Setup lidar sensor for collision avoidance"""
        if not self.world:
            if DEBUG_PRINT:
                print("[AMBULANCE] Warning: No world object provided, skipping lidar setup")
            return
            
        try:
            print("[AMBULANCE] Initializing lidar sensor...")
            blueprint_library = self.world.get_blueprint_library()
            lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
            
            # Configure lidar for forward collision detection
            lidar_bp.set_attribute('channels', str(AMBULANCE_LIDAR_CHANNELS))
            lidar_bp.set_attribute('range', str(AMBULANCE_LIDAR_RANGE))
            lidar_bp.set_attribute('points_per_second', str(AMBULANCE_LIDAR_POINTS_PER_SECOND))
            lidar_bp.set_attribute('rotation_frequency', str(AMBULANCE_LIDAR_ROTATION_FREQUENCY))
            lidar_bp.set_attribute('upper_fov', str(AMBULANCE_LIDAR_UPPER_FOV))
            lidar_bp.set_attribute('lower_fov', str(AMBULANCE_LIDAR_LOWER_FOV))
            
            # Spawn lidar on front of vehicle
            spawn_point = carla.Transform(
                carla.Location(x=2.5, z=1.0),  # Front bumper, raised
                carla.Rotation(pitch=0)
            )
            
            print("[AMBULANCE] Spawning lidar sensor...")
            self.lidar_sensor = self.world.spawn_actor(
                lidar_bp,
                spawn_point,
                attach_to=self.vehicle
            )
            
            # Setup callback
            print("[AMBULANCE] Setting up lidar callback...")
            self.lidar_sensor.listen(lambda data: self._process_lidar_data(data))
            
            print("[AMBULANCE] Lidar sensor initialized successfully for collision avoidance")
                
        except Exception as e:
            print(f"[AMBULANCE] Warning: Could not setup lidar sensor: {e}")
            import traceback
            traceback.print_exc()
            self.lidar_sensor = None
    
    def _process_lidar_data(self, lidar_measurement):
        """
        Process lidar point cloud to detect obstacles ahead
        
        Args:
            lidar_measurement: CARLA lidar measurement
        """
        try:
            # Convert to numpy array (compatible with NumPy 2.x)
            points = np.frombuffer(lidar_measurement.raw_data, dtype=np.float32)
            points = np.reshape(points, (-1, 4))
            
            # Extract x, y, z coordinates (ignore intensity)
            xyz = points[:, :3]
            
            # Filter points in front of vehicle (positive x, limited y spread)
            # Focus on a 4m wide corridor in front, EXCLUDING ground (z > 0.3m above sensor)
            # Lidar is mounted at ~1m height, so ground appears at z ~ -1m
            # Only detect objects above vehicle height (z > 0.3 filters out road surface)
            forward_mask = (
                (xyz[:, 0] > 2.0) &  # At least 2m ahead (ignore bumper/hood)
                (xyz[:, 0] < 30.0) &  # Maximum 30m detection
                (np.abs(xyz[:, 1]) < 2.0) &  # Within 2m left/right
                (xyz[:, 2] > 0.3) &  # Above ground level (vehicles, not road)
                (xyz[:, 2] < 3.0)  # Below 3m (typical vehicle height)
            )
            forward_points = xyz[forward_mask]
            
            if len(forward_points) > 20:  # At least 20 points for reliable detection
                # Calculate distances
                distances = np.sqrt(np.sum(forward_points**2, axis=1))
                min_distance = np.min(distances)
                
                # Critical: < 5m, Warning: < 10m, Clear: > 10m
                if min_distance < 10.0:
                    self.obstacle_ahead = True
                    self.obstacle_distance = min_distance
                    print(f"[LIDAR] Obstacle detected: {min_distance:.2f}m, {len(forward_points)} points")
                else:
                    self.obstacle_ahead = False
                    self.obstacle_distance = min_distance
            else:
                self.obstacle_ahead = False
                self.obstacle_distance = float('inf')
                
        except Exception as e:
            print(f"[AMBULANCE] Lidar processing error: {e}")
            import traceback
            traceback.print_exc()
            self.obstacle_ahead = False
            self.obstacle_distance = float('inf')

    def _build_agent(self):
        """Create a default CARLA BasicAgent (no custom controller tuning)."""
        return BasicAgent(self.vehicle)

    def _project_to_road(self, location):
        """Project a location to the nearest drivable lane - ALWAYS ensure on-road waypoint."""
        try:
            waypoint = self.map.get_waypoint(
                location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
            return waypoint
        except TypeError:
            # Fallback for older CARLA versions
            waypoint = self.map.get_waypoint(location, project_to_road=True)
            return waypoint if waypoint and waypoint.lane_type == carla.LaneType.Driving else None
        
    def set_destination(self, start_location, end_location):
        """
        Set ambulance route from start to end location
        
        Args:
            start_location: carla.Location for start
            end_location: carla.Location for destination
        """
        # Validate that end location can be projected to road, and get current position
        destination_waypoint = self._project_to_road(end_location)
        if destination_waypoint is None:
            if DEBUG_PRINT:
                print("[AMBULANCE] Unable to project destination onto a driving lane")
            return False
        
        current_location = self.vehicle.get_location()
        self.previous_location = current_location
        
        self.destination_waypoint = destination_waypoint
        self.destination_location = destination_waypoint.transform.location if destination_waypoint else end_location
        # Compute shortest route using exact UI-selected coordinates for optimal path planning
        self.route_trace = self._trace_shortest_route(start_location, end_location)
        
        # Build visualization path from computed route, projecting start to road for visualization
        start_waypoint = self._project_to_road(start_location)
        self.route_path = self._build_route_path(start_waypoint, destination_waypoint, self.route_trace)
        self.route_progress_index = 0
        self.trip_started = True
        self.trip_complete = False
        self.start_time = None
        self.total_distance = 0.0
        
        # Set destination using a fixed shortest route to avoid random turns.
        if destination_waypoint is not None and self.route_trace:
            self.route_lane_keys = self._build_route_lane_keys(self.route_trace)
            self.agent.set_global_plan(
                self.route_trace,
                stop_waypoint_creation=True,
                clean_queue=True,
            )
            if DEBUG_PRINT:
                print(
                    f"[AMBULANCE] Fixed shortest route set with {len(self.route_trace)} waypoints "
                    f"to {destination_waypoint.transform.location}"
                )
        elif destination_waypoint is not None:
            if DEBUG_PRINT:
                print("[AMBULANCE] Unable to compute a shortest road route; refusing fallback path")
            return False

        return True

    def _enforce_strict_road_lock(self, current_location, control):
        """Force ambulance to remain on a drivable lane by correcting drift immediately."""
        current_waypoint = self._project_to_road(current_location)
        if current_waypoint is None:
            control.throttle = 0.0
            control.brake = max(control.brake, 1.0)
            control.steer = 0.0
            return control, current_location

        distance_to_road = current_location.distance(current_waypoint.transform.location)
        if distance_to_road <= self.road_lock_tolerance:
            return control, current_location

        corrected_transform = current_waypoint.transform
        corrected_transform.location.z += 0.05

        try:
            self.vehicle.set_transform(corrected_transform)
            self.vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            self.vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        except Exception:
            control.throttle = 0.0
            control.brake = max(control.brake, 1.0)
            control.steer = 0.0
            return control, current_location

        if self.attempting_left_dodge:
            self._finish_left_dodge()

        control.throttle = 0.0
        control.brake = max(control.brake, 0.8)
        control.steer = 0.0

        if DEBUG_PRINT:
            print(f"[AMBULANCE] Strict road lock corrected drift ({distance_to_road:.2f}m)")

        return control, corrected_transform.location

    def _trace_shortest_route(self, start_waypoint, destination_waypoint):
        """Return a deterministic shortest route trace between two locations (or waypoints).
        
        Args:
            start_waypoint: carla.Location or carla.Waypoint for route start
            destination_waypoint: carla.Location or carla.Waypoint for route end
        """
        # Handle both Location and Waypoint inputs
        start_loc = start_waypoint.transform.location if hasattr(start_waypoint, 'transform') else start_waypoint
        end_loc = destination_waypoint.transform.location if hasattr(destination_waypoint, 'transform') else destination_waypoint
        
        if start_loc is None or end_loc is None:
            return []

        try:
            planner = GlobalRoutePlanner(self.map, 2.0)
            return planner.trace_route(
                start_loc,
                end_loc,
            )
        except Exception as error:
            if DEBUG_PRINT:
                print(f"[AMBULANCE] Failed to trace shortest route: {error}")
            return []

    def _build_route_path(self, start_waypoint, destination_waypoint, route_trace=None):
        """Build a polyline for the ambulance route for visualization."""
        if start_waypoint is None or destination_waypoint is None:
            return []

        try:
            route_trace = route_trace if route_trace is not None else self._trace_shortest_route(start_waypoint, destination_waypoint)

            route_path = []
            for waypoint, _ in route_trace:
                location = waypoint.transform.location
                point = (float(location.x), float(location.y))
                if not route_path or route_path[-1] != point:
                    route_path.append(point)

            if not route_path:
                route_path.append((float(start_waypoint.transform.location.x), float(start_waypoint.transform.location.y)))
                route_path.append((float(destination_waypoint.transform.location.x), float(destination_waypoint.transform.location.y)))

            return route_path
        except Exception as error:
            if DEBUG_PRINT:
                print(f"[AMBULANCE] Failed to build route path: {error}")
            return []

    def _build_route_lane_keys(self, route_trace=None):
        """Cache the (road_id, lane_id) pairs used by NPC V2X route checks."""
        route_trace = route_trace if route_trace is not None else self.route_trace
        route_lane_keys = []
        for waypoint, _ in route_trace:
            try:
                route_lane_keys.append((int(waypoint.road_id), int(waypoint.lane_id)))
            except Exception:
                continue
        return route_lane_keys

    def get_route_path(self):
        """Return the planned ambulance route as a list of (x, y) points."""
        return list(self.route_path)

    def _route_waypoints(self):
        """Return the current planned route waypoints."""
        return [waypoint for waypoint, _ in self.route_trace]

    def _get_left_driving_waypoint(self, current_waypoint):
        """Return the adjacent left driving lane waypoint if one exists."""
        if current_waypoint is None:
            return None

        left_wp = current_waypoint.get_left_lane()
        if left_wp is None:
            return None
        if left_wp.lane_type != carla.LaneType.Driving:
            return None
        if left_wp.road_id != current_waypoint.road_id:
            return None
        if left_wp.lane_id * current_waypoint.lane_id < 0:
            return None
        return left_wp

    def _plan_left_dodge_target(self):
        """Plan a shallow temporary target in the left lane to pass a stopped vehicle."""
        current_location = self.vehicle.get_location()
        current_wp = self._project_to_road(current_location)
        if current_wp is None:
            return None

        left_wp = self._get_left_driving_waypoint(current_wp)
        if left_wp is None:
            return None

        # Keep the turn shallow: only a very short lane shift ahead.
        lookahead_distance = min(5.0, float(AMBULANCE_OVERTAKE_LOOKAHEAD))
        ahead = left_wp.next(lookahead_distance)
        if ahead:
            return ahead[0]
        return left_wp

    def _start_left_dodge(self, timestamp):
        """Start a left-lane dodge around a stopped lead vehicle."""
        if not self.stopped_vehicle_ahead:
            return False

        target_wp = self._plan_left_dodge_target()
        if target_wp is None:
            return False

        self.attempting_left_dodge = True
        self.left_dodge_target_waypoint = target_wp
        self.left_dodge_end_time = timestamp + 4.0
        self.left_dodge_started_at = timestamp
        self.stuck_frames = 0

        try:
            self.agent.set_destination(target_wp.transform.location)
        except Exception:
            self.attempting_left_dodge = False
            self.left_dodge_target_waypoint = None
            self.left_dodge_end_time = None
            self.left_dodge_started_at = None
            return False

        if DEBUG_PRINT:
            print(
                f"[AMBULANCE] Left dodge start: switching to lane {target_wp.lane_id} "
                f"toward {target_wp.transform.location}"
            )
        return True

    def _finish_left_dodge(self):
        """Return to the fixed shortest path after a left dodge."""
        self.attempting_left_dodge = False
        self.left_dodge_target_waypoint = None
        self.left_dodge_end_time = None
        self.left_dodge_started_at = None

        if self.destination_location is None:
            return

        current_wp = self._project_to_road(self.vehicle.get_location())
        destination_wp = self._project_to_road(self.destination_location)
        if current_wp is None or destination_wp is None:
            return

        route_trace = self._trace_shortest_route(current_wp, destination_wp)
        if route_trace:
            self.route_trace = route_trace
            self.route_path = self._build_route_path(current_wp, destination_wp, route_trace)
            self.route_progress_index = 0
            self.agent.set_global_plan(
                route_trace,
                stop_waypoint_creation=True,
                clean_queue=True,
            )
            if DEBUG_PRINT:
                print("[AMBULANCE] Left dodge complete: returned to main route")

    def _build_left_dodge_control(self, current_location, current_speed):
        """Create a small manual left-steer command to enter the adjacent lane."""
        control = carla.VehicleControl()
        target_wp = self.left_dodge_target_waypoint
        if target_wp is None:
            return control

        target_location = target_wp.transform.location
        yaw = math.radians(self.vehicle.get_transform().rotation.yaw)
        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)
        to_target_x = target_location.x - current_location.x
        to_target_y = target_location.y - current_location.y

        cross = forward_x * to_target_y - forward_y * to_target_x
        steer = max(-0.35, min(0.35, cross * 0.12))

        control.throttle = 0.28 if current_speed < 4.0 else 0.18
        control.brake = 0.0
        control.steer = steer
        return control

    def _ensure_route_alignment(self, current_location):
        """Keep the ambulance on the planned road corridor by re-planning when it drifts away."""
        if self.attempting_left_dodge:
            return

        now = time.time()
        alignment_interval = max(0.05, float(globals().get('AMBULANCE_ROUTE_ALIGNMENT_INTERVAL', 0.1)))
        if (now - self._last_route_alignment_check) < alignment_interval:
            return
        self._last_route_alignment_check = now

        if not self.route_trace or self.destination_waypoint is None:
            return

        route_waypoints = self._route_waypoints()
        if not route_waypoints:
            return

        search_start = max(0, self.route_progress_index - 5)
        search_end = min(len(route_waypoints), self.route_progress_index + 60)
        candidate_indices = range(search_start, search_end)

        nearest_index = None
        nearest_distance = float('inf')
        for idx in candidate_indices:
            waypoint = route_waypoints[idx]
            distance = current_location.distance(waypoint.transform.location)
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = idx

        if nearest_index is None:
            return

        self.route_progress_index = nearest_index

        # If the ambulance leaves the planned corridor, re-plan from the current road position.
        if nearest_distance > 6.0:
            current_wp = self._project_to_road(current_location)
            destination_wp = self._project_to_road(self.destination_location)
            if current_wp is None or destination_wp is None:
                return

            route_trace = self._trace_shortest_route(current_wp, destination_wp)
            if route_trace:
                self.route_trace = route_trace
                self.route_path = self._build_route_path(current_wp, destination_wp, route_trace)
                self.route_lane_keys = self._build_route_lane_keys(route_trace)
                self.route_progress_index = 0
                self.agent.set_global_plan(
                    route_trace,
                    stop_waypoint_creation=True,
                    clean_queue=True,
                )
                if DEBUG_PRINT:
                    print(
                        f"[AMBULANCE] Route alignment corrected: replanned from {current_wp.transform.location} "
                        f"to {destination_wp.transform.location}"
                    )

    def _enforce_emergency_priority(self):
        """Re-apply emergency behavior flags so red lights are always ignored."""
        if AMBULANCE_IGNORE_TRAFFIC_LIGHTS:
            if hasattr(self.agent, "ignore_traffic_lights"):
                self.agent.ignore_traffic_lights(True)
            else:
                # Fallback for agent implementations that do not expose a local ignore flag.
                traffic_light = self.vehicle.get_traffic_light()
                if traffic_light is not None and traffic_light.state == carla.TrafficLightState.Red:
                    traffic_light.set_state(carla.TrafficLightState.Green)

        if AMBULANCE_IGNORE_STOP_SIGNS and hasattr(self.agent, "ignore_stop_signs"):
            self.agent.ignore_stop_signs(True)

    def _set_traffic_light_ignore_mode(self, ignore_traffic_lights):
        """Toggle traffic-light behavior based on whether the lane ahead is blocked."""
        if hasattr(self.agent, "ignore_traffic_lights"):
            self.agent.ignore_traffic_lights(bool(ignore_traffic_lights))

    def _set_target_speed(self, speed_kmh):
        """Update agent target speed safely for both BasicAgent and BehaviorAgent."""
        speed_kmh = float(max(5.0, speed_kmh))
        if hasattr(self.agent, "set_target_speed"):
            self.agent.set_target_speed(speed_kmh)

    def _compute_turn_speed_limit_kmh(self, steer_abs):
        """Return a safe target speed limit (km/h) for current steering demand."""
        base = float(self.normal_target_speed)
        steer_abs = max(0.0, min(1.0, float(steer_abs)))

        # Keep speed high on straights, progressively lower through sharper turns.
        if steer_abs < 0.12:
            factor = 1.0
        elif steer_abs < 0.22:
            factor = 0.82
        elif steer_abs < 0.32:
            factor = 0.68
        else:
            factor = 0.55

        # Never drop below a low but moving speed during route following.
        return max(18.0, base * factor)

    def _apply_corner_safety(self, control, speed_mps):
        """Shape controls to avoid corner crashes by slowing before/while turning."""
        steer_abs = abs(float(control.steer))
        speed_kmh = float(speed_mps) * 3.6

        # Limit steering spikes to avoid sudden side impacts at corners.
        if steer_abs > 0.55:
            control.steer = max(-0.55, min(0.55, control.steer))
            steer_abs = abs(float(control.steer))

        turn_speed_limit_kmh = self._compute_turn_speed_limit_kmh(steer_abs)
        if speed_kmh > turn_speed_limit_kmh:
            # Over-speed in turn: aggressively cut throttle and brake proportionally.
            overshoot = max(0.0, speed_kmh - turn_speed_limit_kmh)
            overshoot_ratio = max(0.0, min(1.0, overshoot / max(1.0, turn_speed_limit_kmh)))
            control.throttle = min(control.throttle, 0.10)
            control.brake = max(control.brake, 0.22 + 0.55 * overshoot_ratio)
        elif steer_abs > 0.22:
            # Pre-emptive speed reduction in moderate turns.
            control.throttle = min(control.throttle, 0.35)
            control.brake = max(control.brake, 0.08 + 0.18 * ((steer_abs - 0.22) / 0.33))

        return control

    def _apply_turn_aware_obstacle_braking(self, control, speed_mps):
        """Increase collision margins and braking when obstacles are detected during turns."""
        if not (self.lidar_sensor and self.obstacle_ahead):
            return control

        steer_abs = abs(float(control.steer))
        turn_factor = 1.0 + min(0.45, steer_abs * 0.9)

        critical_distance = max(3.0, min(14.0, (speed_mps * AMBULANCE_CRITICAL_TIME_GAP + 1.8) * turn_factor))
        warning_distance = max(9.0, min(32.0, (speed_mps * AMBULANCE_WARNING_TIME_GAP + 3.5) * turn_factor))

        if self.obstacle_distance <= critical_distance:
            control.throttle = 0.0
            control.brake = 1.0
            if DEBUG_PRINT:
                print(
                    f"[AMBULANCE] TURN-AWARE EMERGENCY BRAKE: obstacle {self.obstacle_distance:.2f}m "
                    f"(critical {critical_distance:.2f}m, steer={steer_abs:.2f})"
                )
            return control

        if self.obstacle_distance <= warning_distance:
            span = max(0.1, warning_distance - critical_distance)
            closeness = (warning_distance - self.obstacle_distance) / span
            closeness = max(0.0, min(1.0, closeness))
            control.throttle = 0.0
            control.brake = max(control.brake, 0.30 + 0.60 * closeness)
            if DEBUG_PRINT:
                print(
                    f"[AMBULANCE] TURN-AWARE BRAKING: obstacle {self.obstacle_distance:.2f}m "
                    f"(warning {warning_distance:.2f}m, steer={steer_abs:.2f})"
                )

        return control

    def _scan_stopped_vehicle_ahead(self):
        """Detect the nearest stopped vehicle directly ahead in the current lane."""
        self.stopped_vehicle_ahead = False
        self.stopped_vehicle_distance = float('inf')

        if self.world is None:
            return

        ego_location = self.vehicle.get_location()
        ego_wp = self._project_to_road(ego_location)
        if ego_wp is None:
            return

        ego_forward = self.vehicle.get_transform().get_forward_vector()
        max_scan_distance = max(20.0, AMBULANCE_BLOCKED_DISTANCE_THRESHOLD + 10.0)

        try:
            all_vehicles = self.world.get_actors().filter('vehicle.*')
        except Exception:
            return

        nearest_distance = float('inf')
        for other in all_vehicles:
            if other.id == self.vehicle.id:
                continue

            other_location = other.get_location()
            delta = other_location - ego_location
            forward_projection = (
                delta.x * ego_forward.x
                + delta.y * ego_forward.y
                + delta.z * ego_forward.z
            )
            if forward_projection <= 0.0 or forward_projection > max_scan_distance:
                continue

            other_wp = self._project_to_road(other_location)
            if other_wp is None:
                continue

            if other_wp.road_id != ego_wp.road_id or other_wp.lane_id != ego_wp.lane_id:
                continue

            lane_half_width = max(1.5, 0.5 * float(getattr(ego_wp, 'lane_width', 3.5)))
            lateral_vector = carla.Vector3D(
                x=delta.x - forward_projection * ego_forward.x,
                y=delta.y - forward_projection * ego_forward.y,
                z=delta.z - forward_projection * ego_forward.z,
            )
            lateral_offset = math.sqrt(
                lateral_vector.x ** 2 + lateral_vector.y ** 2 + lateral_vector.z ** 2
            )
            if lateral_offset > lane_half_width:
                continue

            other_velocity = other.get_velocity()
            other_speed = math.sqrt(
                other_velocity.x ** 2 + other_velocity.y ** 2 + other_velocity.z ** 2
            )
            if other_speed > 0.5:
                continue

            distance = ego_location.distance(other_location)
            if distance < nearest_distance:
                nearest_distance = distance

        if nearest_distance < float('inf'):
            self.stopped_vehicle_ahead = True
            self.stopped_vehicle_distance = nearest_distance

    def get_v2x_broadcast(self):
        """
        Get V2X broadcast information (ambulance presence & location)
        Returns dict with ambulance state information
        """
        location = self.vehicle.get_location()
        velocity = self.vehicle.get_velocity()
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        return {
            'vehicle_id': self.vehicle.id,
            'location': location,
            'speed': speed,
            'velocity': velocity,
            'heading': self.vehicle.get_transform().rotation.yaw,
            'is_emergency': True,
            'v2x_range': V2X_DETECTION_RANGE,
            'route_path': list(self.route_path) if self.route_path else [],
            'route_lane_keys': list(self.route_lane_keys),
        }
    
    def update(self, timestamp):
        """
        Update ambulance state each frame
        
        Args:
            timestamp: Current simulation timestamp
            
        Returns:
            True if trip is still in progress, False if complete
        """
        if not self.trip_started:
            return False
        
        if self.start_time is None:
            self.start_time = timestamp
        
        # Handle rotation at destination
        if self.rotation_mode:
            if self.rotation_start_time is None:
                self.rotation_start_time = timestamp
            
            elapsed = timestamp - self.rotation_start_time
            if elapsed < self.rotation_duration:
                # Apply rotation control
                transform = self.vehicle.get_transform()
                new_yaw = transform.rotation.yaw + self.rotation_speed * SIMULATION_TICK_DURATION  # Tick duration
                transform.rotation.yaw = new_yaw
                self.vehicle.set_transform(transform)
                return True
            else:
                # Rotation complete
                return False
        
        # Update distance traveled
        current_location = self.vehicle.get_location()
        distance = current_location.distance(self.previous_location)
        self.total_distance += distance
        self.previous_location = current_location
        if not self.use_default_carla_logic:
            self._ensure_route_alignment(current_location)

        # Check if near destination
        if self.destination_waypoint:
            distance_to_destination = current_location.distance(
                self.destination_waypoint.transform.location
            )

            if distance_to_destination < 10.0:  # Within 10 meters
                self.trip_complete = True
                self.trip_duration = timestamp - self.start_time

                if DEBUG_PRINT:
                    print(f"[AMBULANCE] Trip complete in {self.trip_duration:.2f}s, "
                          f"distance: {self.total_distance:.2f}m")
                    print(f"[AMBULANCE] Starting 360° rotation...")

                # Enter rotation mode
                self.rotation_mode = True
                return True

        # Continue towards destination
        if self.agent.done():
            self.trip_complete = True
            self.trip_duration = timestamp - self.start_time
            self.rotation_mode = True
            return True

        # Default CARLA logic: BasicAgent handles the route and brakes for blocking vehicles.
        if self.use_default_carla_logic:
            self._scan_stopped_vehicle_ahead()
            self._set_traffic_light_ignore_mode(not self.stopped_vehicle_ahead)
            control = self.agent.run_step()
            self.vehicle.apply_control(control)
            return True

        self._enforce_emergency_priority()
        control = self.agent.run_step()
        
        # Safety check: ensure ambulance stays on road
        control, current_location = self._enforce_strict_road_lock(current_location, control)
        
        self.vehicle.apply_control(control)
        
        return True
    
    def get_trip_info(self):
        """Get trip completion information"""
        return {
            'complete': self.trip_complete,
            'duration': self.trip_duration,
            'distance': self.total_distance,
            'average_speed': self.total_distance / self.trip_duration if self.trip_duration else 0
        }
    
    def cleanup(self):
        """Clean up sensors and resources"""
        if self.lidar_sensor is not None:
            try:
                self.lidar_sensor.stop()
                self.lidar_sensor.destroy()
                if DEBUG_PRINT:
                    print("[AMBULANCE] Lidar sensor destroyed")
            except Exception as e:
                if DEBUG_PRINT:
                    print(f"[AMBULANCE] Error destroying lidar: {e}")
            finally:
                self.lidar_sensor = None
