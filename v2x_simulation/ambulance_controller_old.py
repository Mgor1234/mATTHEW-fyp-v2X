"""
Ambulance Controller - Manages ambulance movement and V2X broadcast using CARLA Traffic Manager default logic
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'PythonAPI', 'carla'))

import carla
import math
from config import *


class AmbulanceController:
    def __init__(self, vehicle, world_map, world=None, traffic_manager=None):
        """
        Initialize ambulance controller using CARLA Traffic Manager default logic
        
        Args:
            vehicle: CARLA vehicle actor (ambulance)
            world_map: CARLA map object
            world: CARLA world object (optional)
            traffic_manager: CARLA traffic manager instance
        """
        self.vehicle = vehicle
        self.map = world_map
        self.world = world
        self.traffic_manager = traffic_manager
        
        # Trip tracking
        self.destination_location = None
        self.trip_started = False
        self.trip_complete = False
        self.start_time = None
        self.trip_duration = None
        self.total_distance = 0.0
        self.previous_location = vehicle.get_location()
        self.collision_detected = False
        self.collision_event = None
        
        # Route visualization
        self.route_path = []

        self._enforce_emergency_priority()

    def _enforce_emergency_priority(self):
        """Enable emergency behavior flags so red lights are always ignored."""
        if AMBULANCE_IGNORE_TRAFFIC_LIGHTS:
            try:
                self.traffic_manager.ignore_lights_percentage(self.vehicle, 100.0)
            except Exception:
                pass
        
        # Set ambulance to follow traffic rules but ignore red lights
        if AMBULANCE_IGNORE_STOP_SIGNS:
            try:
                self.traffic_manager.ignore_signs_percentage(self.vehicle, 100.0)
            except Exception:
                pass

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
        Set ambulance route from start to end location using Traffic Manager default logic
        
        Args:
            start_location: carla.Location for start
            end_location: carla.Location for destination
        """
        # Validate destination can be reached
        destination_waypoint = self._project_to_road(end_location)
        if destination_waypoint is None:
            if DEBUG_PRINT:
                print("[AMBULANCE] Unable to project destination onto a driving lane")
            return False
        
        self.destination_location = end_location
        self.route_path = []  # No custom route visualization with TM
        self.trip_started = True
        self.trip_complete = False
        self.start_time = None
        self.total_distance = 0.0
        self.previous_location = self.vehicle.get_location()
        
        # Use Traffic Manager to set destination
        if self.traffic_manager:
            try:
                self.traffic_manager.set_path(self.vehicle, [end_location])
                if DEBUG_PRINT:
                    print(f"[AMBULANCE] Traffic Manager destination set to {end_location}")
            except Exception as e:
                if DEBUG_PRINT:
                    print(f"[AMBULANCE] Warning: Could not set TM path: {e}")
        
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

    def _halt_at_destination(self, current_location):
        """Bring the ambulance to a complete stop at the endpoint."""
        try:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
            self.vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            self.vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            if self.destination_waypoint is not None:
                destination_transform = self.destination_waypoint.transform
                destination_transform.location.z += 0.05
                self.vehicle.set_transform(destination_transform)
            elif current_location is not None:
                self.vehicle.set_transform(carla.Transform(current_location, self.vehicle.get_transform().rotation))
        except Exception:
            pass

    def _set_target_speed(self, speed_kmh):
        """Update agent target speed safely for both BasicAgent and BehaviorAgent."""
        speed_kmh = float(max(5.0, speed_kmh))
        if hasattr(self.agent, "set_target_speed"):
            self.agent.set_target_speed(speed_kmh)

    def _compute_turn_speed_limit_kmh(self, steer_abs):
        """Return a safe target speed limit (km/h) for current steering demand."""
        base = float(self.normal_target_speed)
        steer_abs = max(0.0, min(1.0, float(steer_abs)))

        # Keep speed LOW even on straights, progressively MUCH lower through any steering.
        if steer_abs < 0.05:
            factor = 1.0
        elif steer_abs < 0.10:
            factor = 0.70  # Reduced from 0.82
        elif steer_abs < 0.15:
            factor = 0.50  # Reduced from 0.68
        else:
            factor = 0.35  # Reduced from 0.55

        # Allow slightly higher speeds when needed for steering control
        return min(12.0, max(7.0, base * factor))

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

    def _apply_steer_rate_limit(self, control):
        """Limit steering rate to prevent sudden lane changes and maintain on-road stability."""
        target_steer = float(control.steer)
        max_change = self.max_steer_rate
        
        # Calculate the maximum steer we can change to this frame
        clamped_steer = max(self.last_steer - max_change, min(self.last_steer + max_change, target_steer))
        
        # Also apply max steer angle clamp from config
        max_steer_angle = float(AMBULANCE_MAX_STEER)
        clamped_steer = max(-max_steer_angle, min(max_steer_angle, clamped_steer))
        
        control.steer = clamped_steer
        self.last_steer = clamped_steer
        
        if DEBUG_PRINT and abs(target_steer - clamped_steer) > 0.01:
            print(f"[AMBULANCE] Steer rate limited: requested={target_steer:.3f}, applied={clamped_steer:.3f}")
        
        return control

    def _apply_turn_aware_obstacle_braking(self, control, speed_mps):
        """Increase collision margins and braking when obstacles are detected during turns."""
        if not (self.lidar_sensor and self.obstacle_ahead):
            return control

        steer_abs = abs(float(control.steer))
        # MUCH more aggressive turn factor - increase safety margins during any steering
        turn_factor = 1.0 + min(1.0, steer_abs * 2.0)

        critical_distance = max(3.0, min(16.0, (speed_mps * AMBULANCE_CRITICAL_TIME_GAP + 2.5) * turn_factor))
        warning_distance = max(10.0, min(35.0, (speed_mps * AMBULANCE_WARNING_TIME_GAP + 4.5) * turn_factor))

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

    def _estimate_route_turn_sharpness(self, current_location):
        """Estimate how sharp the next route turn is so the ambulance can slow early."""
        if not self.route_trace:
            return 0.0

        route_waypoints = self._route_waypoints()
        if len(route_waypoints) < 3:
            return 0.0

        search_start = max(0, self.route_progress_index)
        search_end = min(len(route_waypoints) - 2, self.route_progress_index + 12)
        if search_end <= search_start:
            return 0.0

        current_wp = self._project_to_road(current_location)
        if current_wp is None:
            current_wp = route_waypoints[min(self.route_progress_index, len(route_waypoints) - 1)]

        current_yaw = math.radians(current_wp.transform.rotation.yaw)
        heading_x = math.cos(current_yaw)
        heading_y = math.sin(current_yaw)

        sharpness = 0.0
        for idx in range(search_start, search_end):
            a = route_waypoints[idx].transform.location
            b = route_waypoints[idx + 1].transform.location
            c = route_waypoints[min(idx + 2, len(route_waypoints) - 1)].transform.location

            ab_x = b.x - a.x
            ab_y = b.y - a.y
            bc_x = c.x - b.x
            bc_y = c.y - b.y

            ab_mag = math.hypot(ab_x, ab_y)
            bc_mag = math.hypot(bc_x, bc_y)
            if ab_mag < 0.1 or bc_mag < 0.1:
                continue

            ab_x /= ab_mag
            ab_y /= ab_mag
            bc_x /= bc_mag
            bc_y /= bc_mag

            dot = max(-1.0, min(1.0, ab_x * bc_x + ab_y * bc_y))
            turn_angle = math.degrees(math.acos(dot))

            to_turn_x = a.x - current_location.x
            to_turn_y = a.y - current_location.y
            distance_ahead = math.hypot(to_turn_x, to_turn_y)
            direction_gate = heading_x * to_turn_x + heading_y * to_turn_y
            if direction_gate < -3.0:
                continue

            if distance_ahead > 60.0:
                continue

            if turn_angle > sharpness:
                sharpness = turn_angle

        return sharpness

    def _apply_pre_turn_speed_control(self, control, current_location, speed_mps, elapsed_since_start=0.0):
        """Slow the ambulance before sharp route bends so it does not clip objects."""
        if elapsed_since_start < self.launch_grace_period and speed_mps < 1.0:
            return control

        turn_sharpness = self._estimate_route_turn_sharpness(current_location)
        if turn_sharpness <= 0.0:
            return control

        speed_kmh = float(speed_mps) * 3.6

        # Pre-turn braking tuned to balance safety and maneuverability
        if turn_sharpness >= 60.0:
            # Sharp turn: reduce to a safe crawl but allow steering
            target_speed = min(10.0, self.normal_target_speed * 0.45)
            control.throttle = 0.0
            control.brake = max(control.brake, 0.55)
        elif turn_sharpness >= 35.0:
            # Moderate turn: slow but permit controlled steering
            target_speed = min(12.0, self.normal_target_speed * 0.65)
            control.throttle = min(control.throttle, 0.08)
            control.brake = max(control.brake, 0.30)
        elif turn_sharpness >= 15.0:
            # Light turn: reduce speed moderately
            target_speed = min(14.0, self.normal_target_speed * 0.85)
            control.throttle = min(control.throttle, 0.16)
            control.brake = max(control.brake, 0.10)
        else:
            target_speed = self._compute_turn_speed_limit_kmh(abs(control.steer))

        self._set_target_speed(target_speed)

        if speed_kmh > target_speed:
            overshoot = max(0.0, speed_kmh - target_speed)
            overshoot_ratio = max(0.0, min(1.0, overshoot / max(1.0, target_speed)))
            control.throttle = min(control.throttle, 0.12)
            control.brake = max(control.brake, 0.16 + 0.40 * overshoot_ratio)

        if DEBUG_PRINT and turn_sharpness >= 45.0:
            print(
                f"[AMBULANCE] Pre-turn slow-down: sharpness={turn_sharpness:.1f}deg, "
                f"target={target_speed:.1f} km/h, speed={speed_kmh:.1f} km/h"
            )

        return control

    def _apply_start_launch_boost(self, control, speed_mps, elapsed_since_start):
        """Give the ambulance a short launch boost so it can leave the spawn point cleanly."""
        if elapsed_since_start > self.launch_grace_period:
            return control

        if speed_mps >= 1.5:
            return control

        control.throttle = max(control.throttle, 0.55)
        control.brake = 0.0
        control.steer = max(-0.18, min(0.18, control.steer))
        self._set_target_speed(self.launch_boost_speed_kmh)
        if DEBUG_PRINT:
            print(
                f"[AMBULANCE] Launch boost active: elapsed={elapsed_since_start:.1f}s, speed={speed_mps:.2f} m/s"
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
        route_lane_keys = []
        for waypoint, _ in self.route_trace:
            try:
                route_lane_keys.append((int(waypoint.road_id), int(waypoint.lane_id)))
            except Exception:
                continue
        
        return {
            'vehicle_id': self.vehicle.id,
            'location': location,
            'speed': speed,
            'velocity': velocity,
            'heading': self.vehicle.get_transform().rotation.yaw,
            'is_emergency': True,
            'v2x_range': V2X_DETECTION_RANGE,
            'route_path': list(self.route_path) if self.route_path else [],
            'route_lane_keys': route_lane_keys,
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
        elapsed_since_start = timestamp - self.start_time
        
        # Handle rotation at destination
        if self.rotation_mode:
            if self.rotation_start_time is None:
                self.rotation_start_time = timestamp
            
            elapsed = timestamp - self.rotation_start_time
            if elapsed < self.rotation_duration:
                # Apply rotation control
                transform = self.vehicle.get_transform()
                new_yaw = transform.rotation.yaw + self.rotation_speed * 0.05  # 50ms tick
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
        self._ensure_route_alignment(current_location)
        self._scan_stopped_vehicle_ahead()

        # Check if near destination
        if self.destination_waypoint:
            distance_to_destination = current_location.distance(
                self.destination_waypoint.transform.location
            )

            if distance_to_destination < 10.0:  # Within 10 meters
                self.trip_complete = True
                self.trip_duration = timestamp - self.start_time

                self._halt_at_destination(current_location)

                if DEBUG_PRINT:
                    print(f"[AMBULANCE] Trip complete in {self.trip_duration:.2f}s, "
                          f"distance: {self.total_distance:.2f}m")

                return False

        # Continue towards destination
        if self.agent.done():
            self.trip_complete = True
            self.trip_duration = timestamp - self.start_time
            self._halt_at_destination(current_location)
            return False

        # Default CARLA vehicle logic: follow global plan using BasicAgent controller output.
        self._enforce_emergency_priority()
        control = self.agent.run_step()

        speed_vector = self.vehicle.get_velocity()
        speed_mps = math.sqrt(speed_vector.x ** 2 + speed_vector.y ** 2 + speed_vector.z ** 2)

        # Apply steering rate limiter first for smooth, gradual steering changes
        control = self._apply_steer_rate_limit(control)
        
        # Apply only essential modifiers - let BehaviorAgent handle corner safety naturally
        control = self._apply_start_launch_boost(control, speed_mps, elapsed_since_start)
        control = self._apply_turn_aware_obstacle_braking(control, speed_mps)
        
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
            'average_speed': self.total_distance / self.trip_duration if self.trip_duration else 0,
            'collision_detected': self.collision_detected,
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

        if self.collision_sensor is not None:
            try:
                self.collision_sensor.stop()
                self.collision_sensor.destroy()
                if DEBUG_PRINT:
                    print("[AMBULANCE] Collision sensor destroyed")
            except Exception as e:
                if DEBUG_PRINT:
                    print(f"[AMBULANCE] Error destroying collision sensor: {e}")
            finally:
                self.collision_sensor = None
                self.collision_detected = False
                self.collision_event = None
