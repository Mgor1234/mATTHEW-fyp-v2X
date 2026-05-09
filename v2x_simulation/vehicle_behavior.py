"""
Vehicle Behavior - NPC vehicles with V2X-based ambulance detection and path clearing
Implements Hybrid Model: IDM + Rule-Based Protocol + V2X Communication
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
from config import *


class IntelligentDriverModel:
    """IDM-based car following model with emergency parameters"""
    
    def __init__(self):
        # Normal driving parameters
        self.safe_time_headway = 1.5  # seconds
        self.min_gap = 2.0  # meters
        self.max_acceleration = 3.0  # m/s^2
        self.comfortable_decel = 3.0  # m/s^2
        
        # Emergency driving parameters
        self.emergency_time_headway = 3.5  # seconds (increased gap)
        self.emergency_decel = 6.0  # m/s^2 (emergency braking)
        self.emergency_mode = False
    
    def set_emergency_mode(self, enabled):
        """Enable/disable emergency response mode"""
        self.emergency_mode = enabled
    
    def calculate_desired_gap(self, current_speed, speed_difference):
        """Calculate desired gap using IDM formula"""
        time_headway = self.emergency_time_headway if self.emergency_mode else self.safe_time_headway
        decel = self.emergency_decel if self.emergency_mode else self.comfortable_decel
        
        # IDM desired gap formula
        dynamic_term = (current_speed * speed_difference) / (2 * math.sqrt(self.max_acceleration * decel))
        desired_gap = self.min_gap + (current_speed * time_headway) + max(0, dynamic_term)
        
        return desired_gap
    
    def calculate_acceleration(self, current_speed, desired_speed, gap_to_leader, leader_speed):
        """Calculate acceleration using IDM model"""
        if gap_to_leader <= 0:
            return -self.emergency_decel
        
        speed_ratio = current_speed / max(desired_speed, 0.1)
        speed_difference = current_speed - leader_speed
        desired_gap = self.calculate_desired_gap(current_speed, speed_difference)
        gap_ratio = desired_gap / gap_to_leader
        
        # IDM acceleration formula
        free_road_term = 1.0 - pow(speed_ratio, 4)
        interaction_term = pow(gap_ratio, 2)
        acceleration = self.max_acceleration * (free_road_term - interaction_term)
        
        # Limit acceleration/deceleration
        max_decel = self.emergency_decel if self.emergency_mode else self.comfortable_decel
        return max(-max_decel, min(self.max_acceleration, acceleration))


class V2XVehicleBehavior:
    """Vehicle behavior wrapper with type-specific emergency handling."""
    
    def __init__(self, vehicle, world_map, world=None, use_v2x=True, use_lidar=True, destination_location=None):
        """
        Initialize vehicle behavior with IDM and rule-based protocol
        
        Args:
            vehicle: CARLA vehicle actor
            world_map: CARLA map object
            world: CARLA world object (for optional lidar)
        """
        self.vehicle = vehicle
        self.map = world_map
        self.world = world
        self.use_v2x = use_v2x
        self.use_lidar = use_lidar
        self.agent = BasicAgent(vehicle, target_speed=REGULAR_VEHICLE_SPEED)
        self.destination = None
        self.original_destination = None
        self.moving = False
        self.detected_ambulance = None
        self.last_detection_time = None
        self.clearing_path = False
        self.initial_speed = REGULAR_VEHICLE_SPEED
        self.clearance_target_until = 0.0
        self.last_clearance_replan = -999.0
        self.last_type_c_reroute = -999.0
        self.type_c_reroute_until = 0.0
        self._route_planner = None
        self.type_b_switching_lane = False
        self.type_b_pre_switch_lane_key = None
        self.type_b_switch_cooldown_until = 0.0
        self.type_c_last_overlap_check_time = -999.0
        self.type_c_cached_overlap = False
        self.type_c_force_reroute = False
        self.road_lock_tolerance = max(0.5, float(globals().get('NPC_ROAD_LOCK_MAX_DRIFT', 2.2)))
        self.road_lock_recovery_cooldown = max(0.2, float(globals().get('NPC_ROAD_LOCK_RECOVERY_COOLDOWN', 1.0)))
        self.last_road_lock_recovery = -999.0

        # Optional lidar-based collision avoidance
        self.lidar_sensor = None
        self.obstacle_ahead = False
        self.obstacle_distance = float('inf')
        self.rear_obstacle_detected = False
        self.rear_obstacle_distance = float('inf')
        if self.use_lidar and NPC_LIDAR_ENABLED and self.world and NPC_LIDAR_MODE == "all":
            self._setup_lidar_sensor()
        
        # IDM controller
        self.idm = IntelligentDriverModel()
        
        # Emergency response state
        self.emergency_state = {
            'urgency_level': 0,  # 0=normal, 1=aware, 2=clearing, 3=critical
            'predicted_ambulance_path': [],
            'at_intersection': False,
            'target_lane_offset': 0.0,  # Positive = right, negative = left
            'should_stop': False
        }
        
        if destination_location is not None:
            self._set_destination(destination_location, keep_original=True)
        else:
            self._set_random_destination()

    def _reset_emergency_state(self):
        self.detected_ambulance = None
        self.clearing_path = False
        self.type_c_force_reroute = False
        self.emergency_state['urgency_level'] = 0
        self.emergency_state['predicted_ambulance_path'] = []
        self.idm.set_emergency_mode(False)

    def _build_ambulance_corridor_points(self):
        """Build a corridor from predicted and planned ambulance route points."""
        if not self.detected_ambulance:
            return []

        corridor_points = []
        ambulance_location = self.detected_ambulance.get('location')
        if ambulance_location is not None:
            corridor_points.append(ambulance_location)

        predictions = self.emergency_state.get('predicted_ambulance_path', [])
        corridor_points.extend(predictions)

        planned_route = self.detected_ambulance.get('route_path', [])
        route_z = ambulance_location.z if ambulance_location is not None else self.vehicle.get_location().z
        for idx, point in enumerate(planned_route):
            if idx % 3 != 0:
                continue
            try:
                x, y = point
                corridor_points.append(carla.Location(x=float(x), y=float(y), z=route_z))
            except Exception:
                continue

        return corridor_points

    def _get_ambulance_route_lane_keys(self):
        """Return set of (road_id, lane_id) pairs for the ambulance planned route."""
        if not self.detected_ambulance:
            return set()

        lane_keys = set()
        for pair in self.detected_ambulance.get('route_lane_keys', []):
            if not isinstance(pair, (list, tuple)) or len(pair) < 2:
                continue
            try:
                lane_keys.add((int(pair[0]), int(pair[1])))
            except Exception:
                continue
        return lane_keys

    def _get_current_lane_key(self):
        current_wp = self._get_current_driving_waypoint()
        if current_wp is None:
            return None
        return (int(current_wp.road_id), int(current_wp.lane_id))

    def _is_current_lane_on_ambulance_route(self):
        lane_key = self._get_current_lane_key()
        if lane_key is None:
            return False
        return lane_key in self._get_ambulance_route_lane_keys()

    def _is_same_lane_as_ambulance(self):
        """Return True only when this vehicle and ambulance are on the exact same driving lane."""
        if not self.detected_ambulance:
            return False

        try:
            current_wp = self._get_current_driving_waypoint()
            if current_wp is None:
                return False

            ambulance_location = self.detected_ambulance.get('location')
            if ambulance_location is None:
                return False

            try:
                ambulance_wp = self.map.get_waypoint(
                    ambulance_location,
                    project_to_road=True,
                    lane_type=carla.LaneType.Driving,
                )
            except TypeError:
                ambulance_wp = self.map.get_waypoint(ambulance_location, project_to_road=True)

            if ambulance_wp is None:
                return False

            return (
                ambulance_wp.road_id == current_wp.road_id
                and ambulance_wp.lane_id == current_wp.lane_id
            )
        except Exception:
            return False

    def _is_ambulance_behind(self):
        """Return True when the detected ambulance is behind this vehicle."""
        if not self.detected_ambulance:
            return False

        try:
            vehicle_location = self.vehicle.get_location()
            ambulance_location = self.detected_ambulance.get('location')
            if ambulance_location is None:
                return False

            vehicle_transform = self.vehicle.get_transform()
            forward_vector = vehicle_transform.get_forward_vector()
            to_ambulance = carla.Vector3D(
                x=ambulance_location.x - vehicle_location.x,
                y=ambulance_location.y - vehicle_location.y,
                z=0.0,
            )
            dot_product = forward_vector.x * to_ambulance.x + forward_vector.y * to_ambulance.y
            return dot_product < 0.0
        except Exception:
            return False

    def _is_ambulance_at_rear_end(self, distance_to_ambulance):
        """Return True only when ambulance is closely aligned behind this vehicle's rear."""
        if not self.detected_ambulance:
            return False
        if distance_to_ambulance is None:
            return False

        try:
            vehicle_transform = self.vehicle.get_transform()
            vehicle_location = vehicle_transform.location
            ambulance_location = self.detected_ambulance.get('location')
            if ambulance_location is None:
                return False

            forward_vector = vehicle_transform.get_forward_vector()
            dx = float(ambulance_location.x - vehicle_location.x)
            dy = float(ambulance_location.y - vehicle_location.y)

            longitudinal = (forward_vector.x * dx) + (forward_vector.y * dy)
            if longitudinal >= 0.0:
                return False

            rear_distance = -longitudinal
            max_rear_distance = max(
                4.0,
                float(globals().get('TYPE_B_AMBULANCE_REAR_END_DISTANCE', 7.5)),
            )
            if rear_distance > max_rear_distance:
                return False

            lateral_sq = max(0.0, (distance_to_ambulance * distance_to_ambulance) - (longitudinal * longitudinal))
            lateral_distance = math.sqrt(lateral_sq)
            max_lateral_distance = max(
                1.2,
                float(globals().get('TYPE_B_AMBULANCE_REAR_LATERAL_TOLERANCE', 2.2)),
            )
            return lateral_distance <= max_lateral_distance
        except Exception:
            return False

    def _ensure_route_planner(self):
        if self._route_planner is None:
            self._route_planner = GlobalRoutePlanner(self.map, 2.0)

    def _trace_route(self, start_location, end_location):
        try:
            self._ensure_route_planner()
            return self._route_planner.trace_route(start_location, end_location)
        except Exception:
            return []

    def _route_lane_keys(self, route_trace):
        lane_keys = set()
        for waypoint, _ in route_trace:
            try:
                lane_keys.add((int(waypoint.road_id), int(waypoint.lane_id)))
            except Exception:
                continue
        return lane_keys

    def _set_destination(self, destination_location, keep_original=False):
        try:
            self.agent.set_destination(destination_location)
            self.destination = destination_location
            if keep_original or self.original_destination is None:
                self.original_destination = destination_location
            self.moving = True
            return True
        except Exception:
            return False

    def _type_c_route_overlaps_ambulance(self):
        if not (self.use_v2x and not self.use_lidar and self.detected_ambulance):
            return False

        # Fast path: same-lane conflict requires immediate reroute.
        lane_key = self._get_current_lane_key()
        ambulance_lane_keys = self._get_ambulance_route_lane_keys()
        if lane_key is not None and lane_key in ambulance_lane_keys:
            self.type_c_cached_overlap = True
            return True

        # Throttle expensive route tracing for all Type C vehicles.
        now = time.time()
        check_interval = max(0.2, float(globals().get('TYPE_C_ROUTE_OVERLAP_CHECK_INTERVAL', 1.0)))
        if (now - self.type_c_last_overlap_check_time) < check_interval:
            return self.type_c_cached_overlap
        self.type_c_last_overlap_check_time = now

        if not ambulance_lane_keys:
            self.type_c_cached_overlap = False
            return False

        target_destination = self.original_destination or self.destination
        if target_destination is None:
            self.type_c_cached_overlap = False
            return False

        current_location = self.vehicle.get_location()
        route_trace = self._trace_route(current_location, target_destination)
        if not route_trace:
            self.type_c_cached_overlap = False
            return False

        self.type_c_cached_overlap = bool(self._route_lane_keys(route_trace) & ambulance_lane_keys)
        return self.type_c_cached_overlap

    def _select_left_lane_target(self, lookahead=None):
        """Select a same-direction left-lane target on the same road."""
        current_wp = self._get_current_driving_waypoint()
        if current_wp is None:
            return None

        left_wp = current_wp.get_left_lane()
        if left_wp is None:
            return None
        if left_wp.lane_type != carla.LaneType.Driving:
            return None
        if left_wp.road_id != current_wp.road_id:
            return None
        if left_wp.lane_id * current_wp.lane_id < 0:
            return None

        if lookahead is None:
            lookahead = max(5.0, float(globals().get('TYPE_B_LEFT_EVASION_LOOKAHEAD', 18.0)))

        ahead = left_wp.next(lookahead)
        target_wp = ahead[0] if ahead else left_wp
        return target_wp.transform.location

    def _plan_left_lane_evasion(self, timestamp):
        """Type B strategy: shift to left lane only when this vehicle confirms rear detection conditions."""
        if self.use_v2x:
            return False
        if self.detected_ambulance is None:
            return False

        distance_to_ambulance = None
        try:
            distance_to_ambulance = self.vehicle.get_location().distance(self.detected_ambulance['location'])
        except Exception:
            return False

        # Hard safety gate: never trigger or continue a Type B swerve unless
        # this individual vehicle currently detects the ambulance behind within threshold.
        if not self._should_type_b_clear_now(distance_to_ambulance):
            self.clearing_path = False
            return False

        cooldown = max(0.2, float(globals().get('TYPE_B_LEFT_EVASION_REPLAN_COOLDOWN', 1.0)))
        if (timestamp - self.last_clearance_replan) < cooldown:
            return False

        target_location = self._select_left_lane_target()
        if target_location is None:
            return False

        try:
            self.agent.set_destination(target_location)
            self.last_clearance_replan = timestamp
            self.clearance_target_until = timestamp + 1.5
            return True
        except Exception:
            return False

    def _select_type_b_lane_switch_target(self):
        """Select only the left-side adjacent lane target for Type B evasion."""
        current_wp = self._get_current_driving_waypoint()
        if current_wp is None:
            return None

        left_wp = current_wp.get_left_lane()
        if left_wp is None:
            return None

        if left_wp.lane_type != carla.LaneType.Driving:
            return None
        if left_wp.road_id != current_wp.road_id:
            return None
        if left_wp.lane_id * current_wp.lane_id < 0:
            return None

        lookahead = max(10.0, float(globals().get('TYPE_B_LANE_SWITCH_LOOKAHEAD', 20.0)))
        ahead = left_wp.next(lookahead)
        target_wp = ahead[0] if ahead else left_wp
        return target_wp.transform.location

    def _plan_type_b_lane_switch(self, timestamp):
        """Trigger a left-lane-only switch and keep following that lane while clearing."""
        if self.type_b_switching_lane:
            return False
        if timestamp < self.type_b_switch_cooldown_until:
            return False

        target_location = self._select_type_b_lane_switch_target()
        if target_location is None:
            return False

        current_lane = self._get_current_lane_key()
        if current_lane is None:
            return False

        if not self._set_destination(target_location, keep_original=False):
            return False

        self.type_b_pre_switch_lane_key = current_lane
        self.type_b_switching_lane = True
        self.type_b_switch_cooldown_until = timestamp + max(1.0, float(globals().get('TYPE_B_LANE_SWITCH_COOLDOWN', 4.0)))
        return True

    def _complete_type_b_lane_switch_if_ready(self):
        if not self.type_b_switching_lane:
            return

        # Stay in the left lane while the ambulance is still nearby.
        if self.clearing_path and self.detected_ambulance is not None:
            return

        current_lane = self._get_current_lane_key()
        if current_lane is None or self.type_b_pre_switch_lane_key is None:
            return

        if current_lane != self.type_b_pre_switch_lane_key:
            self.type_b_switching_lane = False
            self.type_b_pre_switch_lane_key = None
            self.clearing_path = False
            if self.original_destination is not None:
                self._set_destination(self.original_destination, keep_original=True)

    def _should_type_b_clear_now(self, distance_to_ambulance):
        """Type B clears only when lidar sees a rear obstacle and ambulance is behind nearby."""
        if self.use_v2x:
            return False
        if self.detected_ambulance is None:
            return False
        if distance_to_ambulance is None:
            return False

        nearby_threshold = max(6.0, float(globals().get('TYPE_B_AMBULANCE_REAR_DETECTION_DISTANCE', 10.0)))
        if distance_to_ambulance > nearby_threshold:
            return False

        if not self._is_same_lane_as_ambulance():
            return False
        if not self._is_ambulance_behind():
            return False
        if not self._is_ambulance_at_rear_end(distance_to_ambulance):
            return False
        if not self.use_lidar:
            return False
        if not self.rear_obstacle_detected:
            return False
        if self.rear_obstacle_distance > nearby_threshold + 2.0:
            return False

        # Ensure the rear lidar contact corresponds to ambulance distance, not unrelated traffic.
        rear_match_tolerance = max(1.0, float(globals().get('TYPE_B_REAR_MATCH_TOLERANCE', 2.5)))
        if abs(float(self.rear_obstacle_distance) - float(distance_to_ambulance)) > rear_match_tolerance:
            return False

        return True

    def _restore_original_destination_if_needed(self):
        if self.clearing_path:
            return

        # Type C keeps awareness context while ambulance is detected.
        # Type B should immediately return to the original route when rear detection
        # conditions are no longer met.
        if self.use_v2x and self.detected_ambulance is not None:
            return
        if self.original_destination is None:
            return
        if self.destination is None:
            return

        try:
            if self.destination.distance(self.original_destination) > 1.0:
                self._set_destination(self.original_destination, keep_original=True)
        except Exception:
            pass
    
    def _set_random_destination(self):
        """Set a random roaming destination for this vehicle."""
        try:
            waypoints = self.map.get_spawn_points()
            if waypoints:
                import random
                dest = random.choice(waypoints).location
                self._set_destination(dest, keep_original=True)
        except:
            pass

    def _set_forward_lane_destination(self, lookahead=45.0):
        """Set a destination ahead on the current driving lane to avoid turning plans."""
        try:
            current_wp = self._get_current_driving_waypoint()
            if current_wp is None:
                return False

            ahead = current_wp.next(float(lookahead))
            target_wp = ahead[0] if ahead else current_wp
            target_loc = target_wp.transform.location

            self.agent.set_destination(target_loc)
            self.destination = target_loc
            self.moving = True
            return True
        except Exception:
            return False

    def _should_enable_lidar(self, distance_to_ambulance=None):
        if not self.use_lidar or not NPC_LIDAR_ENABLED or not self.world:
            return False
        if not self.use_v2x:
            if distance_to_ambulance is None:
                return False
            activation_distance = max(8.0, float(globals().get('TYPE_B_LIDAR_ACTIVATION_DISTANCE', 25.0)))
            return distance_to_ambulance <= activation_distance
        if NPC_LIDAR_MODE == "all":
            return True
        if NPC_LIDAR_MODE == "nearby" and distance_to_ambulance is not None:
            return distance_to_ambulance <= NPC_LIDAR_RADIUS
        return False

    def _setup_lidar_sensor(self):
        if self.lidar_sensor or not self.world:
            return
        try:
            blueprint_library = self.world.get_blueprint_library()
            lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('channels', str(NPC_LIDAR_CHANNELS))
            lidar_bp.set_attribute('range', str(NPC_LIDAR_RANGE))
            lidar_bp.set_attribute('points_per_second', str(NPC_LIDAR_POINTS_PER_SECOND))
            lidar_bp.set_attribute('rotation_frequency', str(NPC_LIDAR_ROTATION_FREQUENCY))
            lidar_bp.set_attribute('upper_fov', str(NPC_LIDAR_UPPER_FOV))
            lidar_bp.set_attribute('lower_fov', str(NPC_LIDAR_LOWER_FOV))

            spawn_point = carla.Transform(
                carla.Location(x=2.0, z=1.0),
                carla.Rotation(pitch=0)
            )

            self.lidar_sensor = self.world.spawn_actor(
                lidar_bp,
                spawn_point,
                attach_to=self.vehicle
            )
            self.lidar_sensor.listen(lambda data: self._process_lidar_data(data))
        except Exception as e:
            if DEBUG_PRINT:
                print(f"[VEHICLE {self.vehicle.id}] Lidar setup failed: {e}")
            self.lidar_sensor = None

    def _ensure_lidar(self, distance_to_ambulance=None):
        should_enable = self._should_enable_lidar(distance_to_ambulance)

        if should_enable and self.lidar_sensor:
            return
        if should_enable:
            self._setup_lidar_sensor()
            return

        if self.lidar_sensor:
            try:
                self.lidar_sensor.destroy()
            except Exception:
                pass
            self.lidar_sensor = None
            self.obstacle_ahead = False
            self.obstacle_distance = float('inf')
            self.rear_obstacle_detected = False
            self.rear_obstacle_distance = float('inf')

    def _process_lidar_data(self, lidar_measurement):
        try:
            points = np.frombuffer(lidar_measurement.raw_data, dtype=np.float32)
            points = np.reshape(points, (-1, 4))
            xyz = points[:, :3]

            forward_mask = (
                (xyz[:, 0] > 1.5) &
                (xyz[:, 0] < NPC_LIDAR_RANGE) &
                (np.abs(xyz[:, 1]) < 2.5) &
                (xyz[:, 2] > 0.3) &
                (xyz[:, 2] < 3.0)
            )

            forward_points = xyz[forward_mask]
            if len(forward_points) >= NPC_LIDAR_MIN_POINTS:
                distances = np.linalg.norm(forward_points[:, :2], axis=1)
                self.obstacle_distance = float(np.min(distances))
                self.obstacle_ahead = True
            else:
                self.obstacle_ahead = False
                self.obstacle_distance = float('inf')

            rear_mask = (
                (xyz[:, 0] < -1.0) &
                (xyz[:, 0] > -NPC_LIDAR_RANGE) &
                (np.abs(xyz[:, 1]) < 2.5) &
                (xyz[:, 2] > 0.2) &
                (xyz[:, 2] < 3.0)
            )
            rear_points = xyz[rear_mask]
            if len(rear_points) >= NPC_LIDAR_MIN_POINTS:
                rear_distances = np.linalg.norm(rear_points[:, :2], axis=1)
                self.rear_obstacle_distance = float(np.min(rear_distances))
                self.rear_obstacle_detected = True
            else:
                self.rear_obstacle_detected = False
                self.rear_obstacle_distance = float('inf')
        except Exception:
            self.obstacle_ahead = False
            self.obstacle_distance = float('inf')
            self.rear_obstacle_detected = False
            self.rear_obstacle_distance = float('inf')

    def _apply_lidar_avoidance(self, control):
        if not self.lidar_sensor or not self.obstacle_ahead:
            return control

        if self.obstacle_distance < NPC_LIDAR_STOP_DISTANCE:
            control.throttle = 0.0
            control.brake = 1.0
            return control

        if self.obstacle_distance < NPC_LIDAR_SLOW_DISTANCE:
            control.throttle = min(control.throttle, 0.4)
            control.brake = max(control.brake, 0.2)

        return control

    def _get_current_driving_waypoint(self):
        """Return the nearest drivable waypoint for the current vehicle location."""
        try:
            location = self.vehicle.get_location()
            return self.map.get_waypoint(
                location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )
        except TypeError:
            # Backward compatibility for CARLA builds without lane_type argument.
            try:
                return self.map.get_waypoint(self.vehicle.get_location(), project_to_road=True)
            except Exception:
                return None
        except Exception:
            return None

    def _enforce_strict_road_lock(self, timestamp, control):
        """Keep NPC vehicles on drivable lanes and recover quickly if pushed off-road."""
        if not bool(globals().get('NPC_ROAD_LOCK_ENABLED', True)):
            return control

        current_wp = self._get_current_driving_waypoint()
        if current_wp is None:
            control.throttle = 0.0
            control.brake = max(control.brake, 1.0)
            control.steer = 0.0
            return control

        current_location = self.vehicle.get_location()
        drift = current_location.distance(current_wp.transform.location)
        if drift <= self.road_lock_tolerance:
            return control

        if (timestamp - self.last_road_lock_recovery) < self.road_lock_recovery_cooldown:
            control.throttle = 0.0
            control.brake = max(control.brake, 0.8)
            control.steer = 0.0
            return control

        corrected_transform = current_wp.transform
        corrected_transform.location.z += 0.05
        try:
            self.vehicle.set_transform(corrected_transform)
            self.vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            self.vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            self.last_road_lock_recovery = float(timestamp)
        except Exception:
            control.throttle = 0.0
            control.brake = max(control.brake, 1.0)
            control.steer = 0.0
            return control

        control.throttle = 0.0
        control.brake = max(control.brake, 0.7)
        control.steer = 0.0
        return control

    def _select_clearance_lane_target(self, lookahead=25.0):
        """Pick a safe adjacent driving lane target that increases distance from ambulance path."""
        current_wp = self._get_current_driving_waypoint()
        if current_wp is None:
            return None

        candidates = []
        for lane_wp in (current_wp.get_left_lane(), current_wp.get_right_lane()):
            if lane_wp is None:
                continue
            if lane_wp.lane_type != carla.LaneType.Driving:
                continue
            # Keep same road segment and travel direction.
            if lane_wp.road_id != current_wp.road_id:
                continue
            if lane_wp.lane_id * current_wp.lane_id < 0:
                continue
            ahead = lane_wp.next(lookahead)
            target_wp = ahead[0] if ahead else lane_wp
            candidates.append(target_wp)

        if not candidates:
            return None

        ambulance_loc = None
        if self.detected_ambulance:
            ambulance_loc = self.detected_ambulance.get('location')

        if ambulance_loc is None:
            return candidates[0].transform.location

        # Prefer the adjacent lane that maximizes distance from ambulance.
        best_wp = max(
            candidates,
            key=lambda wp: wp.transform.location.distance(ambulance_loc),
        )
        return best_wp.transform.location

    def _plan_clearance_lane_change(self, timestamp):
        """Set a temporary adjacent-lane destination for safe V2X path clearing."""
        if timestamp < self.clearance_target_until and (timestamp - self.last_clearance_replan) < 1.0:
            return False

        target_location = self._select_clearance_lane_target()
        if target_location is None:
            return False

        try:
            self.agent.set_destination(target_location)
            self.last_clearance_replan = timestamp
            self.clearance_target_until = timestamp + 2.0
            return True
        except Exception:
            return False

    def _vector_length_2d(self, x, y):
        return math.sqrt(x * x + y * y)

    def _min_distance_to_corridor(self, location, corridor_points):
        if not corridor_points:
            return float('inf')
        return min(location.distance(point) for point in corridor_points)

    def _select_type_c_lane_target(self, corridor_points, lookahead=30.0):
        """Choose a forward target that only performs lane switches (no turning reroute).

        Type C policy:
        - Never set side/turning destinations.
        - Only use same-road, same-direction adjacent lanes.
        - Prefer leaving the ambulance-route lane when possible.
        """
        current_wp = self._get_current_driving_waypoint()
        if current_wp is None:
            return None

        # Avoid lane-switch replans inside junctions where steering is ambiguous.
        if getattr(current_wp, 'is_junction', False):
            return None

        ambulance_route_lanes = self._get_ambulance_route_lane_keys()
        current_lane_key = (int(current_wp.road_id), int(current_wp.lane_id))
        force_leave_current_lane = current_lane_key in ambulance_route_lanes

        # Evaluate adjacent lanes only to force true lane-switch behavior.
        lane_candidates = []
        for lane_wp in (current_wp.get_left_lane(), current_wp.get_right_lane()):
            if lane_wp is None:
                continue
            if lane_wp.lane_type != carla.LaneType.Driving:
                continue
            if lane_wp.road_id != current_wp.road_id:
                continue
            if lane_wp.lane_id * current_wp.lane_id < 0:
                continue
            lane_candidates.append(lane_wp)

        # If no adjacent lane exists, keep driving forward in current lane (no turns).
        if not lane_candidates:
            ahead = current_wp.next(float(lookahead))
            target_wp = ahead[0] if ahead else current_wp
            return target_wp.transform.location

        # Prefer adjacent lane that is not on ambulance route when current lane conflicts.
        if force_leave_current_lane:
            non_route_candidates = [
                lane_wp for lane_wp in lane_candidates
                if (int(lane_wp.road_id), int(lane_wp.lane_id)) not in ambulance_route_lanes
            ]
            if non_route_candidates:
                lane_candidates = non_route_candidates

        best_location = None
        best_score = -1e9
        for lane_wp in lane_candidates:
            ahead = lane_wp.next(lookahead)
            target_wp = ahead[0] if ahead else lane_wp
            target_loc = target_wp.transform.location

            corridor_distance = self._min_distance_to_corridor(target_loc, corridor_points)
            lane_change_bonus = 10.0 if lane_wp.lane_id != current_wp.lane_id else 0.0
            non_route_bonus = 14.0 if (int(lane_wp.road_id), int(lane_wp.lane_id)) not in ambulance_route_lanes else 0.0

            # Prefer lanes farther from ambulance corridor; bias toward changing lane when possible.
            score = corridor_distance + lane_change_bonus + non_route_bonus
            if score > best_score:
                best_score = score
                best_location = target_loc

        return best_location

    def _collect_type_c_detour_candidates(self, current_wp, target_destination, max_candidates):
        """Collect immediate turn/lane candidates for Type C rerouting.

        Priority is near-term maneuvers (next turns/lane changes), then broader spawn-point fallbacks.
        """
        candidates = []
        seen = set()

        def _add_location(loc):
            if loc is None:
                return
            key = (round(float(loc.x), 1), round(float(loc.y), 1), round(float(loc.z), 1))
            if key in seen:
                return
            seen.add(key)
            candidates.append(loc)

        # Immediate candidates from upcoming route points and adjacent lanes.
        for step in (12.0, 20.0, 30.0, 40.0):
            next_wps = current_wp.next(step) if current_wp else []
            for next_wp in next_wps:
                _add_location(next_wp.transform.location)
                for lane_wp in (next_wp.get_left_lane(), next_wp.get_right_lane()):
                    if lane_wp is None:
                        continue
                    if lane_wp.lane_type != carla.LaneType.Driving:
                        continue
                    if lane_wp.lane_id * next_wp.lane_id < 0:
                        continue
                    ahead = lane_wp.next(10.0)
                    target_wp = ahead[0] if ahead else lane_wp
                    _add_location(target_wp.transform.location)

        # Fallback candidates from spawn points, sorted by proximity for quicker turn commitment.
        spawn_points = self.map.get_spawn_points()
        if spawn_points:
            current_loc = self.vehicle.get_location()
            sorted_points = sorted(spawn_points, key=lambda sp: current_loc.distance(sp.location))
            for sp in sorted_points[:max_candidates]:
                _add_location(sp.location)

        # Ensure destination is always considered as final fallback.
        _add_location(target_destination)
        return candidates[:max_candidates]

    def _plan_type_c_avoidance_route(self, timestamp, force=False):
        """Type C strategy: reroute to same destination while avoiding ambulance route lanes."""
        if not (self.use_v2x and not self.use_lidar and self.detected_ambulance):
            return False

        target_destination = self.original_destination or self.destination
        if target_destination is None:
            return False

        reroute_cooldown = max(0.1, float(globals().get('TYPE_C_REROUTE_COOLDOWN', 4.0)))
        if (not force) and ((timestamp - self.last_type_c_reroute) < reroute_cooldown):
            return False

        ambulance_lane_keys = self._get_ambulance_route_lane_keys()
        if not ambulance_lane_keys:
            return False

        current_location = self.vehicle.get_location()
        direct_trace = self._trace_route(current_location, target_destination)
        if not direct_trace:
            return False

        direct_overlap = self._route_lane_keys(direct_trace) & ambulance_lane_keys
        if (not force) and (not direct_overlap):
            return False

        current_wp = self._get_current_driving_waypoint()
        max_candidates = max(20, int(globals().get('TYPE_C_MAX_DETOUR_CANDIDATES', 30)))
        detour_candidates = self._collect_type_c_detour_candidates(current_wp, target_destination, max_candidates)

        best_plan = None
        best_length = float('inf')

        for detour_loc in detour_candidates:
            if current_location.distance(detour_loc) < 18.0:
                continue

            leg_one = self._trace_route(current_location, detour_loc)
            if not leg_one or (self._route_lane_keys(leg_one) & ambulance_lane_keys):
                continue

            leg_two = self._trace_route(detour_loc, target_destination)
            if not leg_two or (self._route_lane_keys(leg_two) & ambulance_lane_keys):
                continue

            combined = leg_one + leg_two
            # Prefer shortest feasible non-overlapping route.
            route_length = float(len(combined))
            if route_length < best_length:
                best_length = route_length
                best_plan = combined

        if best_plan is not None:
            try:
                self.agent.set_global_plan(best_plan, stop_waypoint_creation=True, clean_queue=True)
                self.destination = target_destination
                self.last_type_c_reroute = timestamp
                self.type_c_reroute_until = timestamp + 1.5
                self.clearing_path = False
                self.type_c_cached_overlap = False
                self.type_c_force_reroute = False
                return True
            except Exception:
                return False

        return False
    
    def _predict_ambulance_trajectory(self, time_horizon=5.0):
        """
        Predict ambulance trajectory for next N seconds
        
        Args:
            time_horizon: Time in seconds to predict ahead
            
        Returns:
            List of predicted locations
        """
        if not self.detected_ambulance:
            return []
        
        ambulance_location = self.detected_ambulance['location']
        ambulance_velocity = self.detected_ambulance['velocity']
        
        # Simple linear prediction (can be enhanced with road following)
        predictions = []
        dt = 0.5  # 0.5 second intervals
        num_steps = int(time_horizon / dt)
        
        for i in range(num_steps):
            t = (i + 1) * dt
            predicted_location = carla.Location(
                x=ambulance_location.x + ambulance_velocity.x * t,
                y=ambulance_location.y + ambulance_velocity.y * t,
                z=ambulance_location.z
            )
            predictions.append(predicted_location)
        
        return predictions
    
    def _is_in_ambulance_path(self, predictions, threshold=15.0):
        """
        Check if vehicle is in predicted ambulance path
        
        Args:
            predictions: List of predicted ambulance locations
            threshold: Distance threshold in meters
            
        Returns:
            bool, closest distance to path
        """
        if not predictions:
            return False, float('inf')
        
        vehicle_location = self.vehicle.get_location()
        min_distance = float('inf')
        
        for pred_location in predictions:
            distance = vehicle_location.distance(pred_location)
            min_distance = min(min_distance, distance)
        
        return min_distance < threshold, min_distance
    
    def _check_intersection(self):
        """
        Check if vehicle is currently at or approaching an intersection
        
        Returns:
            bool: True if at intersection
        """
        try:
            vehicle_location = self.vehicle.get_location()
            waypoint = self.map.get_waypoint(vehicle_location, project_to_road=True)
            
            if waypoint:
                # Check if waypoint is at junction
                return waypoint.is_junction
        except:
            pass
        
        return False
    
    def _determine_pull_direction(self):
        """
        Determine which direction to pull over (right/left based on road type)
        
        Returns:
            float: Lane offset (positive=right, negative=left)
        """
        try:
            vehicle_location = self.vehicle.get_location()
            waypoint = self.map.get_waypoint(vehicle_location, project_to_road=True)
            
            if waypoint:
                # Get road info
                lane_id = waypoint.lane_id
                
                # Standard rule: pull to the right (positive lane offset)
                # In most countries, emergency vehicles pass on the left
                pull_distance = 1.5  # meters to pull right (reduced to stay within lane)
                
                # Check if we can pull right (check for right lane)
                if waypoint.get_right_lane():
                    return pull_distance
                else:
                    # No right lane, pull left
                    return -pull_distance * 0.8  # Even more conservative when pulling left
        except:
            pass
        
        # Default: pull right minimally
        return 1.5
    
    def _calculate_urgency_level(self, distance_to_ambulance, in_path):
        """
        Calculate urgency level based on distance and path intersection
        
        Args:
            distance_to_ambulance: Distance in meters
            in_path: Whether vehicle is in ambulance's predicted path
            
        Returns:
            int: 0=normal, 1=aware (100-150m), 2=clearing (30-100m), 3=critical (<30m)
        """
        # If already in the predicted corridor, start clearing much earlier.
        if in_path and distance_to_ambulance > 30:
            return 2
        if in_path:
            return 3

        if not in_path and distance_to_ambulance > 100:
            return 0  # Normal driving
        elif distance_to_ambulance > 100:
            return 1  # Aware of ambulance
        elif distance_to_ambulance > 30:
            return 2  # Actively clearing path
        else:
            return 3  # Critical - ambulance very close
    
    def detect_v2x_ambulance(self, ambulance_broadcast, timestamp=None):
        """
        Enhanced V2X detection with trajectory prediction
        
        Args:
            ambulance_broadcast: Dict with ambulance info from AmbulanceController.get_v2x_broadcast()
            timestamp: Optional simulation timestamp (kept for caller compatibility)
        """
        if not ambulance_broadcast:
            self._reset_emergency_state()
            return
        
        vehicle_location = self.vehicle.get_location()
        ambulance_location = ambulance_broadcast['location']
        distance_to_ambulance = vehicle_location.distance(ambulance_location)

        self._ensure_lidar(distance_to_ambulance=distance_to_ambulance)
        
        # Check if ambulance has passed (is behind the vehicle)
        vehicle_transform = self.vehicle.get_transform()
        forward_vector = vehicle_transform.get_forward_vector()
        
        to_ambulance = carla.Vector3D(
            x=ambulance_location.x - vehicle_location.x,
            y=ambulance_location.y - vehicle_location.y,
            z=0
        )
        
        dot_product = forward_vector.x * to_ambulance.x + forward_vector.y * to_ambulance.y
        ambulance_behind = dot_product < 0  # Negative means behind
        
        # If ambulance passed, reset to normal driving
        if ambulance_behind and self.clearing_path:
            self._reset_emergency_state()
            return

        # Check if within V2X detection range
        if distance_to_ambulance > ambulance_broadcast['v2x_range']:
            self._reset_emergency_state()
            self._ensure_lidar(distance_to_ambulance=None)
            return
        
        self.detected_ambulance = ambulance_broadcast

        # Predict ambulance trajectory and merge with its planned route corridor.
        predictions = self._predict_ambulance_trajectory(time_horizon=5.0)
        self.emergency_state['predicted_ambulance_path'] = predictions

        if not self.use_v2x:
            # Type B (lidar only): switch lanes only when rear lidar and geometry confirm ambulance behind nearby.
            should_clear = self._should_type_b_clear_now(distance_to_ambulance)
            self.emergency_state['urgency_level'] = 2 if should_clear else 0
            self.clearing_path = should_clear
            self.idm.set_emergency_mode(should_clear)
            return

        # Type C (v2x only): awareness only. No evasive steering/rerouting.
        # Vehicles keep following their normal route even when ambulance is detected.
        urgency = 1
        self.emergency_state['urgency_level'] = urgency
        self.emergency_state['at_intersection'] = self._check_intersection()
        self.emergency_state['target_lane_offset'] = 0.0
        self.clearing_path = False
        self.type_c_force_reroute = False
        self.idm.set_emergency_mode(False)
    
    def _calculate_clearance_action(self):
        """
        Calculate clearance action using rule-based protocol
        
        Returns:
            dict: Action with 'type', 'target_speed', 'lane_offset'
        """
        if not self.detected_ambulance:
            return {'type': 'normal', 'target_speed': self.initial_speed, 'lane_offset': 0.0}
        
        urgency = self.emergency_state['urgency_level']
        at_intersection = self.emergency_state['at_intersection']
        lane_offset = self.emergency_state['target_lane_offset']
        
        vehicle_location = self.vehicle.get_location()
        ambulance_location = self.detected_ambulance['location']
        distance = vehicle_location.distance(ambulance_location)
        
        # Rule 1: At intersection - stop and let ambulance pass
        if at_intersection and urgency >= 2:
            if distance < 50.0:
                return {
                    'type': 'stop_at_intersection',
                    'target_speed': 0.0,
                    'lane_offset': 0.0,
                    'should_stop': True
                }
            else:
                return {
                    'type': 'clear_intersection',
                    'target_speed': self.initial_speed * 1.5,
                    'lane_offset': 0.0,
                    'should_stop': False
                }
        
        # Rule 2: Critical distance - pull over and stop/slow
        if urgency == 3:
            return {
                'type': 'pull_over_critical',
                'target_speed': self.initial_speed * 0.3,  # Slow down significantly
                'lane_offset': lane_offset,
                'should_stop': False
            }
        
        # Rule 3: Clearing mode - pull to side and maintain speed
        if urgency == 2:
            return {
                'type': 'pull_over_clearing',
                'target_speed': self.initial_speed * 0.8,  # Slight speed reduction
                'lane_offset': lane_offset,
                'should_stop': False
            }
        
        # Rule 4: Aware mode - prepare to clear
        if urgency == 1:
            return {
                'type': 'prepare_to_clear',
                'target_speed': self.initial_speed,
                'lane_offset': lane_offset * 0.5,  # Partial lane change
                'should_stop': False
            }
        
        # Normal driving
        return {
            'type': 'normal',
            'target_speed': self.initial_speed,
            'lane_offset': 0.0,
            'should_stop': False
        }
    
    def _calculate_lane_offset_waypoint(self, lane_offset):
        """
        Calculate target waypoint with lane offset
        
        Args:
            lane_offset: Lateral offset in meters (positive=right)
            
        Returns:
            carla.Location or None
        """
        try:
            vehicle_location = self.vehicle.get_location()
            vehicle_transform = self.vehicle.get_transform()
            waypoint = self.map.get_waypoint(vehicle_location, project_to_road=True)
            
            if waypoint:
                # Calculate perpendicular direction (right side of vehicle)
                forward_vector = vehicle_transform.get_forward_vector()
                right_vector = carla.Vector3D(
                    x=-forward_vector.y,
                    y=forward_vector.x,
                    z=0.0
                )
                
                # Calculate target location with offset
                target_location = carla.Location(
                    x=vehicle_location.x + right_vector.x * lane_offset,
                    y=vehicle_location.y + right_vector.y * lane_offset,
                    z=vehicle_location.z
                )
                
                # Project to road to ensure valid location
                target_waypoint = self.map.get_waypoint(target_location, project_to_road=True)
                if target_waypoint:
                    return target_waypoint.transform.location
        except Exception as e:
            if DEBUG_PRINT:
                print(f"[VEHICLE] Lane offset calculation error: {e}")
        
        return None
    
    def update(self, timestamp):
        """
        Enhanced update with hybrid IDM + Rule-Based + V2X approach
        
        Args:
            timestamp: Current simulation timestamp
        """
        if not self.moving:
            self._set_random_destination()
            if not self.moving:
                return
        
        # Check if we've reached destination
        if self.agent.done():
            self._set_random_destination()
            self.clearing_path = False
            self.detected_ambulance = None
            self.emergency_state['urgency_level'] = 0
            self.idm.set_emergency_mode(False)
        elif self.destination:
            current_location = self.vehicle.get_location()
            if current_location.distance(self.destination) < 5.0:
                self._set_random_destination()

        distance_to_ambulance = None
        if self.detected_ambulance:
            try:
                distance_to_ambulance = self.vehicle.get_location().distance(self.detected_ambulance['location'])
            except Exception:
                distance_to_ambulance = None
        
        # Apply vehicle control
        try:
            if self.clearing_path and self.detected_ambulance:
                if not self.use_v2x:
                    self._plan_left_lane_evasion(timestamp)
                elif self.use_v2x and not self.use_lidar:
                    self._plan_type_c_avoidance_route(timestamp, force=self.type_c_force_reroute)

            if self.agent._target_speed != self.initial_speed:
                self.agent.set_target_speed(self.initial_speed)
            
            # Run agent's decision and apply control
            control = self.agent.run_step()
            control = self._apply_lidar_avoidance(control)

            # Keep behavior-controlled vehicles from aggressive steering that can leave the lane.
            max_steer_norm = 0.60
            control.steer = max(-max_steer_norm, min(max_steer_norm, control.steer))

            # Final safety gate: hard-correct drift if the actor gets pushed off-road.
            control = self._enforce_strict_road_lock(timestamp, control)

            self.vehicle.apply_control(control)
            self._complete_type_b_lane_switch_if_ready()
            self._restore_original_destination_if_needed()
            
        except Exception as e:
            if DEBUG_PRINT:
                print(f"[VEHICLE {self.vehicle.id}] Control error: {e}")
    
    def get_state(self):
        """Get vehicle state information"""
        location = self.vehicle.get_location()
        velocity = self.vehicle.get_velocity()
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        return {
            'vehicle_id': self.vehicle.id,
            'location': location,
            'speed': speed,
            'clearing_path': self.clearing_path,
            'ambulance_detected': self.detected_ambulance is not None,
            'urgency_level': self.emergency_state['urgency_level'],
            'at_intersection': self.emergency_state['at_intersection']
        }

    def cleanup(self):
        if self.lidar_sensor:
            try:
                self.lidar_sensor.destroy()
            except Exception:
                pass
            self.lidar_sensor = None
