"""
Baseline Vehicle Behavior - NPCs with proximity-based ambulance detection
Unlike V2X, vehicles detect ambulance through proximity (simulating visual/audio detection)
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'PythonAPI', 'carla'))

import carla
import math
import numpy as np
from agents.navigation.basic_agent import BasicAgent
from config import *


# Baseline proximity detection range (shorter than V2X)
BASELINE_DETECTION_RANGE = 60.0  # meters - simulating hearing/seeing ambulance


class BaselineVehicleBehavior:
    """Vehicle that responds to nearby ambulance through proximity detection (no V2X)"""
    
    def __init__(self, vehicle, world_map, world=None):
        """
        Initialize baseline vehicle behavior with proximity detection
        
        Args:
            vehicle: CARLA vehicle actor
            world_map: CARLA map object
            world: CARLA world object (for optional lidar)
        """
        self.vehicle = vehicle
        self.map = world_map
        self.world = world
        self.agent = BasicAgent(vehicle, target_speed=REGULAR_VEHICLE_SPEED)
        self.destination = None
        self.moving = False
        self.ambulance_nearby = False
        self.clearing_path = False
        self.initial_speed = REGULAR_VEHICLE_SPEED

        # Optional lidar-based collision avoidance
        self.lidar_sensor = None
        self.obstacle_ahead = False
        self.obstacle_distance = float('inf')
        if NPC_LIDAR_ENABLED and self.world and NPC_LIDAR_MODE == "all":
            self._setup_lidar_sensor()
        
        # Set random destination
        self._set_random_destination()
    
    def _set_random_destination(self):
        """Set a random waypoint as destination"""
        try:
            waypoints = self.map.get_spawn_points()
            if waypoints:
                import random
                dest = random.choice(waypoints).location
                self.agent.set_destination(dest)
                self.destination = dest
                self.moving = True
        except:
            pass

    def _should_enable_lidar(self, distance_to_ambulance=None):
        if not NPC_LIDAR_ENABLED or not self.world:
            return False
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
                print(f"[BASELINE VEHICLE {self.vehicle.id}] Lidar setup failed: {e}")
            self.lidar_sensor = None

    def _ensure_lidar(self, distance_to_ambulance=None):
        if self.lidar_sensor:
            return
        if self._should_enable_lidar(distance_to_ambulance):
            self._setup_lidar_sensor()

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
        except Exception:
            self.obstacle_ahead = False
            self.obstacle_distance = float('inf')

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
            lane_offset = self._determine_pull_direction()
            lateral_steering = np.clip(lane_offset / 2.0, -0.3, 0.3)
            control.steer = np.clip(control.steer + lateral_steering, -1.0, 1.0)

        return control
    
    def _check_intersection(self):
        """Check if vehicle is at an intersection"""
        try:
            vehicle_location = self.vehicle.get_location()
            waypoint = self.map.get_waypoint(vehicle_location, project_to_road=True)
            if waypoint:
                return waypoint.is_junction
        except:
            pass
        return False
    
    def _is_ambulance_behind(self, ambulance_location):
        """
        Check if ambulance is approaching from behind
        
        Args:
            ambulance_location: Location of ambulance
            
        Returns:
            bool: True if ambulance is behind vehicle
        """
        vehicle_transform = self.vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        
        # Vector from vehicle to ambulance
        to_ambulance = carla.Vector3D(
            x=ambulance_location.x - vehicle_location.x,
            y=ambulance_location.y - vehicle_location.y,
            z=0.0
        )
        
        # Vehicle's forward direction
        forward = vehicle_transform.get_forward_vector()
        
        # Dot product to check if ambulance is behind (negative dot product)
        dot_product = forward.x * to_ambulance.x + forward.y * to_ambulance.y
        
        return dot_product < 0  # Ambulance is behind if dot product is negative
    
    def _determine_pull_direction(self):
        """
        Determine which direction to pull (right by default)
        
        Returns:
            float: Lane offset (positive=right, negative=left)
        """
        try:
            vehicle_location = self.vehicle.get_location()
            waypoint = self.map.get_waypoint(vehicle_location, project_to_road=True)
            
            if waypoint:
                # Try to pull right
                if waypoint.get_right_lane():
                    return 3.0  # Pull 3m to the right
                else:
                    # No right lane, pull left
                    return -3.0
        except:
            pass
        
        # Default: pull right
        return 3.0
    
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
                print(f"[VEHICLE] Lane offset error: {e}")
        
        return None
    
    def detect_ambulance_proximity(self, ambulance_location):
        """
        Detect ambulance through proximity (simulating visual/audio detection)
        
        Args:
            ambulance_location: Location of the ambulance
        """
        if not ambulance_location:
            self.ambulance_nearby = False
            self.clearing_path = False
            return
        
        vehicle_location = self.vehicle.get_location()
        distance = vehicle_location.distance(ambulance_location)

        self._ensure_lidar(distance_to_ambulance=distance)
        
        # Check if within detection range (simulating hearing siren)
        if distance <= BASELINE_DETECTION_RANGE:
            # Check if ambulance is behind us (coming up from behind)
            is_behind = self._is_ambulance_behind(ambulance_location)
            
            if is_behind:
                if not self.clearing_path:
                    # Start clearing path
                    self.clearing_path = True
                    if DEBUG_PRINT:
                        print(f"[BASELINE VEHICLE {self.vehicle.id}] Detected ambulance behind at {distance:.1f}m - clearing path")
                
                self.ambulance_nearby = True
            else:
                # Ambulance is ahead or to the side, don't need to clear
                self.ambulance_nearby = False
                # Reset clearing if ambulance is no longer behind us
                if self.clearing_path:
                    self.clearing_path = False
                    if DEBUG_PRINT:
                        print(f"[BASELINE VEHICLE {self.vehicle.id}] Ambulance no longer behind - resuming normal")
        else:
            # Ambulance out of range - reset and resume normal driving
            self.ambulance_nearby = False
            if self.clearing_path:
                self.clearing_path = False
                if DEBUG_PRINT:
                    print(f"[BASELINE VEHICLE {self.vehicle.id}] Ambulance passed - resuming normal driving")
    
    def update(self, timestamp):
        """
        Update vehicle behavior each frame
        
        Args:
            timestamp: Current simulation timestamp
        """
        if not self.moving:
            self._set_random_destination()
            if not self.moving:
                return
        
        # Check if we've reached destination - set new one
        if self.agent.done():
            self._set_random_destination()
            # Don't reset clearing_path here - let it be controlled by proximity detection
        elif self.destination:
            current_location = self.vehicle.get_location()
            if current_location.distance(self.destination) < 5.0:
                self._set_random_destination()
        
        # Apply vehicle control
        try:
            if self.clearing_path and self.ambulance_nearby:
                # Check if at intersection
                at_intersection = self._check_intersection()
                
                if at_intersection:
                    # At intersection - slow down significantly to let ambulance pass
                    # Don't stop completely as it might block traffic
                    self.agent.set_target_speed(self.initial_speed * 0.2)
                else:
                    # Not at intersection - slow down AND shift lane to let ambulance pass
                    self.agent.set_target_speed(self.initial_speed * 0.5)
                    
                    # Apply lane offset (lateral shift to clear path)
                    lane_offset = self._determine_pull_direction()
                    control = self.agent.run_step()
                    control = self._apply_lidar_avoidance(control)
                    
                    # Add lateral steering to shift lane
                    # Scale steering proportional to desired offset
                    lateral_steering = np.clip(lane_offset / 2.0, -0.3, 0.3)
                    control.steer = np.clip(control.steer + lateral_steering, -1.0, 1.0)
                    
                    self.vehicle.apply_control(control)
                    if DEBUG_PRINT:
                        print(f"[BASELINE VEHICLE {self.vehicle.id}] Clearing path: offset={lane_offset:.2f}m, steer={control.steer:.2f}")
                    return
            else:
                # Normal driving - ensure normal speed
                if self.agent._target_speed != self.initial_speed:
                    self.agent.set_target_speed(self.initial_speed)
            
            # Run agent's decision and apply control
            control = self.agent.run_step()
            control = self._apply_lidar_avoidance(control)
            self.vehicle.apply_control(control)
            
        except Exception as e:
            if DEBUG_PRINT:
                print(f"[BASELINE VEHICLE {self.vehicle.id}] Control error: {e}")
    
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
            'ambulance_nearby': self.ambulance_nearby
        }

    def cleanup(self):
        if self.lidar_sensor:
            try:
                self.lidar_sensor.destroy()
            except Exception:
                pass
            self.lidar_sensor = None
