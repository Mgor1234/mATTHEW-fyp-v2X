"""
Real-time Minimap Visualization for V2X Ambulance Simulation
Displays a 2D bird-eye view of the town map with vehicle positions and types.

Vehicle Color Coding:
- WHITE: Ambulance
- RED: Type A vehicles (CARLA autopilot only, no V2X/LIDAR)
- YELLOW: Type B vehicles (LIDAR autonomous driving, no V2X)
- GREEN: Type C vehicles (LIDAR autonomous driving + V2X)
"""

from __future__ import annotations

import threading
import queue
import time
import math
from typing import Dict, List, Tuple, Optional

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False

try:
    from PIL import Image, ImageDraw
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'PythonAPI', 'carla'))

try:
    import carla
    CARLA_AVAILABLE = True
except ImportError:
    CARLA_AVAILABLE = False


class MinimapData:
    """Container for vehicle data to render on minimap"""
    
    def __init__(self, vehicle_id: int, position: Tuple[float, float],
                 vehicle_type: str, rotation: float = 0.0):
        """
        Args:
            vehicle_id: Unique vehicle ID
            position: (x, y) world position
            vehicle_type: 'ambulance', 'A', 'B', or 'C'
            rotation: Yaw angle in degrees
        """
        self.vehicle_id = vehicle_id
        self.position = position
        self.vehicle_type = vehicle_type
        self.rotation = rotation
        self.last_update_time = time.time()


class MinimapRenderer:
    """Base class for minimap renderers"""
    
    # Color palette
    COLORS = {
        'ambulance': (255, 255, 255),      # White
        'A': (255, 0, 0),                   # Red
        'B': (255, 255, 0),                 # Yellow
        'C': (0, 255, 0),                   # Green
        'background': (40, 40, 40),         # Dark gray
        'grid': (80, 80, 80),               # Medium gray
        'map_bounds': (200, 200, 200),      # Light gray
    }

    STALE_TIMEOUT_SECONDS = 3.0
    
    def __init__(self, map_name: str, width: int = 400, height: int = 400,
                 pixels_per_meter: float = 1.0):
        """
        Initialize minimap renderer
        
        Args:
            map_name: CARLA map name (e.g., 'Town01')
            width: Minimap width in pixels
            height: Minimap height in pixels
            pixels_per_meter: Scale factor
        """
        self.map_name = map_name
        self.width = width
        self.height = height
        self.pixels_per_meter = pixels_per_meter
        
        # World bounds (will be updated when connecting to CARLA)
        self.world_min_x = -100
        self.world_max_x = 100
        self.world_min_y = -100
        self.world_max_y = 100
        
        self.road_segments: List[List[Tuple[float, float]]] = []
        self.route_path: List[Tuple[float, float]] = []
        self.vehicles: Dict[int, MinimapData] = {}
        self.lock = threading.Lock()
    
    def set_world_bounds(self, min_x: float, max_x: float, min_y: float, max_y: float):
        """Set the world coordinate bounds for proper scaling"""
        with self.lock:
            self.world_min_x = min_x
            self.world_max_x = max_x
            self.world_min_y = min_y
            self.world_max_y = max_y

    def set_road_geometry(self, road_segments: List[List[Tuple[float, float]]]):
        """Set cached road centerline segments for rendering."""
        with self.lock:
            self.road_segments = road_segments or []

    def set_route_path(self, route_path: List[Tuple[float, float]]):
        """Set the designated ambulance route for rendering."""
        with self.lock:
            self.route_path = route_path or []
    
    def update_vehicle(self, vehicle_id: int, position: Tuple[float, float],
                      vehicle_type: str, rotation: float = 0.0):
        """Update vehicle position and type"""
        with self.lock:
            self.vehicles[vehicle_id] = MinimapData(vehicle_id, position, vehicle_type, rotation)
    
    def remove_vehicle(self, vehicle_id: int):
        """Remove vehicle from minimap"""
        with self.lock:
            self.vehicles.pop(vehicle_id, None)
    
    def world_to_screen(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert world coordinates to screen coordinates"""
        # Normalize world coordinates to 0-1 range.
        if self.world_max_x != self.world_min_x:
            norm_x = (world_x - self.world_min_x) / (self.world_max_x - self.world_min_x)
            # CARLA world orientation for this project requires X flip for map-facing view.
            screen_x = (1.0 - norm_x) * (self.width - 1)
        else:
            screen_x = (self.width - 1) / 2
        
        if self.world_max_y != self.world_min_y:
            norm_y = (world_y - self.world_min_y) / (self.world_max_y - self.world_min_y)
            screen_y = (1.0 - norm_y) * (self.height - 1)
        else:
            screen_y = (self.height - 1) / 2

        screen_x = max(0, min(self.width - 1, screen_x))
        screen_y = max(0, min(self.height - 1, screen_y))

        return int(round(screen_x)), int(round(screen_y))

    def _expand_bounds_to_include(self, world_x: float, world_y: float):
        """Expand bounds if a vehicle leaves current extents to avoid edge-sticking."""
        margin = 20.0
        if world_x < self.world_min_x:
            self.world_min_x = world_x - margin
        if world_x > self.world_max_x:
            self.world_max_x = world_x + margin
        if world_y < self.world_min_y:
            self.world_min_y = world_y - margin
        if world_y > self.world_max_y:
            self.world_max_y = world_y + margin
        
    def _prune_stale_vehicles_locked(self):
        """Drop vehicles that have not received updates recently."""
        now = time.time()
        stale_ids = [
            vehicle_id
            for vehicle_id, vehicle_data in self.vehicles.items()
            if (now - vehicle_data.last_update_time) > self.STALE_TIMEOUT_SECONDS
        ]
        for vehicle_id in stale_ids:
            self.vehicles.pop(vehicle_id, None)
    
    def render(self) -> Optional[object]:
        """Render minimap. Override in subclasses. Returns image/surface."""
        raise NotImplementedError
    
    def get_vehicle_color(self, vehicle_type: str) -> Tuple[int, int, int]:
        """Get RGB color for vehicle type"""
        return self.COLORS.get(vehicle_type, (200, 200, 200))

    def _world_points_to_screen(self, points: List[Tuple[float, float]]) -> List[Tuple[int, int]]:
        """Convert a list of world points to on-screen points."""
        return [self.world_to_screen(x, y) for x, y in points]

    def _draw_road_network_pygame(self, surface: 'pygame.Surface') -> None:
        road_color = (72, 76, 84)
        road_highlight = (106, 112, 124)
        with self.lock:
            segments = list(self.road_segments)

        for segment in segments:
            if len(segment) < 2:
                continue
            points = self._world_points_to_screen(segment)
            pygame.draw.lines(surface, road_color, False, points, 9)
            pygame.draw.lines(surface, road_highlight, False, points, 2)

    def _draw_route_path_pygame(self, surface: 'pygame.Surface') -> None:
        route_outer = (89, 187, 255)
        route_inner = (170, 225, 255)
        with self.lock:
            route_path = list(self.route_path)

        if len(route_path) < 2:
            return

        points = self._world_points_to_screen(route_path)
        pygame.draw.lines(surface, route_outer, False, points, 7)
        pygame.draw.lines(surface, route_inner, False, points, 3)

    def _draw_road_network_pil(self, draw: 'ImageDraw.ImageDraw') -> None:
        road_color = (72, 76, 84)
        road_highlight = (106, 112, 124)
        with self.lock:
            segments = list(self.road_segments)

        for segment in segments:
            if len(segment) < 2:
                continue
            points = self._world_points_to_screen(segment)
            draw.line(points, fill=road_color, width=9)
            draw.line(points, fill=road_highlight, width=2)

    def _draw_route_path_pil(self, draw: 'ImageDraw.ImageDraw') -> None:
        route_outer = (89, 187, 255)
        route_inner = (170, 225, 255)
        with self.lock:
            route_path = list(self.route_path)

        if len(route_path) < 2:
            return

        points = self._world_points_to_screen(route_path)
        draw.line(points, fill=route_outer, width=7)
        draw.line(points, fill=route_inner, width=3)


class PyGameMinimapRenderer(MinimapRenderer):
    """Pygame-based minimap renderer with live window"""
    
    def __init__(self, map_name: str, width: int = 400, height: int = 400):
        super().__init__(map_name, width, height)
        self.screen = None
        self.clock = None
        self.running = True
        self.fps = 30
        
        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption(f"V2X Simulation Minimap - {map_name}")
        self.clock = pygame.time.Clock()
    
    def render(self) -> Optional[pygame.Surface]:
        """Render minimap and update display"""
        # Create surface
        surface = pygame.Surface((self.width, self.height))
        surface.fill(self.COLORS['background'])

        # Draw road network first so vehicles render on top.
        self._draw_road_network_pygame(surface)
        self._draw_route_path_pygame(surface)
        
        # Draw grid
        grid_spacing = 50
        grid_color = self.COLORS['grid']
        for x in range(0, self.width, grid_spacing):
            pygame.draw.line(surface, grid_color, (x, 0), (x, self.height), 1)
        for y in range(0, self.height, grid_spacing):
            pygame.draw.line(surface, grid_color, (0, y), (self.width, y), 1)
        
        # Draw border
        pygame.draw.rect(surface, self.COLORS['map_bounds'], 
                        (0, 0, self.width, self.height), 2)
        
        # Draw vehicles
        with self.lock:
            self._prune_stale_vehicles_locked()
            for vehicle_data in self.vehicles.values():
                screen_x, screen_y = self.world_to_screen(
                    vehicle_data.position[0], vehicle_data.position[1])
                
                color = self.get_vehicle_color(vehicle_data.vehicle_type)
                
                # Draw vehicle as circle
                radius = 4 if vehicle_data.vehicle_type == 'ambulance' else 3
                pygame.draw.circle(surface, color, (screen_x, screen_y), radius)
                
                # Draw direction indicator (line pointing in direction of travel)
                if vehicle_data.rotation != 0:
                    angle_rad = math.radians(vehicle_data.rotation)
                    end_x = screen_x - 5 * math.cos(angle_rad)
                    end_y = screen_y - 5 * math.sin(angle_rad)
                    pygame.draw.line(surface, color, (screen_x, screen_y), 
                                   (end_x, end_y), 1)
        
        # Draw legend
        legend_y = 5
        legend_x = 5
        font = pygame.font.Font(None, 20)
        
        legend_items = [
            ('ambulance', 'Ambulance'),
            ('A', 'Type A (Autopilot)'),
            ('B', 'Type B (LIDAR)'),
            ('C', 'Type C (V2X+LIDAR)'),
        ]
        
        for vehicle_type, label in legend_items:
            color = self.get_vehicle_color(vehicle_type)
            pygame.draw.circle(surface, color, (legend_x + 5, legend_y + 5), 3)
            text = font.render(label, True, (255, 255, 255))
            surface.blit(text, (legend_x + 15, legend_y))
            legend_y += 20
        
        # Update display
        self.screen.blit(surface, (0, 0))
        pygame.display.flip()
        
        return surface
    
    def handle_events(self):
        """Handle pygame events (window close, etc)"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
    
    def quit(self):
        """Clean up pygame resources"""
        pygame.quit()
        self.running = False


class PILMinimapRenderer(MinimapRenderer):
    """PIL-based minimap renderer (static image) - for headless rendering or image export"""
    
    def render(self) -> Optional[Image.Image]:
        """Render minimap as PIL Image"""
        # Create image
        image = Image.new('RGB', (self.width, self.height), self.COLORS['background'])
        draw = ImageDraw.Draw(image)

        # Draw road network first so vehicles render on top.
        self._draw_road_network_pil(draw)
        self._draw_route_path_pil(draw)
        
        # Draw grid
        grid_spacing = 50
        for x in range(0, self.width, grid_spacing):
            draw.line([(x, 0), (x, self.height)], fill=self.COLORS['grid'], width=1)
        for y in range(0, self.height, grid_spacing):
            draw.line([(0, y), (self.width, y)], fill=self.COLORS['grid'], width=1)
        
        # Draw border
        draw.rectangle([(0, 0), (self.width - 1, self.height - 1)], 
                      outline=self.COLORS['map_bounds'], width=2)
        
        # Draw vehicles
        with self.lock:
            self._prune_stale_vehicles_locked()
            for vehicle_data in self.vehicles.values():
                screen_x, screen_y = self.world_to_screen(
                    vehicle_data.position[0], vehicle_data.position[1])
                
                color = self.get_vehicle_color(vehicle_data.vehicle_type)
                
                # Draw vehicle as circle
                radius = 4 if vehicle_data.vehicle_type == 'ambulance' else 3
                draw.ellipse([(screen_x - radius, screen_y - radius),
                            (screen_x + radius, screen_y + radius)],
                           fill=color, outline=color)
                
                # Draw direction indicator
                if vehicle_data.rotation != 0:
                    angle_rad = math.radians(vehicle_data.rotation)
                    end_x = screen_x - 5 * math.cos(angle_rad)
                    end_y = screen_y - 5 * math.sin(angle_rad)
                    draw.line([(screen_x, screen_y), (end_x, end_y)],
                            fill=color, width=1)
        
        # Note: Legend drawing with PIL is more complex; skipped for now
        return image


class MinimapUpdateQueue:
    """Thread-safe queue for minimap updates"""
    
    def __init__(self, max_size: int = 1000):
        self.queue = queue.Queue(maxsize=max_size)
    
    def add_update(self, vehicle_id: int, position: Tuple[float, float],
                  vehicle_type: str, rotation: float = 0.0):
        """Add vehicle update to queue"""
        try:
            self.queue.put_nowait((vehicle_id, position, vehicle_type, rotation))
        except queue.Full:
            pass  # Drop update if queue is full
    
    def get_all_updates(self) -> List[Tuple[int, Tuple[float, float], str, float]]:
        """Get all pending updates"""
        updates = []
        while not self.queue.empty():
            try:
                updates.append(self.queue.get_nowait())
            except queue.Empty:
                break
        return updates


class RealTimeMinimapWindow(threading.Thread):
    """Real-time minimap display running in separate thread"""
    
    def __init__(self, map_name: str, update_queue: MinimapUpdateQueue,
                 width: int = 400, height: int = 400, fps: int = 30,
                 road_segments: Optional[List[List[Tuple[float, float]]]] = None):
        super().__init__(daemon=True)
        self.map_name = map_name
        self.update_queue = update_queue
        self.width = width
        self.height = height
        self.fps = fps
        self.running = False
        self.renderer = None
        self._pending_world_bounds = None
        self._pending_road_segments = road_segments
        self._pending_route_path = None

        # Pick backend now, but create renderer inside run() so all display operations
        # happen on the same thread (important for stable window behavior on Windows).
        if PYGAME_AVAILABLE:
            self._renderer_backend = "pygame"
        elif PIL_AVAILABLE:
            self._renderer_backend = "pil"
        else:
            raise RuntimeError("Neither Pygame nor PIL available for minimap rendering")
    
    def set_world_bounds(self, min_x: float, max_x: float, min_y: float, max_y: float):
        """Set world bounds for coordinate mapping"""
        self._pending_world_bounds = (min_x, max_x, min_y, max_y)
        if self.renderer:
            self.renderer.set_world_bounds(min_x, max_x, min_y, max_y)

    def set_road_geometry(self, road_segments: List[List[Tuple[float, float]]]):
        """Set road geometry for rendering."""
        self._pending_road_segments = road_segments
        if self.renderer:
            self.renderer.set_road_geometry(road_segments)

    def set_route_path(self, route_path: List[Tuple[float, float]]):
        """Set ambulance route geometry for rendering."""
        self._pending_route_path = route_path
        if self.renderer:
            self.renderer.set_route_path(route_path)
    
    def run(self):
        """Main thread loop"""
        self.running = True
        
        try:
            if self._renderer_backend == "pygame":
                self.renderer = PyGameMinimapRenderer(self.map_name, self.width, self.height)
            else:
                self.renderer = PILMinimapRenderer(self.map_name, self.width, self.height)

            if self._pending_world_bounds:
                self.renderer.set_world_bounds(*self._pending_world_bounds)
            if self._pending_road_segments:
                self.renderer.set_road_geometry(self._pending_road_segments)
            if self._pending_route_path:
                self.renderer.set_route_path(self._pending_route_path)

            while self.running:
                # Process all pending updates
                updates = self.update_queue.get_all_updates()
                for vehicle_id, position, vehicle_type, rotation in updates:
                    self.renderer.update_vehicle(vehicle_id, position, vehicle_type, rotation)
                
                # Render
                if isinstance(self.renderer, PyGameMinimapRenderer):
                    self.renderer.handle_events()
                    if not self.renderer.running:
                        self.running = False
                        break
                
                self.renderer.render()
                
                # Frame rate control
                time.sleep(1.0 / self.fps)
        
        except Exception as e:
            print(f"[MINIMAP ERROR] {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            self.stop()
    
    def stop(self):
        """Stop minimap thread"""
        self.running = False
        if self.renderer and isinstance(self.renderer, PyGameMinimapRenderer):
            self.renderer.quit()
        if self.renderer:
            self.renderer.set_route_path([])
            self.renderer.set_road_geometry([])
            self.renderer.vehicles.clear()
    
    def remove_vehicle(self, vehicle_id: int):
        """Remove vehicle from display"""
        if self.renderer:
            self.renderer.remove_vehicle(vehicle_id)


def get_vehicle_world_bounds(world: 'carla.World') -> Tuple[float, float, float, float]:
    """Get the world bounding box from spawn points"""
    _, bounds = get_map_road_geometry(world)
    return bounds


def get_map_road_geometry(world: 'carla.World', sample_distance: float = 6.0) -> Tuple[List[List[Tuple[float, float]]], Tuple[float, float, float, float]]:
    """Build a cached road centerline geometry and matching world bounds."""
    try:
        topology = world.get_map().get_topology()
        if not topology:
            return [], (-100, 100, -100, 100)

        road_segments: List[List[Tuple[float, float]]] = []
        xs: List[float] = []
        ys: List[float] = []
        seen = set()

        for start_wp, end_wp in topology:
            segment_key = (
                start_wp.road_id,
                start_wp.section_id,
                start_wp.lane_id,
                round(start_wp.transform.location.x, 1),
                round(start_wp.transform.location.y, 1),
                end_wp.road_id,
                end_wp.section_id,
                end_wp.lane_id,
                round(end_wp.transform.location.x, 1),
                round(end_wp.transform.location.y, 1),
            )
            if segment_key in seen:
                continue
            seen.add(segment_key)

            segment = _sample_topology_segment(start_wp, end_wp, sample_distance)
            if len(segment) < 2:
                continue

            road_segments.append(segment)
            for x, y in segment:
                xs.append(x)
                ys.append(y)

        if not road_segments:
            return [], (-100, 100, -100, 100)

        padding = 40.0
        bounds = (
            min(xs) - padding,
            max(xs) + padding,
            min(ys) - padding,
            max(ys) + padding,
        )
        return road_segments, bounds
    except Exception:
        return [], (-100, 100, -100, 100)


def _sample_topology_segment(start_wp: 'carla.Waypoint', end_wp: 'carla.Waypoint', sample_distance: float) -> List[Tuple[float, float]]:
    """Sample a single topology segment into a polyline."""
    points = [(start_wp.transform.location.x, start_wp.transform.location.y)]
    current_wp = start_wp
    end_location = end_wp.transform.location

    for _ in range(512):
        if current_wp.transform.location.distance(end_location) <= sample_distance:
            break

        next_waypoints = current_wp.next(sample_distance)
        if not next_waypoints:
            break

        next_wp = min(next_waypoints, key=lambda wp: wp.transform.location.distance(end_location))
        next_point = (next_wp.transform.location.x, next_wp.transform.location.y)
        if next_point != points[-1]:
            points.append(next_point)

        current_wp = next_wp

    end_point = (end_location.x, end_location.y)
    if points[-1] != end_point:
        points.append(end_point)

    return points
