"""
UI Launcher for V2X Ambulance Simulation
Allows configuring Type A/B/C vehicle counts and scenario parameters before each run.
"""

import glob
import copy
import json
import os
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import signal
import gc

# Ensure CARLA Python API is importable for map preview features.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'PythonAPI', 'carla'))
import carla

try:
    from agents.navigation.global_route_planner import GlobalRoutePlanner
except Exception:
    GlobalRoutePlanner = None

from simulation import run_with_overrides

class SimulationLauncherUI:
    def __init__(self, root):
        self.root = root
        self.root.title("V2X Ambulance Simulation Launcher")
        self.root.geometry("980x700")
        self.root.resizable(False, False)

        self.preview_canvas_width = 560
        self.preview_canvas_height = 560
        self.preview_padding = 28
        self.spawn_points = []
        self.screen_points = []
        self.preview_route_world_path = []
        self.preview_bounds = None
        self.preview_map_name = None
        self.preview_world_map = None
        self.start_selected_index = None
        self.end_selected_index = None
        self.preview_loading = False
        self.preview_status_var = tk.StringVar(value="Preview: Select a map and click Load Preview")
        self.selection_mode = tk.StringVar(value="start")
        self.carla_executable = self._resolve_carla_executable()
        self.simulation_thread = None
        self.simulation_running = False
        self.simulation_process = None
        self.last_run_overrides = None
        self.restart_requested = False

        self._build_form()

    def _build_form(self):
        frame = ttk.Frame(self.root, padding=16)
        frame.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(frame)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 12))

        right = ttk.Frame(frame)
        right.grid(row=0, column=1, sticky="nsew")

        title = ttk.Label(
            left,
            text="Configure Simulation Scenario",
            font=("Segoe UI", 14, "bold")
        )
        title.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 12))

        self.map_name = tk.StringVar(value="Town01")
        self.num_runs = tk.StringVar(value="1")
        self.type_a_count = tk.StringVar(value="12")
        self.type_b_count = tk.StringVar(value="12")
        self.type_c_count = tk.StringVar(value="12")
        self.ambulance_start = tk.StringVar(value="0")
        self.ambulance_end = tk.StringVar(value="-1")
        self.storm_enabled = tk.BooleanVar(value=False)

        self.total_label = ttk.Label(frame, text="Total Traffic Vehicles: 60")

        row = 1
        ttk.Label(left, text="Map Name").grid(row=row, column=0, sticky="w", pady=4)
        self.map_combo = ttk.Combobox(
            left,
            textvariable=self.map_name,
            values=self._candidate_maps(),
            state="normal",
            width=20,
        )
        self.map_combo.grid(row=row, column=1, sticky="e", pady=4)
        self.load_preview_button = ttk.Button(left, text="Load Preview", command=self._on_load_preview_clicked)
        self.load_preview_button.grid(row=row, column=2, sticky="e", pady=4, padx=(8, 0))
        row += 1

        workflow_title = ttk.Label(left, text="Workflow", font=("Segoe UI", 10, "bold"))
        workflow_title.grid(row=row, column=0, columnspan=3, sticky="w", pady=(6, 4))
        row += 1

        flow_frame = ttk.Frame(left)
        flow_frame.grid(row=row, column=0, columnspan=3, sticky="w")
        self.open_carla_button = ttk.Button(flow_frame, text="1) Open CarlaUE4", command=self._open_carla)
        self.open_carla_button.pack(side=tk.LEFT)
        self.restart_carla_button = ttk.Button(flow_frame, text="Restart CarlaUE4", command=self._restart_carla)
        self.restart_carla_button.pack(side=tk.LEFT, padx=(8, 0))
        self.open_results_button = ttk.Button(flow_frame, text="4) Open Results", command=self._open_results_folder)
        self.open_results_button.pack(side=tk.LEFT, padx=(8, 0))
        row += 1

        self.workflow_hint = ttk.Label(
            left,
            text="Order: 1) Open CarlaUE4 -> modify UI -> Run Simulation -> Open Results",
        )
        self.workflow_hint.grid(row=row, column=0, columnspan=3, sticky="w", pady=(2, 6))
        row += 1

        self._add_labeled_input(left, row, "Number of Runs", self.num_runs)
        row += 1
        self._add_labeled_input(left, row, "Type A Count (basic CARLA)", self.type_a_count)
        row += 1
        self._add_labeled_input(left, row, "Type B Count (left-lane evasion via rear lidar)", self.type_b_count)
        row += 1
        self._add_labeled_input(left, row, "Type C Count (autonomous + lidar + V2X)", self.type_c_count)
        row += 1
        self._add_labeled_input(left, row, "Ambulance Start Waypoint Index", self.ambulance_start)
        row += 1
        self._add_labeled_input(left, row, "Ambulance End Waypoint Index (-1 random)", self.ambulance_end)
        row += 1

        selection_title = ttk.Label(left, text="Map Point Selection", font=("Segoe UI", 10, "bold"))
        selection_title.grid(row=row, column=0, columnspan=3, sticky="w", pady=(8, 4))
        row += 1

        mode_frame = ttk.Frame(left)
        mode_frame.grid(row=row, column=0, columnspan=3, sticky="w")
        ttk.Radiobutton(mode_frame, text="Select Start", variable=self.selection_mode, value="start").pack(side=tk.LEFT)
        ttk.Radiobutton(mode_frame, text="Select End", variable=self.selection_mode, value="end").pack(side=tk.LEFT, padx=(10, 0))
        ttk.Button(mode_frame, text="Clear Selection", command=self._clear_selection).pack(side=tk.LEFT, padx=(14, 0))
        row += 1

        storm_check = ttk.Checkbutton(left, text="Enable Storm Weather", variable=self.storm_enabled)
        storm_check.grid(row=row, column=0, columnspan=3, sticky="w", pady=(8, 8))
        row += 1

        self.total_label.grid(row=row, column=0, columnspan=3, sticky="w", pady=(4, 12))
        row += 1

        self.status_label = ttk.Label(left, text="Status: Ready")
        self.status_label.grid(row=row, column=0, columnspan=3, sticky="w", pady=(0, 6))
        row += 1

        self.preview_status_label = ttk.Label(left, textvariable=self.preview_status_var)
        self.preview_status_label.grid(row=row, column=0, columnspan=3, sticky="w", pady=(0, 10))
        row += 1

        self.run_button = ttk.Button(left, text="Run Simulation", command=self._on_run_clicked)
        self.run_button.grid(row=row, column=0, sticky="w")

        self.restart_button = ttk.Button(
            left,
            text="Restart Simulation",
            command=self._on_restart_clicked,
            state=tk.DISABLED,
        )
        self.restart_button.grid(row=row, column=1, sticky="ew", padx=(8, 8))

        quit_button = ttk.Button(left, text="Close", command=self.root.destroy)
        quit_button.grid(row=row, column=2, sticky="e")

        self.preview_canvas = tk.Canvas(
            right,
            width=self.preview_canvas_width,
            height=self.preview_canvas_height,
            bg="#111318",
            highlightthickness=1,
            highlightbackground="#2e3440",
        )
        self.preview_canvas.grid(row=0, column=0, sticky="nsew")
        self.preview_canvas.bind("<Button-1>", self._on_preview_click)
        self._draw_preview_placeholder()

        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(0, weight=1)

        left.columnconfigure(0, weight=1)
        left.columnconfigure(1, weight=1)
        left.columnconfigure(2, weight=1)
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)

        self.type_a_count.trace_add("write", lambda *_: self._refresh_total())
        self.type_b_count.trace_add("write", lambda *_: self._refresh_total())
        self.type_c_count.trace_add("write", lambda *_: self._refresh_total())
        self.ambulance_start.trace_add("write", self._on_preview_selection_changed)
        self.ambulance_end.trace_add("write", self._on_preview_selection_changed)

    def _candidate_maps(self):
        return [
            "Town01", "Town02", "Town03", "Town04", "Town05",
            "Town06", "Town07", "Town10HD", "Town11", "Town12", "Town13",
        ]

    def _resolve_carla_executable(self):
        candidates = [
            os.path.join(os.path.dirname(__file__), "..", "..", "CarlaUE4.exe"),
            os.path.join(os.path.dirname(__file__), "..", "CarlaUE4.exe"),
            os.path.join(os.path.dirname(__file__), "..", "..", "CarlaUE4", "Binaries", "Win64", "CarlaUE4.exe"),
            os.path.join(os.path.dirname(__file__), "..", "CarlaUE4", "Binaries", "Win64", "CarlaUE4.exe"),
        ]
        for path in candidates:
            abs_path = os.path.abspath(path)
            if os.path.isfile(abs_path):
                return abs_path
        return ""

    def _open_carla(self):
        if not self.carla_executable:
            messagebox.showerror(
                "CARLA Not Found",
                "Could not find CarlaUE4.exe automatically.\nPlease start CARLA manually, then continue.",
            )
            return

        try:
            subprocess.Popen([self.carla_executable], cwd=os.path.dirname(self.carla_executable))
            self.status_label.config(text="Status: CarlaUE4 launching... wait 30-60 seconds, then load preview/run.")
        except Exception as error:
            messagebox.showerror("Launch Failed", f"Failed to launch CarlaUE4.\n\n{error}")

    def _restart_carla(self):
        if not self.carla_executable:
            messagebox.showerror(
                "CARLA Not Found",
                "Could not find CarlaUE4.exe automatically.\nPlease restart CARLA manually.",
            )
            return

        try:
            if os.name == "nt":
                subprocess.run(["taskkill", "/F", "/IM", "CarlaUE4.exe"], check=False, capture_output=True, text=True)
            
            # Clear Python memory cache before restarting CARLA
            gc.collect()
            
            # Wait a bit for the system to free up resources
            time.sleep(2)
            
            subprocess.Popen([self.carla_executable], cwd=os.path.dirname(self.carla_executable))
            self.status_label.config(text="Status: CarlaUE4 restarted. Wait 30-60 seconds before simulation.")
        except Exception as error:
            messagebox.showerror("Restart Failed", f"Failed to restart CarlaUE4.\n\n{error}")

    def _open_results_folder(self):
        results_dir = os.path.join(os.path.dirname(__file__), "results")
        os.makedirs(results_dir, exist_ok=True)
        try:
            os.startfile(results_dir)
        except Exception as error:
            messagebox.showerror("Open Results Failed", f"Could not open results folder.\n\n{error}")

    def _add_labeled_input(self, parent, row, label, variable):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=4)
        entry = ttk.Entry(parent, textvariable=variable, width=24)
        entry.grid(row=row, column=1, sticky="e", pady=4)

    def _refresh_total(self):
        try:
            total = int(self.type_a_count.get()) + int(self.type_b_count.get()) + int(self.type_c_count.get())
            self.total_label.config(text=f"Total Traffic Vehicles: {total}")
        except ValueError:
            self.total_label.config(text="Total Traffic Vehicles: invalid input")

    def _validate_inputs(self):
        try:
            num_runs = int(self.num_runs.get())
            type_a = int(self.type_a_count.get())
            type_b = int(self.type_b_count.get())
            type_c = int(self.type_c_count.get())
            start_wp = int(self.ambulance_start.get())
            end_wp = int(self.ambulance_end.get())
        except ValueError:
            messagebox.showerror("Invalid Input", "Numeric fields must contain valid integers.")
            return None

        if num_runs <= 0:
            messagebox.showerror("Invalid Input", "Number of runs must be greater than 0.")
            return None

        if type_a < 0 or type_b < 0 or type_c < 0:
            messagebox.showerror("Invalid Input", "Vehicle counts cannot be negative.")
            return None

        if (type_a + type_b + type_c) <= 0:
            messagebox.showerror("Invalid Input", "At least one traffic vehicle must be spawned.")
            return None

        # Treat the text fields as source-of-truth at run time.
        # Preview clicks already update these fields, so this avoids stale
        # in-memory selection state overriding manual edits.
        self.start_selected_index = start_wp
        self.end_selected_index = end_wp

        overrides = {
            "map_name": self.map_name.get().strip() or "Town01",
            "num_runs": num_runs,
            "type_a_count": type_a,
            "type_b_count": type_b,
            "type_c_count": type_c,
            "ambulance_start_waypoint": start_wp,
            "ambulance_end_waypoint": end_wp,
            "strict_ambulance_start_waypoint": True,
            "storm_enabled": bool(self.storm_enabled.get()),
        }

        # Always resolve start/end coordinates from the active map at run time.
        # This avoids stale preview data causing default spawn behavior.
        active_map = overrides["map_name"]
        if not self._ensure_spawn_points_for_map(active_map):
            messagebox.showerror(
                "Missing Spawn Points",
                "Unable to resolve spawn points for selected map from CARLA. "
                "Load Preview and ensure CARLA is ready.",
            )
            return None

        if not (0 <= start_wp < len(self.spawn_points)):
            messagebox.showerror("Invalid Start", "Selected start index is outside map spawn points.")
            return None
        if not (0 <= end_wp < len(self.spawn_points)):
            messagebox.showerror("Invalid End", "Selected end index is outside map spawn points.")
            return None

        start_loc = self.spawn_points[start_wp].location
        overrides["ambulance_start_location"] = {
            "x": float(start_loc.x),
            "y": float(start_loc.y),
            "z": float(start_loc.z),
        }

        end_loc = self.spawn_points[end_wp].location
        overrides["ambulance_end_location"] = {
            "x": float(end_loc.x),
            "y": float(end_loc.y),
            "z": float(end_loc.z),
        }

        return overrides

    def _ensure_spawn_points_for_map(self, map_name):
        """Ensure spawn points correspond to the selected map, fetching from CARLA if needed."""
        if self.spawn_points and self.preview_map_name == map_name:
            return True

        try:
            client = carla.Client("localhost", 2000)
            client.set_timeout(20.0)
            world = client.load_world(map_name)
            world_map = world.get_map()
            spawn_points = world_map.get_spawn_points()
            if not spawn_points:
                return False

            points, bounds = self._project_spawn_points(spawn_points)
            self.spawn_points = spawn_points
            self.screen_points = points
            self.preview_bounds = bounds
            self.preview_map_name = map_name
            self.preview_world_map = world_map
            self.start_selected_index = self._coerce_index(self.ambulance_start.get(), len(self.spawn_points))
            self.end_selected_index = self._coerce_index(self.ambulance_end.get(), len(self.spawn_points))
            self._update_preview_route_path()
            if self.screen_points:
                self._redraw_preview()
            return True
        except Exception:
            return False

    def _draw_preview_placeholder(self):
        self.preview_canvas.delete("all")
        w = self.preview_canvas_width
        h = self.preview_canvas_height
        self.preview_canvas.create_text(
            w / 2,
            h / 2,
            fill="#d8dee9",
            text="Map preview will appear here\nLoad Preview to select start/end points",
            font=("Segoe UI", 11),
            justify=tk.CENTER,
        )

    def _on_load_preview_clicked(self):
        if self.preview_loading:
            return

        map_name = self.map_name.get().strip()
        if not map_name:
            messagebox.showerror("Missing Map", "Please enter a map name before loading preview.")
            return

        self.preview_loading = True
        self.load_preview_button.config(state=tk.DISABLED)
        self.preview_status_var.set(f"Preview: Loading {map_name} from CARLA...")

        worker = threading.Thread(target=self._load_map_preview_worker, args=(map_name,), daemon=True)
        worker.start()

    def _load_map_preview_worker(self, map_name):
        try:
            client = carla.Client("localhost", 2000)
            client.set_timeout(45.0)
            world = client.load_world(map_name)
            world_map = world.get_map()
            spawn_points = world_map.get_spawn_points()

            if not spawn_points:
                raise RuntimeError(f"Map {map_name} has no spawn points.")

            points, bounds = self._project_spawn_points(spawn_points)
            self.root.after(0, self._on_preview_loaded, map_name, world_map, spawn_points, points, bounds)
        except Exception as error:
            self.root.after(0, self._on_preview_failed, str(error))

    def _project_spawn_points(self, spawn_points):
        xs = [sp.location.x for sp in spawn_points]
        ys = [sp.location.y for sp in spawn_points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        usable_w = self.preview_canvas_width - 2 * self.preview_padding
        usable_h = self.preview_canvas_height - 2 * self.preview_padding
        span_x = max(1e-6, max_x - min_x)
        span_y = max(1e-6, max_y - min_y)

        scale = min(usable_w / span_x, usable_h / span_y)

        points = []
        for idx, sp in enumerate(spawn_points):
            px = self.preview_padding + (sp.location.x - min_x) * scale
            py = self.preview_canvas_height - (self.preview_padding + (sp.location.y - min_y) * scale)
            points.append((idx, px, py))

        bounds = (min_x, max_x, min_y, max_y)
        return points, bounds

    def _on_preview_loaded(self, map_name, world_map, spawn_points, points, bounds):
        self.preview_loading = False
        self.load_preview_button.config(state=tk.NORMAL)

        self.spawn_points = spawn_points
        self.screen_points = points
        self.preview_bounds = bounds
        self.preview_map_name = map_name
        self.preview_world_map = world_map

        self.start_selected_index = self._coerce_index(self.ambulance_start.get(), len(self.spawn_points))
        self.end_selected_index = self._coerce_index(self.ambulance_end.get(), len(self.spawn_points))
        self._update_preview_route_path()

        self.preview_status_var.set(
            f"Preview: {map_name} loaded with {len(self.spawn_points)} spawn points. Click to select start/end."
        )
        self._redraw_preview()

    def _on_preview_failed(self, error):
        self.preview_loading = False
        self.load_preview_button.config(state=tk.NORMAL)
        self.preview_status_var.set("Preview: Failed to load map")
        messagebox.showerror(
            "Preview Load Failed",
            f"Unable to load map preview. Ensure CARLA is running on localhost:2000.\n\n{error}",
        )

    def _coerce_index(self, value, max_len):
        try:
            idx = int(value)
        except ValueError:
            return None

        if 0 <= idx < max_len:
            return idx
        return None

    def _redraw_preview(self):
        self.preview_canvas.delete("all")

        self.preview_canvas.create_rectangle(
            0,
            0,
            self.preview_canvas_width,
            self.preview_canvas_height,
            fill="#151a22",
            outline="",
        )

        self.preview_canvas.create_text(
            10,
            10,
            anchor="nw",
            fill="#d8dee9",
            font=("Segoe UI", 10, "bold"),
            text="Bird's-eye Spawn Point Preview",
        )

        if self.preview_route_world_path and self.preview_bounds:
            route_points = [self._world_to_screen(x, y) for x, y in self.preview_route_world_path]
            route_points = [(px, py) for px, py in route_points if px is not None and py is not None]
            if len(route_points) >= 2:
                flattened = []
                for px, py in route_points:
                    flattened.extend((px, py))
                self.preview_canvas.create_line(
                    *flattened,
                    fill="#ffd166",
                    width=4,
                    smooth=True,
                    splinesteps=24,
                )

        for idx, px, py in self.screen_points:
            color = "#7f8c9b"
            radius = 2
            if idx == self.start_selected_index:
                color = "#2ecc71"
                radius = 5
            elif idx == self.end_selected_index:
                color = "#e74c3c"
                radius = 5

            self.preview_canvas.create_oval(px - radius, py - radius, px + radius, py + radius, fill=color, outline="")

        self.preview_canvas.create_text(
            10,
            self.preview_canvas_height - 32,
            anchor="nw",
            fill="#aab7c4",
            font=("Segoe UI", 9),
            text="Green: start  |  Red: end  |  Click nearest point to assign",
        )

        start_text = self.ambulance_start.get()
        end_text = self.ambulance_end.get()
        self.preview_canvas.create_text(
            10,
            self.preview_canvas_height - 16,
            anchor="nw",
            fill="#cbd5e1",
            font=("Segoe UI", 9),
            text=f"Selected Start: {start_text}   End: {end_text}",
        )

    def _world_to_screen(self, x, y):
        if not self.preview_bounds:
            return None, None

        min_x, max_x, min_y, max_y = self.preview_bounds
        span_x = max(1e-6, max_x - min_x)
        span_y = max(1e-6, max_y - min_y)
        usable_w = self.preview_canvas_width - 2 * self.preview_padding
        usable_h = self.preview_canvas_height - 2 * self.preview_padding
        scale = min(usable_w / span_x, usable_h / span_y)

        px = self.preview_padding + (x - min_x) * scale
        py = self.preview_canvas_height - (self.preview_padding + (y - min_y) * scale)
        return px, py

    def _update_preview_route_path(self):
        self.preview_route_world_path = []

        if not self.spawn_points or self.preview_bounds is None:
            return

        start_index = self._coerce_index(self.ambulance_start.get(), len(self.spawn_points))
        end_index = self._coerce_index(self.ambulance_end.get(), len(self.spawn_points))
        self.start_selected_index = start_index
        self.end_selected_index = end_index

        if start_index is None or end_index is None:
            return
        if start_index == end_index:
            location = self.spawn_points[start_index].location
            self.preview_route_world_path = [(float(location.x), float(location.y))]
            return

        start_location = self.spawn_points[start_index].location
        end_location = self.spawn_points[end_index].location

        route_points = []
        try:
            if GlobalRoutePlanner is not None and self.preview_world_map is not None:
                planner = GlobalRoutePlanner(self.preview_world_map, 2.0)
                route_trace = planner.trace_route(start_location, end_location)
                for waypoint, _ in route_trace:
                    location = waypoint.transform.location
                    point = (float(location.x), float(location.y))
                    if not route_points or route_points[-1] != point:
                        route_points.append(point)
        except Exception:
            route_points = []

        if not route_points:
            route_points = [
                (float(start_location.x), float(start_location.y)),
                (float(end_location.x), float(end_location.y)),
            ]

        self.preview_route_world_path = route_points

    def _on_preview_selection_changed(self, *_):
        if not self.spawn_points:
            return

        self._update_preview_route_path()
        self.preview_status_var.set(
            f"Preview: route updated for start {self.ambulance_start.get()} and end {self.ambulance_end.get()}."
        )
        self._redraw_preview()

    def _on_preview_click(self, event):
        if not self.screen_points:
            return

        nearest_index = None
        nearest_dist_sq = None
        for idx, px, py in self.screen_points:
            dx = px - event.x
            dy = py - event.y
            dist_sq = dx * dx + dy * dy
            if nearest_dist_sq is None or dist_sq < nearest_dist_sq:
                nearest_dist_sq = dist_sq
                nearest_index = idx

        if nearest_index is None:
            return

        mode = self.selection_mode.get()
        if mode == "start":
            self.start_selected_index = nearest_index
            self.ambulance_start.set(str(nearest_index))
        else:
            self.end_selected_index = nearest_index
            self.ambulance_end.set(str(nearest_index))

        self._update_preview_route_path()
        self.preview_status_var.set(
            f"Preview: {mode.capitalize()} waypoint set to index {nearest_index}."
        )
        self._redraw_preview()

    def _clear_selection(self):
        self.start_selected_index = None
        self.end_selected_index = None
        self.preview_route_world_path = []
        self.ambulance_start.set("0")
        self.ambulance_end.set("-1")
        self.preview_status_var.set("Preview: Selection cleared")
        if self.screen_points:
            self._redraw_preview()

    def _on_run_clicked(self):
        if self.simulation_running:
            return

        overrides = self._validate_inputs()
        if overrides is None:
            return

        self.last_run_overrides = copy.deepcopy(overrides)
        self.restart_requested = False
        self._start_simulation(overrides, status_text="Status: Simulation running... check terminal for live logs.")

    def _start_simulation(self, overrides, status_text):
        self.run_button.config(state=tk.DISABLED)
        self.restart_button.config(state=tk.NORMAL)
        self.status_label.config(text=status_text)
        self.simulation_running = True
        self.simulation_process = subprocess.Popen(
            self._build_simulation_command(overrides),
            cwd=os.path.dirname(__file__),
        )

        self.simulation_thread = threading.Thread(
            target=self._monitor_simulation_process,
            args=(self.simulation_process,),
            daemon=True,
        )
        self.simulation_thread.start()

    def _build_simulation_command(self, overrides):
        payload = json.dumps(overrides)
        entry_code = (
            "import json, sys\n"
            "from simulation import run_with_overrides\n"
            "run_with_overrides(json.loads(sys.argv[1]))\n"
        )
        return [sys.executable, "-u", "-c", entry_code, payload]

    def _on_restart_clicked(self):
        if not self.last_run_overrides:
            messagebox.showerror("No Previous Run", "Run a simulation at least once before restarting.")
            return

        if self.simulation_running:
            self.restart_requested = True
            self.restart_button.config(state=tk.DISABLED)
            self.status_label.config(text="Status: Restart requested... stopping current simulation.")
            self._terminate_active_simulation()
            return

        self._start_simulation(
            copy.deepcopy(self.last_run_overrides),
            status_text="Status: Restarting simulation with last settings...",
        )

    def _terminate_active_simulation(self):
        process = self.simulation_process
        if not process:
            self.root.after(0, self._start_restart_after_stop)
            return

        try:
            process.terminate()
        except Exception:
            pass

    def _monitor_simulation_process(self, process):
        # The simulation can legitimately run for minutes, so keep polling until
        # the child process exits instead of killing it after a short timeout.
        returncode = None
        while True:
            returncode = process.poll()
            if returncode is not None:
                break
            time.sleep(1.0)

        self.root.after(0, self._on_simulation_process_finished, returncode)

    def _on_simulation_process_finished(self, returncode):
        self.simulation_process = None

        if self.restart_requested:
            self.root.after(0, self._start_restart_after_stop)
            return

        if returncode not in (0, None):
            self._on_run_failure(f"Simulation process exited with code {returncode}.")
            return

        latest_sheet = self._latest_result_sheet()
        self._on_run_success(latest_sheet)

    def _start_restart_after_stop(self):
        self.restart_requested = False

        if not self.last_run_overrides:
            self.run_button.config(state=tk.NORMAL)
            self.restart_button.config(state=tk.DISABLED)
            self.status_label.config(text="Status: Restart canceled. No previous run settings available.")
            return

        self._start_simulation(
            copy.deepcopy(self.last_run_overrides),
            status_text="Status: Restarting simulation with last settings...",
        )

    def _latest_result_sheet(self):
        results_dir = os.path.join(os.path.dirname(__file__), "results")
        pattern = os.path.join(results_dir, "v2x_result_sheet_*.csv")
        files = glob.glob(pattern)
        if not files:
            return ""
        files.sort(key=os.path.getmtime, reverse=True)
        return files[0]

    def _on_run_success(self, sheet_path):
        self.run_button.config(state=tk.NORMAL)
        self.restart_button.config(state=tk.NORMAL if self.last_run_overrides else tk.DISABLED)
        self.status_label.config(text="Status: Simulation completed.")
        self.simulation_running = False
        self.restart_requested = False
        self.simulation_process = None

        if sheet_path:
            messagebox.showinfo(
                "Simulation Complete",
                f"Simulation finished successfully.\nResult sheet:\n{sheet_path}"
            )
            try:
                os.startfile(os.path.dirname(sheet_path))
            except Exception:
                pass
        else:
            messagebox.showinfo(
                "Simulation Complete",
                "Simulation finished successfully. No result sheet file was found."
            )

    def _on_run_stopped(self):
        self.run_button.config(state=tk.NORMAL)
        self.restart_button.config(state=tk.NORMAL if self.last_run_overrides else tk.DISABLED)
        self.status_label.config(text="Status: Simulation stopped.")
        self.simulation_running = False
        self.restart_requested = False
        self.simulation_process = None

    def _on_run_failure(self, error):
        self.run_button.config(state=tk.NORMAL)
        self.restart_button.config(state=tk.NORMAL if self.last_run_overrides else tk.DISABLED)
        self.status_label.config(text="Status: Simulation failed.")
        self.simulation_running = False
        self.restart_requested = False
        self.simulation_process = None
        messagebox.showerror("Simulation Failed", error)


def main():
    root = tk.Tk()
    style = ttk.Style(root)
    if "vista" in style.theme_names():
        style.theme_use("vista")

    app = SimulationLauncherUI(root)
    app._refresh_total()

    # Make sure the launcher appears in front (CARLA often steals focus).
    root.update_idletasks()
    width = root.winfo_width()
    height = root.winfo_height()
    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()
    pos_x = max(0, (screen_w - width) // 2)
    pos_y = max(0, (screen_h - height) // 2)
    root.geometry(f"{width}x{height}+{pos_x}+{pos_y}")
    root.deiconify()
    root.lift()
    root.attributes("-topmost", True)
    root.after(250, lambda: root.attributes("-topmost", False))
    root.focus_force()

    root.mainloop()


if __name__ == "__main__":
    main()
