"""
V2X Ambulance Simulation Configuration
"""

# CARLA Connection
CARLA_HOST = "localhost"
CARLA_PORT = 2000
CARLA_TIMEOUT = 60.0  # Increased timeout to reduce simulator timeout errors
TICK_RETRY_COUNT = 3  # Retry world.tick() this many times on transient timeout
TICK_RETRY_DELAY = 0.25  # Seconds between tick retry attempts

# Map Configuration
MAP_NAME = "Town01"  # Change to other town maps as needed (Town01-Town13)

# Simulation Parameters
SIMULATION_FPS = 30  # Target frame rate in Hz (stable physics without overdriving CPU)
NUM_RUNS = 5  # Number of simulation runs to average
SIMULATION_DURATION = 120  # seconds per run
TRAFFIC_DENSITY = 36  # Number of NPC vehicles (lower default for smoother runtime)
AMBULANCE_SPEED = 38  # km/h (target emergency cruising speed, reduced from 60 for safer cornering)
REGULAR_VEHICLE_SPEED = 15  # m/s

# Traffic Mix (Type A/B/C)
# Type A: Basic CARLA traffic logic (no autonomy stack, no lidar, no V2X)
# Type B: Autonomous + lidar (no V2X)
# Type C: Autonomous + V2X only (no lidar)
TYPE_A_COUNT = 12
TYPE_B_COUNT = 12
TYPE_C_COUNT = 12

# Traffic spawn safety around ambulance start
TRAFFIC_SPAWN_EXCLUSION_RADIUS = 30.0  # meters around ambulance where NPC traffic is not spawned

# NPC Lidar (Lightweight)
NPC_LIDAR_ENABLED = True
NPC_LIDAR_MODE = "nearby"  # off | nearby | all
NPC_LIDAR_RADIUS = 30.0  # meters around ambulance when mode=nearby
NPC_LIDAR_CHANNELS = 4
NPC_LIDAR_RANGE = 30.0
NPC_LIDAR_POINTS_PER_SECOND = 12000
NPC_LIDAR_ROTATION_FREQUENCY = 8
NPC_LIDAR_UPPER_FOV = 10.0
NPC_LIDAR_LOWER_FOV = -10.0
NPC_LIDAR_MIN_POINTS = 10
NPC_LIDAR_SLOW_DISTANCE = 8.0
NPC_LIDAR_STOP_DISTANCE = 3.0

# Ambulance lidar (kept enabled but reduced from high-density defaults)
AMBULANCE_LIDAR_CHANNELS = 8
AMBULANCE_LIDAR_RANGE = 35.0
AMBULANCE_LIDAR_POINTS_PER_SECOND = 64000
AMBULANCE_LIDAR_ROTATION_FREQUENCY = 10
AMBULANCE_LIDAR_UPPER_FOV = 10.0
AMBULANCE_LIDAR_LOWER_FOV = -10.0

# Traffic Light Control
DISABLE_TRAFFIC_LIGHTS = False  # Set to True to freeze all lights to green

# Ambulance Driving Behavior
# Options: "basic" or "behavior" (BehaviorAgent if available)
AMBULANCE_AGENT_TYPE = "basic"
AMBULANCE_BEHAVIOR = "normal"  # cautious | normal | aggressive
AMBULANCE_IGNORE_TRAFFIC_LIGHTS = True
AMBULANCE_IGNORE_STOP_SIGNS = True
AMBULANCE_PROJECT_DESTINATION_TO_ROAD = True
AMBULANCE_BLOCKED_SPEED_THRESHOLD = 7.0  # m/s, considered blocked when slower than this with front obstacle
AMBULANCE_BLOCKED_DISTANCE_THRESHOLD = 14.0  # m, front obstacle distance threshold
AMBULANCE_BLOCKED_FRAMES_THRESHOLD = 90  # ~4.5 seconds at 20 fps - VERY conservative, only overtake as last resort
AMBULANCE_OVERTAKE_LOOKAHEAD = 35.0  # m ahead on adjacent lane for temporary target
AMBULANCE_OVERTAKE_HOLD_TIME = 8.0  # seconds to stay in overtake mode before route restore (extended to complete maneuver)
AMBULANCE_OVERTAKE_COOLDOWN = 6.0  # seconds before trying another overtake (extended to prevent rapid lane switching)
AMBULANCE_ENABLE_SIDEWALK_ESCAPE = True  # Allow temporary shoulder/sidewalk routing if all lanes are blocked
AMBULANCE_SIDEWALK_BLOCKED_FRAMES_THRESHOLD = 150  # Trigger escape after VERY prolonged blockage (7.5 seconds)
AMBULANCE_SIDEWALK_ESCAPE_LOOKAHEAD = 25.0  # m ahead along shoulder/sidewalk
AMBULANCE_SIDEWALK_ESCAPE_HOLD_TIME = 5.0  # seconds before attempting to return to route
AMBULANCE_ESCAPE_SPEED = 20.0  # km/h while using shoulder/sidewalk escape paths
AMBULANCE_WARNING_TIME_GAP = 1.2  # seconds, dynamic warning distance from current speed
AMBULANCE_CRITICAL_TIME_GAP = 0.7  # seconds, dynamic emergency-brake distance

# V2X Communication Range
V2X_DETECTION_RANGE = 150.0  # meters - distance vehicles can detect ambulance

# Emergency Path Clearance Parameters
PATH_CLEARANCE_URGENCY = 2.0  # How urgently vehicles move out of the way (speed multiplier)
CLEARANCE_LOOK_AHEAD = 100.0  # How far ahead we look for obstacles

# Type C (V2X only) route-level avoidance behavior
TYPE_C_REROUTE_COOLDOWN = 4.0  # seconds between route-level reroute requests
TYPE_C_MAX_DETOUR_CANDIDATES = 12  # sampled detour points when finding non-overlapping path
TYPE_C_ROUTE_OVERLAP_CHECK_INTERVAL = 1.5  # seconds between expensive route-overlap recomputations

# Type B (autonomous + lidar, no V2X) lane-evasion behavior
TYPE_B_AMBULANCE_REAR_DETECTION_DISTANCE = 10.0  # meters (rear lidar + ambulance behind trigger)
TYPE_B_AMBULANCE_REAR_END_DISTANCE = 7.5  # meters (ambulance must be close to rear bumper of that individual Type B)
TYPE_B_AMBULANCE_REAR_LATERAL_TOLERANCE = 2.2  # meters lateral tolerance for rear-end alignment
TYPE_B_REAR_MATCH_TOLERANCE = 2.5  # meters tolerance between rear lidar obstacle distance and ambulance distance
TYPE_B_LANE_SWITCH_LOOKAHEAD = 20.0  # meters ahead on adjacent lane for smooth lane switch
TYPE_B_LANE_SWITCH_COOLDOWN = 4.0  # seconds between lane-switch responses
TYPE_B_LIDAR_ACTIVATION_DISTANCE = 25.0  # meters from ambulance before enabling type-B lidar

# Ambulance Route Parameters
AMBULANCE_START_WAYPOINT = 0  # Starting spawn point index
AMBULANCE_END_WAYPOINT = -1  # Destination waypoint index (-1 for random selection)
RETURN_TO_START = True  # Whether ambulance returns to origin

# Recording Parameters
RECORD_RESULTS = True
RESULTS_DIR = "./results"
SAVE_VEHICLE_TRACES = False  # Save detailed vehicle trajectories

# Visualization/Debug
DRAW_V2X_RANGE = False  # Draw detection range in simulation
DEBUG_PRINT = False  # Print detailed logs

# Disaster Visual Presets
# Options: "off", "storm", "flood", "fire_glow"
DISASTER_VISUAL_MODE = "off"  # Keep visuals stable by default; enable storm manually when needed
DISASTER_RUNTIME_TEXTURE_ENABLED = False
DISASTER_TEXTURE_TARGETS = 0
DISASTER_MARKERS_ENABLED = False  # Disabled - no cones/props at destination

# Fire target selection (optional override)
# If set, the animation will apply to the matching object name.
FIRE_TARGET_BUILDING_NAME = ""
# If no explicit name is set, prefer house-like objects.
FIRE_TARGET_BUILDING_KEYWORDS = ["house", "suburb"]
# Number of buildings to set on fire (for street-fire effect)
FIRE_TARGET_BUILDING_COUNT = 0  # Disable street-fire targets in performance profile
# When true, print candidate house objects near destination selection time.
FIRE_TARGET_DEBUG_LIST = False
# Max number of candidate names to print when debugging.
FIRE_TARGET_DEBUG_LIMIT = 25
# When true, prompt to select a target building from the printed list.
FIRE_TARGET_PROMPT = False

# Animated Fire Effect (for fire_glow preset)
FIRE_ANIMATION_SPEED = 0.15  # seconds between fire stage changes
FIRE_STAGES = [
    {"emissive": 12.0, "r_base": 255, "g_base": 80, "b_base": 10, "intensity": 1.0},  # Bright orange-red
    {"emissive": 18.0, "r_base": 255, "g_base": 120, "b_base": 20, "intensity": 1.2},  # Building intensity
    {"emissive": 24.0, "r_base": 255, "g_base": 100, "b_base": 30, "intensity": 1.4},  # High intensity
    {"emissive": 30.0, "r_base": 255, "g_base": 200, "b_base": 100, "intensity": 1.5},  # Peak yellow-white
    {"emissive": 20.0, "r_base": 255, "g_base": 80, "b_base": 40, "intensity": 1.3},  # Deep Red
    {"emissive": 15.0, "r_base": 255, "g_base": 150, "b_base": 50, "intensity": 1.1},  # Yellow-orange
]

# Camera Follow (Spectator)
CAMERA_FOLLOW_ENABLED = True
CAMERA_FOLLOW_DISTANCE = 12.0  # meters behind the ambulance
CAMERA_FOLLOW_HEIGHT = 6.0  # meters above the ambulance
CAMERA_FOLLOW_PITCH = -15.0  # degrees (negative looks downward)

# Real-Time Minimap Configuration
MINIMAP_ENABLED = True  # Enable/disable real-time minimap display
MINIMAP_WIDTH = 280  # Width in pixels
MINIMAP_HEIGHT = 280  # Height in pixels
MINIMAP_FPS = 6  # Window redraw frequency in frames per second
MINIMAP_UPDATE_EVERY_N_FRAMES = 12  # Push vehicle updates every N sim frames

# Performance tuning
NPC_V2X_UPDATE_INTERVAL = 1.0 / 10.0  # Update NPC V2X decisions at ~10 Hz
AMBULANCE_ROUTE_ALIGNMENT_INTERVAL = 1.0 / 8.0  # Recheck route drift at ~8 Hz
NPC_BEHAVIOR_UPDATE_EVERY_N_FRAMES = 3  # Stagger Type B/C controller updates across frames
SPECTATOR_UPDATE_EVERY_N_FRAMES = 3  # Update spectator camera at lower rate

# Minimap Color Legend:
# WHITE: Ambulance (emergency vehicle)
# RED: Type A vehicles (CARLA autopilot only - no V2X/LIDAR)
# YELLOW: Type B vehicles (LIDAR autonomous driving - no V2X communication)
# GREEN: Type C vehicles (V2X autonomous driving - no lidar)
#
# The minimap shows:
# - 2D bird-eye view of the town map
# - Vehicle positions as colored dots with rotation indicators
# - Grid background for reference
# - Live updates during simulation to show real-time vehicle behavior

