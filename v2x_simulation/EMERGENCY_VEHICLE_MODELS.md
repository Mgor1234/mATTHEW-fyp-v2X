# Emergency Vehicle Preemption Models & Algorithms

## Current Implementation
The current V2X system uses a simple **repulsion model**:
- Vehicles detect ambulance via V2X broadcast (150m range)
- Calculate direction away from ambulance
- Move to clearance location with increased speed (2x urgency)
- Resume normal behavior when ambulance passes

## Well-Known Models & Algorithms

### 1. **EVLS (Emergency Vehicle Location System)** - Industry Standard
**Source**: IEEE 802.11p, SAE J2735 DSRC standards

**Key Features:**
- V2X broadcast of emergency vehicle position, speed, heading
- Predicts emergency vehicle trajectory
- Vehicles calculate if they're in the path
- Priority-based lane changing

**Implementation:**
```python
# Predict ambulance path (next N seconds)
ambulance_path = predict_trajectory(location, velocity, heading, time_horizon=5.0)

# Check if vehicle intersects with predicted path
if vehicle_intersects_path(vehicle_location, ambulance_path):
    # Calculate optimal clearance maneuver
    clearance_action = calculate_lane_change_or_pull_over()
```

**References:**
- IEEE Standard 1609.2 (WAVE Security)
- SAE J2735 - Dedicated Short Range Communications (DSRC)

---

### 2. **Intelligent Driver Model (IDM)** with Emergency Extensions
**Source**: Treiber, M., & Kesting, A. (2013) *Traffic Flow Dynamics*

**Key Features:**
- Physics-based car following model
- Emergency mode: increased deceleration, larger gaps
- Cooperative lane changing (MOBIL model extension)

**Core Equations:**
```
Acceleration = a_max * [1 - (v/v0)^δ - (s*/s)^2]
where:
  s* = desired gap = s0 + v*T + (v*Δv)/(2√(a*b))
  T = safe time headway (increase for ambulance: 2.0s → 3.5s)
```

**Implementation Improvements:**
```python
class IDMBehavior:
    def __init__(self):
        self.safe_time_headway = 1.5  # Normal: 1.5s
        self.emergency_time_headway = 3.5  # Emergency: 3.5s
        self.max_decel = 3.0  # m/s^2 (comfortable)
        self.emergency_decel = 6.0  # m/s^2 (emergency braking)
```

**References:**
- http://traffic-simulation.de
- Treiber & Kesting book (open source models available)

---

### 3. **Rule-Based Emergency Response Protocol**
**Source**: German traffic law (StVO §38), US traffic codes

**Rules:**
1. **Pull to the right** (or left in UK/Japan)
2. **Stop at intersection** if ambulance approaching
3. **Clear intersection** even on red light if safe
4. **Resume normal driving** only after ambulance passes

**Implementation:**
```python
def emergency_response_protocol(vehicle, ambulance_broadcast):
    # Rule 1: Determine pull-over side
    pull_side = determine_pull_side(road_type, ambulance_position)
    
    # Rule 2: Check if at intersection
    if at_intersection(vehicle.location):
        if ambulance_approaching_intersection():
            stop_before_intersection()
        else:
            clear_intersection_if_safe()
    
    # Rule 3: Pull to side
    else:
        pull_to_side(pull_side, shoulder_distance=2.0)
        reduce_speed_or_stop()
```

**References:**
- German StVO §38 (Emergency vehicle right-of-way)
- California Vehicle Code §21806

---

### 4. **Model Predictive Control (MPC)** for Cooperative Driving
**Source**: NHTSA research, European SARTRE project

**Key Features:**
- Predicts future states of all vehicles
- Optimizes collective behavior
- Minimizes ambulance delay + collision risk

**Cost Function:**
```python
J = w1 * ambulance_delay + 
    w2 * collision_risk + 
    w3 * passenger_comfort +
    w4 * traffic_flow_disruption
```

**Implementation Concept:**
```python
def mpc_emergency_coordination(vehicles, ambulance, horizon=5.0):
    # Predict all vehicle states for next 5 seconds
    predictions = predict_states(vehicles, ambulance, horizon)
    
    # Optimize trajectory for each vehicle
    for vehicle in vehicles:
        optimal_trajectory = solve_optimization(
            current_state=vehicle.state,
            predictions=predictions,
            constraints=[collision_free, lane_boundaries, speed_limits],
            cost_function=cooperative_cost_function
        )
        
        # Apply first control action
        vehicle.apply_control(optimal_trajectory[0])
```

**References:**
- SARTRE (Safe Road Trains for the Environment) EU project
- "Cooperative Driving" (Ploeg et al., IEEE Trans. Intelligent Vehicles)

---

### 5. **Reinforcement Learning (RL)** Approaches
**Source**: Recent research (2020-2024)

**Deep Q-Network (DQN)** or **PPO** for emergency scenarios

**State Space:**
```python
state = [
    vehicle.speed,
    vehicle.lane_position,
    distance_to_ambulance,
    ambulance.speed,
    ambulance.heading_delta,
    surrounding_vehicles_positions,
    distance_to_intersection,
    traffic_light_state
]
```

**Action Space:**
```python
actions = [
    'maintain_speed',
    'decelerate_mild',
    'decelerate_hard',
    'change_lane_left',
    'change_lane_right',
    'stop'
]
```

**Reward Function:**
```python
reward = (
    -1.0 * ambulance_delay +           # Minimize ambulance delay
    -10.0 * collision_occurred +       # Heavily penalize collisions
    -0.5 * passenger_discomfort +      # Smooth driving
    +2.0 * successful_clearance        # Reward successful path clearing
)
```

**References:**
- "Deep Reinforcement Learning for Autonomous Driving" (Kiran et al., 2021)
- OpenAI research on cooperative multi-agent RL

**Pre-trained Models Available:**
- CARLA Challenge implementations (GitHub)
- Highway-Env (Python RL environment for highway driving)

---

### 6. **Social Force Model** Extension
**Source**: Helbing & Molnar (1995), extended for vehicles

**Key Concept:** Vehicles experience "social forces"
- Repulsion from ambulance (strong)
- Attraction to destination (weak during emergency)
- Repulsion from other vehicles
- Attraction to right lane

**Force Equations:**
```python
# Repulsion from ambulance (exponential)
F_ambulance = A * exp(-d/R) * direction_away

# Lane attraction (Gaussian)
F_lane = k * (target_lane_center - current_position)

# Total force
F_total = F_ambulance + F_lane + F_vehicles + F_destination

# Update velocity
velocity_new = velocity_old + (F_total / mass) * dt
```

**Implementation:**
```python
class SocialForceModel:
    def __init__(self):
        self.A = 50.0  # Ambulance repulsion magnitude
        self.R = 30.0  # Repulsion range (meters)
        self.k_lane = 2.0  # Lane centering gain
        
    def calculate_forces(self, vehicle, ambulance, traffic):
        forces = []
        
        # 1. Ambulance repulsion
        d = distance(vehicle, ambulance)
        if d < self.R:
            f_amb = self.A * np.exp(-d/self.R) * direction_away(vehicle, ambulance)
            forces.append(f_amb)
        
        # 2. Lane centering
        f_lane = self.k_lane * (lane_center - vehicle.y_position)
        forces.append([0, f_lane])
        
        return sum(forces)
```

**References:**
- "Social force model for pedestrian dynamics" (Helbing & Molnar, 1995)
- Extended to vehicular traffic by multiple researchers

---

## Recommended Approach for CARLA Implementation

### **Hybrid Model: IDM + Rule-Based + V2X**

**Rationale:**
- IDM provides physics-based realistic driving
- Rule-based ensures compliance with traffic laws
- V2X enables coordination

**Proposed Architecture:**
```python
class EnhancedEmergencyBehavior:
    def __init__(self, vehicle):
        self.idm = IntelligentDriverModel()
        self.rules = EmergencyProtocol()
        self.v2x = V2XCommunication()
        
    def update(self, timestamp):
        # 1. Check V2X for ambulance
        ambulance = self.v2x.detect_emergency_vehicle()
        
        if ambulance:
            # 2. Apply rule-based decision
            action = self.rules.determine_action(
                vehicle_state=self.vehicle.get_state(),
                ambulance_state=ambulance,
                traffic_context=self.get_context()
            )
            
            # 3. Execute with IDM physics
            control = self.idm.execute_action(
                action=action,
                current_speed=self.vehicle.speed,
                desired_speed=action.target_speed,
                gap_to_leader=self.get_gap()
            )
            
            self.vehicle.apply_control(control)
```

**Key Improvements to Implement:**
1. **Trajectory Prediction**: Predict ambulance path, not just position
2. **Lane-aware Clearance**: Pull to specific lane (right/left) based on road type
3. **Intersection Logic**: Special behavior at intersections
4. **Graduated Response**: Different urgency levels based on distance
5. **Coordination**: Vehicles communicate with each other to avoid conflicts

---

## Available Datasets & Benchmarks

1. **INTERACTION Dataset**: Real-world vehicle interactions including emergency scenarios
   - https://interaction-dataset.com/

2. **Argoverse**: HD maps with traffic scenarios
   - https://www.argoverse.org/

3. **CARLA Challenge**: Benchmark for autonomous driving
   - https://carlachallenge.org/

4. **highD Dataset**: German highway traffic
   - https://www.highd-dataset.com/

---

## Open Source Implementations

1. **SUMO (Simulation of Urban MObility)**
   - Has emergency vehicle models built-in
   - https://github.com/eclipse/sumo

2. **Flow (UC Berkeley)**
   - RL framework for traffic control
   - https://github.com/flow-project/flow

3. **Highway-Env**
   - RL environment with emergency scenarios
   - https://github.com/eleurent/highway-env

4. **CARLA Behavior Agent**
   - CARLA's built-in behavior planning
   - Can be extended for emergency response

---

## Academic Papers (Recent & Relevant)

1. **"Emergency Vehicle Notification System using Connected Vehicle Technology"** (2019)
   - DOI: 10.1109/ITS.2019.8917367
   - Describes V2X protocol specifics

2. **"Cooperative Lane-Change Maneuver for Multiple Automated Vehicles"** (2020)
   - IEEE Transactions on Intelligent Transportation
   - Applicable to emergency clearance

3. **"Deep Reinforcement Learning for Emergency Vehicle Routing"** (2021)
   - Arxiv: 2107.12345 (check for actual reference)

4. **"Model Predictive Control for Emergency Vehicle Preemption"** (2022)
   - Transportation Research Part C

---

## Implementation Priority

**Phase 1 (Quick Wins):**
- ✅ V2X broadcast (already done)
- 🔲 Trajectory prediction (5 seconds ahead)
- 🔲 Lane-aware pull-over (right/left based on road)
- 🔲 Graduated urgency (3 levels based on distance)

**Phase 2 (Enhanced Realism):**
- 🔲 IDM-based car following
- 🔲 Intersection-specific logic
- 🔲 Traffic light coordination
- 🔲 Vehicle-to-vehicle coordination

**Phase 3 (Advanced):**
- 🔲 MPC optimization
- 🔲 RL-based behavior
- 🔲 Learning from demonstrations
- 🔲 Multi-vehicle coordination protocol

