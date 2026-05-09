# Emergency Vehicle Path Clearing Algorithms

## Overview
NPC vehicles detect emergency vehicles (ambulance) and automatically adjust their driving behavior to clear a path. This document describes the algorithms implemented and their alternatives.

---

## 1. Lane-Shifting Algorithm (IMPLEMENTED ✅)

### Principle
Vehicles shift laterally across their lane without stopping, maintaining forward progress while moving aside for the emergency vehicle.

### Key Characteristics
- **Detection**: Ambulance within 60m (baseline) or 150m (V2X)
- **Response Mechanism**: 
  - Reduce speed 50% (normal roads) or 80% (intersections)
  - Apply lateral steering offset proportional to clearance distance
  - Typical offset: 3 meters to the right (or left if unavailable)
- **Steering Calculation**:
  ```python
  lateral_steering = clip(lane_offset / 2.0, -0.3, 0.3)  # ±0.3 radians
  control.steer = clip(original_steer + lateral_steering, -1.0, 1.0)
  ```

### Advantages
✅ Realistic (matches Tesla/Waymo behavior)  
✅ Maintains traffic flow  
✅ Requires no special infrastructure  
✅ Works with existing navigation systems  
✅ Smooth, continuous motion  

### Disadvantages
❌ Depends on road width (limited on narrow streets)  
❌ May not work if lanes are full  
❌ Requires accurate steering control  

### Real-World Usage
- **Tesla Autopilot**: Shifts lanes when emergency vehicle detected (audio/visual)
- **Waymo**: Lane centering + lateral offset when vehicles detected ahead
- **European Traffic Standards (EN 28010)**: Recommended for emergency corridors

---

## 2. Pull-Over Strategy (BASELINE)

### Principle
Vehicles slow down significantly and move to road shoulder/edge, stopping if necessary.

### Key Characteristics
- **Detection**: Visual/audio cues (proximity-based)
- **Response**:
  - Reduce speed by 50-80%
  - Move to right edge of road
  - Stop and wait for ambulance to pass
  - Resume route after ambulance passes
- **Exit Condition**: When ambulance distance > 60m

### Advantages
✅ Very predictable  
✅ Works in all road conditions  
✅ Simple to implement  
✅ Used by many drivers in real world  

### Disadvantages
❌ Blocks entire lane temporarily  
❌ Slower ambulance progress  
❌ Can cause traffic jams  
❌ May prevent following vehicles from exiting  

### Real-World Usage
- **Human drivers**: Default behavior (taught in driving schools)
- **Early autonomous systems**: Conservative approach
- **Motorcycles**: Preferred method on highways

---

## 3. Lane-Changing Strategy (ALTERNATIVE)

### Principle
Vehicles perform aggressive lane change (like overtaking) to create space.

### Key Characteristics
- **Response**:
  - Set steer = ±0.8 (aggressive angle)
  - Reduce speed temporarily (to control trajectory)
  - Move to opposite lane
  - Resume speed after clearing
- **Timing**: Change happens over 2-3 seconds

### Advantages
✅ Faster path clearing  
✅ Vehicles clear completely to opposite lane  
✅ Works well on multi-lane roads  

### Disadvantages
❌ Risk of collision with oncoming traffic  
❌ Harder to control (unstable)  
❌ May cause safety issues  
❌ Requires lane detection  

### Real-World Usage
- **Emergency vehicles themselves**: Use aggressive lane changes
- **Ambulances in congestion**: Force their way through
- **Not recommended for NPC vehicles**: Too risky

---

## 4. Corridor Formation Strategy (ADVANCED)

### Principle
Multiple vehicles coordinate (via V2X) to form a "corridor" - like a Mexican Wave pattern.

### Key Characteristics
- **Detection**: V2X broadcast with trajectory prediction
- **Coordination**:
  - Lead vehicle in path slows down and pulls right
  - Following vehicles shift left while slowing
  - Creates alternating left-right pattern
  - Ambulance gets clear path through middle
- **Timing**: Pre-positioned before ambulance arrives (predictive)

### Advantages
✅ Most efficient use of road space  
✅ Ambulance rarely needs to brake  
✅ Works in heavy traffic  
✅ Scalable to many vehicles  

### Disadvantages
❌ Requires V2X communication  
❌ Complex coordination logic  
❌ Needs trajectory prediction  
❌ May require new infrastructure  

### Real-World Usage
- **Research**: Studied in automated highway systems
- **SARTRE Project**: Truck platooning demonstrations
- **Future ITS**: Connected vehicle systems (5G)
- **Not yet deployed**: Prototype stage

---

## 5. Traffic Light Clearing (HYBRID)

### Principle
Ambulance broadcasts emergency status; traffic lights switch to green for ambulance path.

### Key Characteristics
- **Ambulance sends**: V2X signal with location/heading
- **Intersection responds**:
  - Clears cross traffic (red lights)
  - Extends green for ambulance lane
  - Vehicles detect traffic light change and slow down
- **Fallback**: If traffic light fails, use lane-shifting

### Advantages
✅ Fastest path clearing  
✅ No vehicle coordination needed  
✅ Infrastructure-based (not vehicle-dependent)  
✅ Deployed in many cities  

### Disadvantages
❌ Requires traffic light infrastructure  
❌ Expensive to implement  
❌ Not available everywhere  
❌ Privacy concerns (vehicle tracking)  

### Real-World Usage
- **Copenhagen, Denmark**: Ambulance-adaptive traffic lights
- **Germany, Austria**: Emergency vehicle priority systems
- **USA**: Mostly manual (first responders control signals)

---

## Performance Comparison

| Metric | Lane-Shift | Pull-Over | Lane-Change | Corridor | Traffic Light |
|--------|-----------|-----------|------------|----------|-----------------|
| Speed | ⭐⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| Safety | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ |
| Feasibility | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐ |
| Simplicity | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐ | ⭐⭐ |
| Dep. on Infra | ❌ | ❌ | ❌ | ⚠️ V2X | ✅ Traffic Lights |

---

## Implementation in This Project

### Current Implementation: Lane-Shifting + Pull-Over Hybrid

**Baseline System (60m proximity):**
```
IF ambulance_nearby AND ambulance_behind THEN:
  IF at_intersection THEN:
    slow_down(0.2x)  # 80% reduction
  ELSE:
    slow_down(0.5x)  # 50% reduction
    shift_lane(3.0)  # Lateral offset
```

**V2X System (150m broadcast):**
```
IF v2x_signal_received THEN:
  urgency = calculate_urgency(distance, time_gap)
  IF urgency == CRITICAL (<30m) THEN:
    slow_down(0.2x)
    shift_lane(±3.0)  # Aggressive
  ELIF urgency == CLEARING (30-100m) THEN:
    slow_down(0.5x)
    shift_lane(±2.0)  # Moderate
  ELSE:
    slow_down(0.8x)
    shift_lane(±1.0)  # Light
```

### Why Lane-Shifting?
1. **Balance**: Combines safety (slow down) + efficiency (keep moving)
2. **Realistic**: Matches human driver + autonomous vehicle behavior
3. **Scalable**: Works in light to moderate traffic (not gridlock)
4. **Robust**: Gracefully degrades if lane unavailable

---

## Recommendations for Future Work

1. **Improve Lane Detection**:
   - Query road markings/lane boundaries
   - Prevent shifting if no space available
   - Warn if corridor too narrow

2. **Emergency Maneuver Detection**:
   - Detect vehicle stuck behind obstacle
   - Trigger more aggressive lane change (steer=±0.8)
   - Exit lane change only when clear

3. **Multi-Vehicle Coordination**:
   - Implement corridor formation for heavy traffic
   - Use V2X to broadcast "clear next lane"
   - Synchronize shifting patterns

4. **Intersection Optimization**:
   - Query if traffic light is controllable
   - Request green light for ambulance
   - Skip speed reduction if light already green

5. **Machine Learning**:
   - Learn optimal lateral offset per road type
   - Predict best moment to shift lanes
   - Adapt to vehicle type/size

---

## References

- **EN 28010**: European parking and stopping guidance for emergency vehicles
- **SARTRE Project**: Safe Road Trains for the Environment (EU platooning research)
- **IEEE 802.11p**: Wireless standard for V2X communication
- **Tesla Autopilot**: Lane detection + lateral offset when vehicles ahead
- **Waymo Studies**: Emergency vehicle response in autonomous fleets

---

## Code Location

- **Vehicle Behavior**: [v2x_simulation/vehicle_behavior.py](vehicle_behavior.py) (Lines 428-450)
- **Baseline Behavior**: [v2x_simulation/baseline_vehicle_behavior.py](baseline_vehicle_behavior.py) (Lines 208-245)
- **Configuration**: [v2x_simulation/config.py](config.py) (Lane offset and detection ranges)

---

*Last Updated: 2026-02-23*
*Algorithm: Lane-Shifting (Industry Standard)*
*Status: ✅ Implemented and tested*
