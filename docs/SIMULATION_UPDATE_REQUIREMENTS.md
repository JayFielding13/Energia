# Simulation Update Requirements

**Document Created:** November 28, 2025
**Purpose:** Comprehensive guide for Claude Code instance managing simulation development
**Status:** Action Required - Simulation needs updates to match real rover architecture

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Real Rover Architecture (Current)](#real-rover-architecture-current)
3. [Simulation Architecture (Needs Update)](#simulation-architecture-needs-update)
4. [Required Changes](#required-changes)
5. [Reference Files](#reference-files)
6. [Implementation Checklist](#implementation-checklist)
7. [Testing Verification](#testing-verification)

---

## Executive Summary

The real Jetson rover underwent significant architecture changes on **November 27, 2025**. The simulation needs to be updated to accurately reflect these changes for valid testing.

### Key Changes Made to Real Rover

| Aspect | OLD | NEW |
|--------|-----|-----|
| API Port | 5000 | **5001** |
| Motor Control | Direct MAVLink to Cube Orange | **MAVROS2 RC Override via MAVProxy** |
| Channel Mapping | Ch1=Steer, Ch3=Throttle | **Ch1=Throttle, Ch3=Steer** |
| Communication | pymavlink direct serial | **ROS2 topics → MAVROS2 → MAVProxy → Serial** |

### Why This Matters for Simulation

The simulation should replicate the real rover's behavior as closely as possible. While the simulation doesn't have a real Cube Orange flight controller, it should:
1. Use the same API endpoints and data formats
2. Respond to commands the same way
3. Simulate the same safety behaviors (timeouts, killswitch)
4. Allow testing of autonomous features before deploying to real hardware

---

## Real Rover Architecture (Current)

### Communication Stack (November 27, 2025)

```
Mobile RTK Module (Pi 5)
    │
    │  HTTP REST API (port 5001)
    ▼
┌─────────────────────────────────────────────────────────────┐
│  Jetson Orin Nano (100.91.191.47)                           │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  ros2_unified_bridge.py (Flask API on :5001)        │   │
│  │  - Receives HTTP commands (/api/velocity, etc.)     │   │
│  │  - Publishes to MAVROS2 RC Override topic           │   │
│  │  - 500ms safety timeout (auto-stops if no command)  │   │
│  └──────────────────────┬──────────────────────────────┘   │
│                         │ ROS2 Topics                       │
│                         │ /mavros/mavros/override           │
│                         ▼                                   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  MAVROS2 Node                                        │   │
│  │  - Converts RC Override to MAVLink                   │   │
│  │  - fcu_url: udp://:14550@                           │   │
│  └──────────────────────┬──────────────────────────────┘   │
│                         │ UDP :14550                        │
│                         ▼                                   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  MAVProxy (Serial-to-UDP Bridge)                     │   │
│  │  - Workaround for MAVROS2 serial deadlock bug        │   │
│  │  - --master=/dev/ttyACM0,57600                       │   │
│  │  - --out=udp:127.0.0.1:14550                         │   │
│  └──────────────────────┬──────────────────────────────┘   │
│                         │ Serial /dev/ttyACM0               │
└─────────────────────────┼───────────────────────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────────┐
│  Cube Orange+ Flight Controller                             │
│  - ArduRover V4.6.2                                         │
│  - Skid-steer 4WD configuration                             │
│  - RC Override for motor control                            │
│  - Channel 1 = Throttle, Channel 3 = Steering               │
└─────────────────────────────────────────────────────────────┘
```

### Motor Control Mapping (Verified November 27, 2025)

```
Channel 1 = THROTTLE (forward/reverse)
Channel 3 = STEERING (left/right rotation)

PWM Values:
  FORWARD:      Ch1=1600, Ch3=1500 (throttle forward, steer neutral)
  REVERSE:      Ch1=1400, Ch3=1500 (throttle reverse, steer neutral)
  ROTATE LEFT:  Ch1=1500, Ch3=1400 (throttle neutral, steer left)
  ROTATE RIGHT: Ch1=1500, Ch3=1600 (throttle neutral, steer right)
  STOP:         Ch1=1500, Ch3=1500 (both neutral)
```

### API Endpoints (Real Rover)

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/health` | GET | Health check |
| `/api/status` | GET | Robot status (GPS, armed, mode, battery) |
| `/api/arm` | POST | ARM motors |
| `/api/disarm` | POST | DISARM motors |
| `/api/velocity` | POST | Send velocity command `{linear, angular}` |
| `/api/velocity/release` | POST | Release RC override |
| `/api/stop` | POST | Emergency stop |

### Velocity Command Format

```json
{
    "linear": 0.5,    // -1.0 (reverse) to +1.0 (forward)
    "angular": 0.0    // -1.0 (right) to +1.0 (left)
}
```

### Safety Features

1. **500ms Velocity Timeout**: If no velocity command received, rover auto-stops
2. **Heltec LoRa Killswitch**: Hardware killswitch with 3-second heartbeat
3. **MAVROS Failsafe**: Flight controller watchdog
4. **Geofencing**: (Planned) Software boundary enforcement

---

## Simulation Architecture (Needs Update)

### Current Simulation Stack

```
Xbox Controller (Hardware) or HTTP API
         ↓
    joy_node (ROS2) / http_bridge.py
         ↓
    /cmd_vel topic (Twist)
         ↓
Gazebo Planar Move Plugin (libgazebo_ros_planar_move.so)
         ↓
Robot Motion + /odom topic
```

### Issues with Current Simulation

1. **No MAVROS simulation** - Uses direct Gazebo control, not RC Override
2. **Missing killswitch** - Cannot test emergency stop behaviors
3. **Hardcoded paths** - Launch files reference `~/Desktop/Mini Rover Development/...`
4. **No 500ms timeout** - Safety timeout not implemented in simulation bridge

---

## Required Changes

### Priority 1: Fix Hardcoded Paths (CRITICAL)

**File:** `ros2_ws/src/energia_sim/launch/full_simulation.launch.py`

**Current (Broken):**
```python
rviz_config_file = os.path.expanduser('~/Desktop/Mini Rover Development/Jetson Cube Orange Outdoor Rover/rover_sensors.rviz')
xbox_config_file = os.path.expanduser('~/Desktop/Mini Rover Development/Jetson Cube Orange Outdoor Rover/xbox_rover.config.yaml')
```

**Should Be:**
```python
# Use package-relative paths
package_dir = get_package_share_directory('energia_sim')
rviz_config_file = os.path.join(package_dir, 'config', 'rover_sensors.rviz')
xbox_config_file = os.path.join(package_dir, 'config', 'xbox_rover.config.yaml')
```

### Priority 2: Update HTTP Bridge to Match Real Rover

**File:** `shared/ros2_ws/src/energia_bridge/energia_bridge/http_bridge.py`

**Changes Needed:**
1. Add `/api/velocity` endpoint (may already exist, verify format matches)
2. Add `/api/velocity/release` endpoint
3. Implement 500ms safety timeout
4. Ensure response format matches real rover exactly

**Real Rover Response Format:**
```json
{
    "success": true,
    "linear": 0.5,
    "angular": 0.0,
    "message": "Velocity command sent"
}
```

### Priority 3: Add Simulated Killswitch

Create a simulated killswitch node that:
1. Monitors a "heartbeat" topic (simulating LoRa communication)
2. Publishes ARM/DISARM state
3. Can trigger emergency stop when heartbeat lost
4. Allows testing of safety behaviors

**Suggested Implementation:**
```python
# New file: shared/ros2_ws/src/energia_bridge/energia_bridge/simulated_killswitch.py

class SimulatedKillswitch(Node):
    def __init__(self):
        super().__init__('simulated_killswitch')
        self.heartbeat_timeout = 3.0  # seconds
        self.last_heartbeat = time.time()
        self.armed = False

        # Subscribe to simulated heartbeat
        self.heartbeat_sub = self.create_subscription(
            Empty, '/killswitch/heartbeat', self.heartbeat_callback, 10)

        # Publish armed state
        self.armed_pub = self.create_publisher(Bool, '/killswitch/armed', 10)

        # Timer to check heartbeat
        self.timer = self.create_timer(0.5, self.check_heartbeat)
```

### Priority 4: Align Motor Control Logic

The simulation uses `/cmd_vel` (Twist message) which is correct, but ensure the conversion matches real rover behavior:

**Real Rover Conversion (reference):**
```python
# From ros2_unified_bridge.py
def send_velocity(self, linear: float, angular: float):
    PWM_NEUTRAL = 1500
    DEADZONE = 0.1

    is_moving = abs(linear) > DEADZONE
    is_turning = abs(angular) > DEADZONE

    if not is_moving and not is_turning:
        throttle_pwm = PWM_NEUTRAL
        steering_pwm = PWM_NEUTRAL
    elif is_turning and not is_moving:
        # Pure rotation
        throttle_pwm = PWM_NEUTRAL
        steering_pwm = 1400 if angular > 0 else 1600
    elif is_moving and not is_turning:
        # Pure forward/reverse
        steering_pwm = PWM_NEUTRAL
        throttle_pwm = 1600 if linear > 0 else 1400
    else:
        # Combined movement
        throttle_pwm = 1600 if linear > 0 else 1400
        steering_pwm = int(PWM_NEUTRAL - (angular * 100))
```

---

## Reference Files

### Real Rover Repository

**Location:** `/home/jay/Git Sandbox/Jetson Cube Orange Outdoor Rover/`

| File | Description |
|------|-------------|
| `rover/jetson_stable/stable/scripts/ros2_unified_bridge.py` | Main HTTP API + MAVROS bridge |
| `docs/AUTONOMOUS_ARCHITECTURE.md` | Autonomous navigation architecture |

### Key Reference Documents

1. **Autonomous Architecture:** Defines MQTT topics, Follow-Me mode, waypoint missions, geofencing
   - Location: `../Jetson Cube Orange Outdoor Rover/docs/AUTONOMOUS_ARCHITECTURE.md`

2. **API Update Guide:** Documents all API changes made November 27, 2025
   - Location: `../Mobile RTK Control Module/Current Issues/JETSON_ROVER_API_UPDATE_NOV27_2025.md`

### Network Information

| Device | Tailscale IP | Local IP | Port |
|--------|--------------|----------|------|
| Jetson Orin Nano | 100.91.191.47 | 192.168.8.110 | 5001 |
| Mobile RTK Module (beaconpi) | 100.73.233.124 | - | - |
| RTK Base Station | - | 192.168.254.165 | 1883 (MQTT) |

---

## Implementation Checklist

### Phase 1: Critical Fixes
- [ ] Fix hardcoded paths in `full_simulation.launch.py`
- [ ] Move config files to package directory structure
- [ ] Test launch files work on fresh system

### Phase 2: API Alignment
- [ ] Verify `/api/velocity` endpoint format matches real rover
- [ ] Add `/api/velocity/release` endpoint
- [ ] Implement 500ms safety timeout in HTTP bridge
- [ ] Test with Mobile RTK Module (same commands should work)

### Phase 3: Safety Simulation
- [ ] Create `simulated_killswitch.py` node
- [ ] Add heartbeat topic subscription
- [ ] Implement ARM/DISARM state publishing
- [ ] Add emergency stop trigger on heartbeat loss
- [ ] Test safety behaviors match real rover

### Phase 4: Advanced Features
- [ ] Add MQTT simulation for position sharing
- [ ] Simulate GPS data in correct format
- [ ] Add geofence simulation
- [ ] Test autonomous features before real hardware

### Phase 5: Documentation
- [ ] Update README.md with new architecture
- [ ] Document all API endpoints
- [ ] Create testing guide
- [ ] Add troubleshooting section

---

## Testing Verification

### API Compatibility Test

Run these commands against both real rover and simulation - responses should be identical:

```bash
# Health check
curl http://localhost:5001/api/health

# Get status
curl http://localhost:5001/api/status

# ARM
curl -X POST http://localhost:5001/api/arm

# Send velocity (forward)
curl -X POST -H "Content-Type: application/json" \
  -d '{"linear": 0.5, "angular": 0.0}' \
  http://localhost:5001/api/velocity

# Stop
curl -X POST http://localhost:5001/api/stop

# DISARM
curl -X POST http://localhost:5001/api/disarm
```

### Safety Timeout Test

1. Send velocity command
2. Wait 600ms without sending another command
3. Verify rover/simulation has stopped

### Killswitch Test (After Implementation)

1. Start simulation with killswitch node
2. Verify rover responds to heartbeat
3. Stop sending heartbeat
4. Verify emergency stop triggers after 3 seconds

---

## Contact / Support

- **Real Rover Repository:** `/home/jay/Git Sandbox/Jetson Cube Orange Outdoor Rover/`
- **Mobile RTK Module Repository:** `/home/jay/Git Sandbox/Mobile RTK Control Module/`
- **This Simulation Repository:** `/home/jay/Git Sandbox/Energia Rover Simulation/`

**Note to Claude Code Instance:**
When making changes, always verify compatibility with the real rover by checking the reference files listed above. The goal is simulation accuracy - commands that work in simulation should work identically on real hardware.

---

*Document Version: 1.0*
*Created: November 28, 2025*
*For: Energia Rover Simulation Claude Code Instance*
