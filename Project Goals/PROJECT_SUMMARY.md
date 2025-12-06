# Jetson Outdoor Rover - Project Summary

**Created:** December 2, 2025
**Last Updated:** December 2, 2025
**Status:** Planning Complete, Ready for Implementation

---

## Executive Summary

This project is a **ground-up rebuild** of an autonomous outdoor rover, pivoting away from a complex Cube Orange + MAVROS + MAVProxy stack toward **direct Jetson-to-hardware control**. The simplified architecture eliminates middleware layers that caused recurring crashes, enabling all navigation, sensor fusion, and motor control to run on a single Jetson Orin Nano.

### Key Architectural Change

**Before (Complex - Problematic):**
```
Mobile RTK Module → Jetson → MAVROS → MAVProxy → Cube Orange → Motors
                            └── Multiple failure points, hard to debug
```

**After (Simplified - This Project):**
```
Mobile RTK Module → Jetson Orin Nano → CAN Bus → STM32 → MDDS30 → Motors
                    └── All logic here, single point of control
```

---

## Hardware Stack

| Component | Model | Interface | Status |
|-----------|-------|-----------|--------|
| Main Computer | Jetson Orin Nano | - | Have |
| GPS/IMU | SparkFun ZED-F9R RTK Dead Reckoning Kit | USB (UBX protocol) | Ordered |
| Motor Controller MCU | Nucleo STM32 + CAN transceiver | CAN bus | Ordered |
| CAN Adapter | USB-to-CAN | USB | Ordered |
| Motor Driver | MDDS30 (dual brushed, Mixed R/C mode) | PWM from STM32 | Have |
| Killswitch | Heltec LoRa V3 (Gen 2) | Serial to Jetson | Have |
| Ultrasonic Sensors | HC-SR04 (x4) | Via Arduino to Jetson | Have |

### Physical Specifications

| Property | Value |
|----------|-------|
| Chassis | 27" × 23.75" × 12" (0.69m × 0.60m × 0.30m) |
| Weight | 200 lbs (90.7 kg) |
| Wheels | 10" diameter, 3" wide, 4WD skid-steer |
| Wheelbase | 17" (0.43m) |
| Ground clearance | 3" |

---

## Network Architecture

All devices communicate via **Tailscale VPN** mesh network for reliable connectivity from anywhere.

### Device Registry

| Device | Hostname | Tailscale IP | Home IP | Purpose |
|--------|----------|--------------|---------|---------|
| Jetson Orin Nano | `jetson1` | `100.91.191.47` | `192.168.254.100` | Rover brain |
| Mobile RTK Terminal | `beaconpi` | `100.73.233.124` | `192.168.254.127` | Handheld control |
| RTK Base Station | `rtkpi` | `100.66.67.11` | `192.168.254.165` | MQTT broker + RTCM3 |
| Ubuntu Desktop | `jay-desktop` | `100.73.129.15` | `192.168.254.66` | Development |
| VPN Pi | `vpnpi` | `100.69.252.111` | `192.168.254.134` | Subnet router + Pi-hole |

### Communication Protocols

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              MQTT BROKER (rtkpi @ 100.66.67.11:1883)            │
│                                                                                 │
│   rtk/base/corrections  ◄── RTK Base Station (RTCM3 binary, continuous)        │
│   rtk/mobile/position   ◄── Mobile RTK Module (JSON, 5Hz)                      │
│   rtk/rover/position    ◄── Jetson Rover (JSON, 10Hz)                          │
│   rtk/mission/status    ◄── Jetson Rover (navigation state updates)            │
└─────────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────────┐
│                         HTTP REST API (jetson1 @ 100.91.191.47:5001)            │
│                                                                                 │
│   GET  /api/health       - Connection test                                      │
│   GET  /api/status       - Rover state (GPS, armed, mode)                       │
│   POST /api/arm          - Enable motors                                        │
│   POST /api/disarm       - Disable motors                                       │
│   POST /api/target       - Send GPS waypoint {latitude, longitude}              │
│   POST /api/velocity     - Direct joystick control {linear, angular}            │
│   POST /api/return_to_me - Enable follow mode {enabled, distance}               │
│   POST /api/stop         - Emergency stop                                       │
│   POST /api/pause        - Stop but stay armed                                  │
│   POST /api/cancel       - Cancel current mission                               │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### MQTT Message Formats

**Mobile Terminal Position (`rtk/mobile/position`):**
```json
{
  "device": "mobile",
  "latitude": 45.123456,
  "longitude": -122.654321,
  "altitude": 100.5,
  "fix_type": 6,
  "satellites": 14,
  "hdop": 0.8,
  "heading": 180.0,
  "speed": 1.2,
  "timestamp": 1698345678.123
}
```

---

## Project Structure

```
Jetson Outdoor Rover/
├── PROJECT_SUMMARY.md              # This document
├── SESSION_HANDOFF.md              # Original handoff documentation
│
├── simulation/                     # Gazebo simulation environment
│   ├── ros2_ws_src/
│   │   └── jetson_rover_sim/
│   │       ├── urdf/
│   │       │   ├── jetson_rover.urdf.xacro      # Robot model (updated for Jetson-only)
│   │       │   ├── jetson_rover_gazebo.xacro    # Gazebo plugins
│   │       │   └── sensors/
│   │       │       ├── ultrasonic_sensors.xacro # 6x ultrasonic
│   │       │       └── lidar_camera_gps.xacro   # LiDAR, camera, ZED-F9R GPS/IMU
│   │       ├── launch/
│   │       │   └── full_simulation.launch.py
│   │       └── worlds/
│   │           └── test_yard.world
│   ├── bridge_src/
│   │   └── jetson_rover_bridge/
│   │       └── http_bridge.py                   # Flask HTTP ↔ ROS2 bridge
│   └── examples/
│       ├── sensor_monitor.py
│       └── simple_avoid.py
│
├── reference/                      # Proven code from 5 previous projects
│   ├── api/
│   │   └── ros2_unified_bridge.py              # Flask API reference
│   ├── config/
│   │   └── config.py                           # Navigation parameters
│   ├── gps_rtk/
│   │   └── gps_rtk_handler.py                  # GPS + RTK handling
│   ├── motor_control/
│   │   ├── arduino_motor_control.ino           # MDDS30 Mixed R/C mode
│   │   └── WIRING_GUIDE.md
│   ├── obstacle_avoidance/
│   │   └── bluetooth_optimized_navigator_v6.py # Navigation state machine
│   └── killswitch/
│       └── (LoRa killswitch reference)
│
└── firmware/
    ├── heltec_killswitch/          # LoRa wireless emergency stop (complete)
    │   ├── Receiver/
    │   └── Transmitter/
    └── nucleo_motor_ctrl/          # STM32 motor controller (to be created)
```

---

## Milestone 1: Waypoint Navigation + Return-to-Me

### Goal

Navigate to a GPS waypoint designated by the Mobile RTK Control Module, then "return to me" (follow the terminal's GPS position) and stop 2 meters away.

### Scope

| Feature | Included | Notes |
|---------|----------|-------|
| Navigate to waypoint | ✅ | Stop within 0.5m of target |
| Return to Me | ✅ | Stop 2m from terminal, track moving position |
| Manual joystick control | ✅ | Already exists via `/api/velocity` |
| Obstacle avoidance | ❌ | Deferred - test in open field first |

### State Machine

```
                    ┌──────────┐
                    │   IDLE   │
                    └────┬─────┘
                         │ /api/target OR /api/return_to_me
                         ▼
                    ┌──────────┐
         ┌─────────│ ROTATING │◄────────┐
         │         └────┬─────┘         │
         │              │ facing target │ target moved significantly
         │              ▼               │
         │         ┌──────────┐         │
         │         │ DRIVING  │─────────┘
         │         └────┬─────┘
         │              │ distance < threshold
         │              ▼
         │         ┌──────────┐
         └────────►│ ARRIVED  │
          /api/stop└──────────┘
```

### Key Parameters

| Parameter | Value |
|-----------|-------|
| Waypoint arrival threshold | 0.5m |
| Return-to-me standoff distance | 2.0m |
| Max linear speed | 0.7 m/s |
| Max angular speed | 0.8 rad/s |
| GPS update rate | 10 Hz |
| Navigation loop rate | 10 Hz |

---

## Sequential Development Goals

Work through these goals in order. Each builds on the previous.

### Phase 1: Simulation Foundation

| # | Goal | Description | Validation |
|---|------|-------------|------------|
| 1.1 | **Build simulation workspace** | Set up ROS2 workspace, compile URDF, verify Gazebo launches | Rover spawns in Gazebo, sensors publish data |
| 1.2 | **Verify sensor topics** | Confirm `/gps/fix`, `/imu/data`, `/cmd_vel` work | Echo topics, see valid data |
| 1.3 | **Test HTTP bridge** | Launch `http_bridge.py`, verify API endpoints | `curl` commands control rover in Gazebo |
| 1.4 | **Manual control test** | Send `/api/velocity` commands, rover moves | Rover responds to joystick-style inputs |

### Phase 2: Navigation Logic (Simulation)

| # | Goal | Description | Validation |
|---|------|-------------|------------|
| 2.1 | **Implement GPS utilities** | Haversine distance, bearing calculation | Unit tests pass |
| 2.2 | **Create navigation node** | State machine (IDLE → ROTATING → DRIVING → ARRIVED) | State transitions work in isolation |
| 2.3 | **Waypoint navigation** | Navigate to hardcoded GPS coordinate | Rover drives to point in Gazebo |
| 2.4 | **HTTP waypoint integration** | Accept waypoint via `/api/target` | Send waypoint via curl, rover navigates |

### Phase 3: MQTT Integration (Simulation)

| # | Goal | Description | Validation |
|---|------|-------------|------------|
| 3.1 | **MQTT subscriber** | Subscribe to `rtk/mobile/position` | Receive position updates from real terminal |
| 3.2 | **Return-to-Me mode** | Track terminal position, maintain 2m standoff | Rover follows simulated/real terminal |
| 3.3 | **Position publisher** | Publish rover position to `rtk/rover/position` | Terminal shows rover on map |
| 3.4 | **Full simulation test** | Connect real Mobile RTK Module to Gazebo simulation | End-to-end: terminal → MQTT → Gazebo rover |

### Phase 4: Hardware Preparation (Parallel with Phase 2-3)

| # | Goal | Description | Validation |
|---|------|-------------|------------|
| 4.1 | **STM32 firmware** | CAN receive → PWM generation | Bench test with logic analyzer |
| 4.2 | **CAN bus setup** | USB-to-CAN adapter, message format | Send test messages, STM32 responds |
| 4.3 | **Motor abstraction** | Create interface that works for sim (`/cmd_vel`) and real (CAN) | Same code controls both |
| 4.4 | **Bench integration** | Jetson → CAN → STM32 → MDDS30 → motor spin | Motors respond to Jetson commands |

### Phase 5: Hardware Integration

| # | Goal | Description | Validation |
|---|------|-------------|------------|
| 5.1 | **ZED-F9R driver** | Read GPS/IMU via `pyubx2`, publish to same topics | Real GPS data matches simulation interface |
| 5.2 | **RTK corrections** | Subscribe to `rtk/base/corrections`, forward to ZED-F9R | RTK fix achieved |
| 5.3 | **Killswitch integration** | LoRa heartbeat, emergency stop | Killswitch stops motors within 100ms |
| 5.4 | **Full hardware test** | All systems connected on rover chassis | Systems communicate correctly |

### Phase 6: Field Testing

| # | Goal | Description | Validation |
|---|------|-------------|------------|
| 6.1 | **Manual control** | Joystick control in open field | Rover responds smoothly |
| 6.2 | **Waypoint navigation** | Navigate to marked point | Arrives within 0.5m |
| 6.3 | **Return-to-Me** | Follow terminal at 2m standoff | Rover maintains distance |
| 6.4 | **Edge cases** | GPS dropout, communication loss | Fails safely (stops) |

---

## Reference: Related Projects

| Project | Location | Purpose |
|---------|----------|---------|
| Jetson Rover Simulation | `~/Git Sandbox/Jetson Rover Simulation` | Original simulation (legacy/sandbox) |
| Mobile RTK Control Module | `~/Git Sandbox/Mobile RTK Control Module` | Handheld terminal software |
| Static RTK Base Station | `~/Git Sandbox/Static RTK Base Station` | RTK corrections via MQTT |
| VPN Configuration | `~/Git Sandbox/VPN Configuration` | Tailscale mesh network setup |

---

## Reference: Existing Code to Leverage

### Navigation State Machine
- **Source:** `reference/obstacle_avoidance/bluetooth_optimized_navigator_v6.py`
- **Lines:** 1,238
- **Key features:** State machine, GPS waypoint navigation, speed ramping

### Motor Control (MDDS30)
- **Source:** `reference/motor_control/arduino_motor_control.ino`
- **PWM values:** 1500 = center, 1700 = forward, 1300 = backward
- **To adapt:** Change from serial input to CAN input

### HTTP Bridge
- **Source:** `simulation/bridge_src/jetson_rover_bridge/http_bridge.py`
- **Port:** 5001
- **Already implements:** `/api/status`, `/api/arm`, `/api/target`, `/api/stop`

### GPS Position Handler
- **Source (terminal):** `Mobile RTK Control Module/Dashboard Development/Current Stable Dashboard Software/dependencies/mqtt_position_handler.py`
- **Topics:** `rtk/mobile/position`, `rtk/rover/position`
- **Publish rate:** 5 Hz

---

## Development Environment

### Software Requirements

**Jetson Orin Nano:**
- Ubuntu 22.04 (JetPack)
- ROS2 Humble
- Python 3.10+
- `pyubx2` - ZED-F9R UBX protocol
- `python-can` - CAN bus communication
- `flask` - REST API
- `paho-mqtt` - MQTT client

**Desktop (Simulation):**
- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic (11.x)
- Same Python packages as Jetson

### Build Commands (Simulation)

```bash
# Source ROS2 first
source /opt/ros/humble/setup.bash

# Navigate to simulation directory
cd ~/Git\ Sandbox/Jetson\ Outdoor\ Rover/simulation

# Run setup script (creates workspace, links packages, builds)
./setup_workspace.sh

# Source the workspace
source ros2_ws/install/setup.bash

# Launch minimal simulation (no joystick required)
ros2 launch jetson_rover_sim gazebo_only.launch.py

# Or launch full simulation with RViz and joystick
ros2 launch jetson_rover_sim full_simulation.launch.py
```

---

## Quick Reference: Key Commands

### Simulation

```bash
# Launch minimal simulation (Gazebo only, no joystick needed)
ros2 launch jetson_rover_sim gazebo_only.launch.py

# Launch full simulation with RViz and joystick
ros2 launch jetson_rover_sim full_simulation.launch.py

# Check available topics
ros2 topic list

# Check sensor topics
ros2 topic echo /gps/fix
ros2 topic echo /imu/data
ros2 topic echo /scan  # LiDAR
ros2 topic echo /ultrasonic/front  # Ultrasonic sensors

# Send velocity command (test motor control)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.0}}' --once

# Test HTTP API (requires http_bridge node running)
curl http://localhost:5001/api/health
curl -X POST http://localhost:5001/api/arm
curl -X POST http://localhost:5001/api/target -H "Content-Type: application/json" \
  -d '{"latitude": 45.0, "longitude": -122.0}'
```

### MQTT

```bash
# Monitor all RTK topics
mosquitto_sub -h 100.66.67.11 -t "rtk/#" -v

# Monitor mobile terminal position
mosquitto_sub -h 100.66.67.11 -t "rtk/mobile/position"

# Publish test rover position
mosquitto_pub -h 100.66.67.11 -t "rtk/rover/position" -m '{"latitude": 45.0, "longitude": -122.0}'
```

### Network

```bash
# Check Tailscale status
tailscale status

# Ping devices
ping jetson1  # or 100.91.191.47
ping rtkpi    # or 100.66.67.11
ping beaconpi # or 100.73.233.124
```

---

## Notes

- **Obstacle avoidance deferred** - Will be added after basic navigation is validated in open field
- **Simulation validates logic** - Gazebo testing provides confidence before real hardware
- **Same code for sim/real** - Motor abstraction layer allows identical navigation code
- **Tailscale enables remote development** - Can test from anywhere with internet

---

## Document History

| Date | Changes |
|------|---------|
| 2025-12-02 | Initial creation from planning session |
