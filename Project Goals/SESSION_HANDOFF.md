# Session Handoff: Jetson Outdoor Rover
**Created:** December 2, 2025

---

## Project Overview

This is a **ground-up rebuild** of an outdoor autonomous rover, pivoting away from a complex Cube Orange + MAVROS + MAVProxy stack that suffered from recurring crashes, ROS2 DDS conflicts, and maintenance headaches. The new architecture prioritizes **simplicity and reliability**.

### Why We Pivoted
The previous system (documented in `/home/jay/Git Sandbox/Jetson Cube Orange Outdoor Rover/`) used:
- Cube Orange flight controller running ArduRover
- MAVROS2 for ROS2 ↔ MAVLink translation
- MAVProxy as a serial-to-UDP bridge
- HERE3+ RTK GPS

**Problems encountered:**
- MAVROS plugins crashed due to ROS2 DDS topic conflicts
- MAVProxy would hang or lose connection
- Complex 4-layer stack (App → Bridge → MAVROS → MAVProxy → Cube Orange)
- Hours spent debugging middleware instead of building features

### New Architecture Goals
1. **Eliminate middleware complexity** - Jetson talks directly to sensors and motor controller
2. **Use proven, working code** - Reuse motor control and navigation code that worked in earlier projects
3. **Modern GPS/IMU fusion** - ZED-F9R handles sensor fusion internally
4. **CAN bus for reliability** - Industrial-grade communication to motor controller

---

## New Hardware Stack

| Component | Model | Purpose | Interface | Status |
|-----------|-------|---------|-----------|--------|
| Main Computer | Jetson Orin Nano | All high-level logic, navigation, API | - | Have |
| GPS/IMU | SparkFun RTK Dead Reckoning Kit (ZED-F9R) | RTK GPS + IMU + compass + dead reckoning + sensor fusion | USB (UBX protocol) | Ordered |
| Motor Controller MCU | Nucleo STM32 + CAN transceiver | Receives commands, generates PWM | CAN bus | Ordered |
| CAN Adapter | USB-to-CAN | Jetson to CAN bus | USB | Ordered |
| Motor Driver | MDDS30 | Dual brushed motor driver (Mixed R/C mode) | PWM from STM32 | Have |
| Killswitch | Heltec LoRa V3 (Gen 2) | Wireless emergency stop with ACK | Serial to Jetson | Have |
| Ultrasonic Sensors | HC-SR04 (x4) | Obstacle detection | Via Arduino to Jetson | Have |

---

## Reference Code Library

All code in `/reference/` comes from **5 previous rover projects** and represents working, tested implementations. Here's what each piece does and how to use it:

### Motor Control (`/reference/motor_control/`)

#### `arduino_motor_control.ino`
**Origin:** Jetson Simple Rover (working indoor rover)
**Purpose:** Receives serial commands, outputs PWM to MDDS30 motor driver

```cpp
// MDDS30 is in MIXED R/C mode:
// - Channel 1 (D6): THROTTLE - controls forward/backward for both motors
// - Channel 2 (D5): STEERING - adds differential for turning
//
// PWM Values:
//   1500 = stopped (center)
//   1700 = full forward / full right turn
//   1300 = full backward / full left turn

// Serial Commands (115200 baud):
//   F150  → Forward at speed 150 (0-255)
//   B100  → Backward at speed 100
//   L80   → Turn left at speed 80
//   R80   → Turn right at speed 80
//   S     → Stop (both channels to 1500)
```

**For new architecture:** This Arduino code will be ported to STM32 with CAN input instead of serial.

#### `bluetooth_optimized_navigator_v6.py`
**Origin:** Pi-based Bluetooth rover (extensively tested)
**Purpose:** Complete navigation system with obstacle avoidance

Key features to reuse:
- JSON motor command protocol
- Navigation state machine (IDLE → SEARCHING → TRACKING → APPROACHING → HOLDING)
- Progressive obstacle avoidance
- GPS waypoint navigation
- Speed ramping for smooth motion

```python
# Motor command format (JSON over serial):
{"motor": {"left": 150, "right": 150}}   # Forward
{"motor": {"left": -100, "right": 100}}  # Spin left
{"motor": {"left": 0, "right": 0}}       # Stop

# Navigation states:
class NavigationState(Enum):
    IDLE = "idle"           # Waiting for target
    SEARCHING = "searching" # Looking for GPS lock
    TRACKING = "tracking"   # Moving toward waypoint
    APPROACHING = "approaching"  # Close to target, slowing down
    HOLDING = "holding"     # At target, maintaining position
    AVOIDING = "avoiding"   # Obstacle detected, maneuvering
    RECOVERING = "recovering"  # Returning to path after avoidance
```

### API Layer (`/reference/api/`)

#### `ros2_unified_bridge.py`
**Origin:** Jetson Cube Orange project
**Purpose:** Flask REST API for remote control

Endpoints to keep:
- `GET /api/status` - Rover state, GPS, battery, mode
- `POST /api/arm` / `/api/disarm` - Enable/disable motors
- `POST /api/target` - Send GPS waypoint `{"latitude": X, "longitude": Y}`
- `POST /api/velocity` - Manual joystick control
- `POST /api/stop` - Emergency stop

**For new architecture:** Strip out all ROS2/MAVROS code, keep Flask API structure.

#### `robot_controller.py`
**Origin:** Mobile RTK Control Module
**Purpose:** HTTP client class for sending commands from mobile app to rover

### GPS & RTK (`/reference/gps_rtk/`)

#### `gps_rtk_handler.py`
**Origin:** Jetson Cube Orange project
**Purpose:** NMEA parsing + MQTT subscription for RTK corrections

**For new architecture:** Replace NMEA parsing with `pyubx2` for ZED-F9R's UBX protocol. RTK correction injection pattern is still valid.

### Sensors (`/reference/sensors/`)

#### `sensor_interface.ino` + `arduino_sensor_interface.py`
**Purpose:** Arduino reads 4x HC-SR04 ultrasonic sensors, sends CSV over serial

```
# Serial output format (115200 baud):
# LF,RF,LS,RS (distances in cm, -1 for timeout/error)
# Example: 45,52,120,95

# Pin mapping:
# Left Front:  TRIG=2, ECHO=3
# Right Front: TRIG=4, ECHO=5
# Left Side:   TRIG=6, ECHO=7
# Right Side:  TRIG=8, ECHO=9
```

### Killswitch (`/firmware/heltec_killswitch/`)

Complete LoRa-based wireless killswitch system:
- **Transmitter:** Handheld unit with button, sends kill/arm commands
- **Receiver:** On rover, receives commands, controls relay/signal to motor controller
- **Protocol:** Commands include sequence numbers and ACK responses for reliability

---

## Software Development Roadmap

### Phase 1: Simulation & Core Logic (No Hardware Needed)

**Goal:** Port navigation code to new architecture, test in Gazebo

| Task | Description | Test Method |
|------|-------------|-------------|
| Create Gazebo rover model | Differential drive robot with simulated sensors | Spawn in Gazebo |
| Port navigation state machine | Extract from `bluetooth_optimized_navigator_v6.py` | Unit tests |
| Implement motor command interface | Abstract class that can target sim or real hardware | Mock tests |
| Port obstacle avoidance | Progressive avoidance from reference code | Gazebo obstacles |
| Create Flask API | Based on `ros2_unified_bridge.py` (no ROS2 deps) | curl/Postman |
| GPS waypoint navigation | Simulate GPS coordinates in Gazebo | Scripted waypoints |

**Simulation allows testing:**
- Navigation state transitions
- Obstacle avoidance behavior
- API endpoint functionality
- Motor command generation
- Waypoint arrival detection

### Phase 2: Hardware Drivers (Unit Testing)

**Goal:** Write and test drivers for each hardware component individually

| Task | Description | Test Method |
|------|-------------|-------------|
| ZED-F9R driver | `pyubx2` parsing for position, velocity, heading, RTK status | USB to module, print parsed data |
| CAN bus layer | `python-can` wrapper for motor commands | Loopback test or logic analyzer |
| STM32 firmware | CAN receive → PWM output to MDDS30 | Bench test with oscilloscope |
| Killswitch integration | Serial protocol to Heltec receiver | Test kill/arm commands |
| Ultrasonic integration | Parse CSV from Arduino sensor interface | Verify distances |

**Libraries to install:**
```bash
pip3 install pyubx2        # ZED-F9R UBX protocol
pip3 install python-can    # CAN bus communication
pip3 install flask         # REST API
pip3 install pyserial      # Serial devices
```

### Phase 3: Integration Testing (Bench)

**Goal:** Connect all components, test full data flow without wheels on ground

| Task | Description | Success Criteria |
|------|-------------|------------------|
| End-to-end command flow | API → Jetson → CAN → STM32 → MDDS30 | Motor responds to API calls |
| Sensor fusion | ZED-F9R heading + velocity into navigation | Correct heading calculations |
| Killswitch safety | Kill command stops motors within 100ms | Measured response time |
| Obstacle detection | Ultrasonic → avoidance → motor adjustment | Motors respond to obstacles |

### Phase 4: Field Testing

**Goal:** Outdoor testing with full system

| Task | Description | Success Criteria |
|------|-------------|------------------|
| RTK GPS accuracy | Compare to known points | < 2cm accuracy with RTK fix |
| Dead reckoning | GPS dropout recovery | Maintains position estimate |
| Waypoint navigation | Navigate to GPS coordinates | Arrives within 0.5m |
| Obstacle course | Navigate around obstacles | No collisions |
| Long-duration test | 30+ minute continuous operation | No crashes or hangs |

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      JETSON ORIN NANO                           │
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │  Flask API   │  │  Navigation  │  │   Sensor Fusion      │  │
│  │  (Port 5001) │  │  State       │  │   (ZED-F9R data)     │  │
│  │              │  │  Machine     │  │                      │  │
│  └──────┬───────┘  └──────┬───────┘  └──────────┬───────────┘  │
│         │                 │                     │              │
│         └─────────────────┼─────────────────────┘              │
│                           │                                    │
│                           ▼                                    │
│         ┌─────────────────────────────────────┐                │
│         │       Motor Command Generator       │                │
│         │  CAN Message: ID=0x100, [L_HI,      │                │
│         │               L_LO, R_HI, R_LO]     │                │
│         └─────────────────┬───────────────────┘                │
│                           │                                    │
│  ┌────────────────────────┼────────────────────────────────┐   │
│  │ USB Devices:           │                                │   │
│  │  • /dev/ttyUSB0 - USB-to-CAN adapter                    │   │
│  │  • /dev/ttyACM0 - ZED-F9R GPS/IMU                       │   │
│  │  • /dev/ttyUSB1 - Heltec Killswitch                     │   │
│  │  • /dev/ttyUSB2 - Arduino Sensors                       │   │
│  └────────────────────────┼────────────────────────────────┘   │
└───────────────────────────┼────────────────────────────────────┘
                            │
                   ─────────┴─────────
                  │     CAN BUS       │
                   ───────────────────
                            │
              ┌─────────────┴─────────────┐
              │      NUCLEO STM32         │
              │   • CAN RX interrupt      │
              │   • Parse motor commands  │
              │   • Generate PWM signals  │
              │   • Watchdog timeout      │
              └─────────────┬─────────────┘
                            │ PWM (1000-2000μs)
                            ▼
              ┌───────────────────────────┐
              │         MDDS30            │
              │   Mixed R/C Mode          │
              │   • Ch1: Throttle (D6)    │
              │   • Ch2: Steering (D5)    │
              └─────────────┬─────────────┘
                            │
                   ┌────────┴────────┐
                   │                 │
                   ▼                 ▼
              [LEFT MOTOR]     [RIGHT MOTOR]
```

---

## File Structure

```
Jetson Outdoor Rover/
├── SESSION_HANDOFF.md          ← You are here
├── README.md                   ← Project overview (to create)
│
├── reference/                  ← Proven code from previous projects
│   ├── api/
│   │   ├── ros2_unified_bridge.py    # Flask API (strip ROS2 parts)
│   │   └── robot_controller.py       # HTTP client
│   ├── motor_control/
│   │   ├── arduino_motor_control.ino # MDDS30 serial control
│   │   ├── bluetooth_optimized_navigator_v6.py  # Full navigation
│   │   └── CubeOrange_to_Drok_Interface.ino     # DROK reference
│   ├── gps_rtk/
│   │   └── gps_rtk_handler.py        # RTK injection pattern
│   ├── sensors/
│   │   ├── sensor_interface.ino      # Arduino ultrasonic
│   │   └── arduino_sensor_interface.py
│   ├── obstacle_avoidance/
│   │   └── bluetooth_optimized_navigator_v6.py  # Avoidance logic
│   └── config/
│       └── config.py                 # Navigation parameters
│
├── firmware/                   ← Microcontroller code
│   ├── heltec_killswitch/
│   │   ├── Receiver/
│   │   └── Transmitter/
│   └── stm32_motor_controller/ ← To create
│
├── src/                        ← New Jetson code (to create)
│   ├── main.py                 # Entry point
│   ├── api/                    # Flask endpoints
│   ├── navigation/             # State machine, waypoints
│   ├── drivers/                # ZED-F9R, CAN, sensors
│   └── utils/                  # Logging, config
│
├── simulation/                 ← Gazebo files (to create)
│   ├── models/
│   ├── worlds/
│   └── launch/
│
└── tests/                      ← Unit and integration tests
```

---

## Quick Start for New Session

### 1. Understand the Context
```bash
# Read this file first
cat SESSION_HANDOFF.md

# Review the reference code structure
tree reference/

# Look at the navigation state machine
cat reference/motor_control/bluetooth_optimized_navigator_v6.py | head -200
```

### 2. Development Priorities
1. **If hardware hasn't arrived:** Start with Gazebo simulation
2. **If ZED-F9R arrived:** Write pyubx2 driver first
3. **If STM32 arrived:** Write CAN motor command firmware
4. **If all hardware ready:** Integration testing

### 3. Key Decisions Already Made
- ✅ Use Flask (not ROS2) for API - simpler, proven
- ✅ Use CAN bus (not serial) for motor commands - reliable, industrial
- ✅ Use ZED-F9R (not separate GPS+IMU) - integrated sensor fusion
- ✅ Keep MDDS30 in Mixed R/C mode - proven working
- ✅ Keep Heltec killswitch - safety critical, tested

---

## GitHub Repository

**URL:** https://github.com/JayFielding13/Jetson-Outdoor-Rover
**Visibility:** Private
**Branch:** main

---

## Contact & Notes

- All previous rover projects are in `/home/jay/Git Sandbox/`
- Original Cube Orange project: `Jetson Cube Orange Outdoor Rover/`
- Indoor rover that worked: `Jetson Simple Rover/`
- Mobile app: `Mobile RTK Control Module/`
