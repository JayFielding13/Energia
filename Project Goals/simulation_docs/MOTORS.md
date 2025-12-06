# Jetson Rover Motor & Drive System

## Overview

The Jetson Cube Orange rover uses a **4-wheel drive (4WD) skid-steer** configuration powered by wheelchair motors.

## Motor Configuration

### Motors

**Type:** Off-the-shelf wheelchair motors
**Quantity:** 4 motors (one per wheel)
**Drive:** Direct drive (each motor directly drives one wheel)
**Configuration:** 4WD skid-steer

#### Typical Wheelchair Motor Specifications (Estimates)

| Parameter | Estimated Value | Notes |
|-----------|----------------|-------|
| **Voltage** | 24V | Common for wheelchair motors |
| **Power** | 200-250W each | Total: 800-1000W |
| **RPM** | 200-250 RPM | At rated voltage |
| **Torque** | ~10-15 Nm each | High torque for mobility |
| **Current Draw** | 8-10A peak | Per motor |
| **Gearbox** | Integrated | Planetary or spur gears |

### Motor Controllers

**Model:** MDDS30 Dual Channel Motor Drivers
**Quantity:** 2 drivers
**Manufacturer:** Cytron Technologies

#### MDDS30 Specifications

| Parameter | Value |
|-----------|-------|
| **Channels per Driver** | 2 (dual channel) |
| **Voltage Range** | 5V - 30V DC |
| **Continuous Current** | 15A per channel |
| **Peak Current** | 30A per channel (10 seconds) |
| **PWM Frequency** | 20 kHz |
| **Control Interface** | PWM, Analog, RC, Serial |
| **Protection** | Over-current, over-temperature, under-voltage |

#### Motor Driver Wiring Configuration

```
MDDS30 Driver #1 (Front Wheels)
├── Channel A → Front Left Motor
└── Channel B → Front Right Motor

MDDS30 Driver #2 (Rear Wheels)
├── Channel A → Rear Left Motor
└── Channel B → Rear Right Motor
```

### Drive Train Architecture

```
┌─────────────────────────────────────────────────┐
│              Control System                      │
│  ┌──────────────────────────────────────┐       │
│  │    Cube Orange (Flight Controller)   │       │
│  │    - Receives navigation commands    │       │
│  │    - Outputs motor control signals   │       │
│  └──────────┬────────────┬──────────────┘       │
│             │            │                       │
│       PWM   │            │   PWM                 │
│             ▼            ▼                       │
│    ┌────────────┐  ┌────────────┐              │
│    │  MDDS30 #1 │  │  MDDS30 #2 │              │
│    │   (Front)  │  │   (Rear)   │              │
│    └─┬────────┬─┘  └─┬────────┬─┘              │
│      │        │      │        │                 │
│      ▼        ▼      ▼        ▼                 │
│   ┌───┐    ┌───┐  ┌───┐    ┌───┐              │
│   │ FL│    │ FR│  │ RL│    │ RR│              │
│   │Mtr│    │Mtr│  │Mtr│    │Mtr│              │
│   └─┬─┘    └─┬─┘  └─┬─┘    └─┬─┘              │
│     │        │      │        │                 │
│     ▼        ▼      ▼        ▼                 │
│   Wheel    Wheel  Wheel    Wheel               │
│   (10")    (10")  (10")    (10")               │
└─────────────────────────────────────────────────┘

FL = Front Left, FR = Front Right
RL = Rear Left, RR = Rear Right
```

## Physical Specifications

### Overall System

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Total Weight** | 200 lbs (90.7 kg) | Including all components |
| **Center of Mass** | 4 inches above bottom edge | Centered in X and Y |
| **Wheel Weight** | ~10 lbs each (4.5 kg) | Motor + wheel assembly |
| **Chassis Weight** | ~160 lbs (72.6 kg) | Remaining weight after wheels |

### Weight Distribution

```
Top View:
        FRONT
    ●─────┬─────●
    │     │     │   ● = Motor + Wheel (~10 lbs each)
    │   [CoM]   │   Total wheel weight: 40 lbs
    │     │     │   Chassis + components: 160 lbs
    ●─────┴─────●
        REAR

Side View:
        ┌─────────┐ ← Top of chassis (12" high)
        │         │
        │  [CoM]  │ ← Center of mass (4" from bottom)
    ●───┴────────┴───● ← Wheel level
        3" clearance
    ─────────────────── Ground
```

## Performance Characteristics

### Speed Calculations

With 10-inch diameter wheels and typical wheelchair motor specs:

| Motor RPM | Wheel RPS | Linear Speed | Speed (mph) |
|-----------|-----------|--------------|-------------|
| 100 RPM | 1.67 RPS | 0.67 m/s | 1.5 mph |
| 150 RPM | 2.50 RPS | 1.00 m/s | 2.2 mph |
| 200 RPM | 3.33 RPS | 1.33 m/s | 3.0 mph |
| 250 RPM | 4.17 RPS | 1.67 m/s | 3.7 mph |

**Formula:**
- Wheel circumference = π × diameter = 3.14159 × 0.254m = 0.798m
- Linear speed (m/s) = RPM × circumference / 60

### Estimated Performance

| Metric | Value | Conditions |
|--------|-------|------------|
| **Top Speed** | ~2-3 mph (1.0-1.5 m/s) | Flat terrain, conservative |
| **Cruising Speed** | ~1-2 mph (0.5-1.0 m/s) | Normal outdoor operation |
| **Acceleration** | Moderate | Limited by motor controllers |
| **Turn in Place** | Yes | Skid-steer allows zero-radius turns |
| **Climbing Ability** | Good | High torque motors, 4WD |
| **Max Slope** | 15-20° estimated | Depends on traction |

### Torque Requirements

For a 200 lb (90.7 kg) rover:

- **Weight per wheel:** 50 lbs (22.7 kg) static load
- **Required torque (level ground):** ~5-10 Nm per wheel
- **Required torque (incline):** ~15-20 Nm per wheel (15° slope)
- **Available torque:** ~10-15 Nm per motor (estimated)
- **Safety factor:** 1.5-2x (adequate for terrain)

## Skid-Steer Kinematics

### How Skid-Steer Works

```
Forward Motion:                Turning Left:
All wheels same speed          Right faster than left

  ↑  ↑                          ↑↑  ↑↑
  │  │                          ││  ││
  │  │                          ││  ││
  ↑  ↑                          ↑   ↑

Turning Right:                 Spin in Place (Left):
Left faster than right         Left backward, right forward

  ↑  ↑                          ↓↓  ↑↑
  │  │                          ││  ││
  │  │                          ││  ││
  ↑  ↑                          ↓   ↑
```

### Control Equations

For differential/skid-steer drive:

**Linear velocity (v) and angular velocity (ω):**
```
v_left  = v - (ω × track_width / 2)
v_right = v + (ω × track_width / 2)

Where:
  v = linear velocity (m/s)
  ω = angular velocity (rad/s)
  track_width = 0.60325 m (23.75 inches)
```

**Wheel velocities:**
```
ω_left_wheels  = v_left  / wheel_radius
ω_right_wheels = v_right / wheel_radius

Where:
  wheel_radius = 0.127 m (5 inches)
```

### Turning Radius

**Minimum turning radius:** 0 (can spin in place)

**Typical turning radius at speed:**
```
R = v / ω

Example:
  v = 1.0 m/s
  ω = 0.5 rad/s
  R = 1.0 / 0.5 = 2.0 meters
```

## ROS 2 Integration

### Command Interface

The rover listens to `/cmd_vel` (geometry_msgs/Twist) for control:

```python
from geometry_msgs.msg import Twist

cmd = Twist()

# Forward at 0.5 m/s
cmd.linear.x = 0.5
cmd.angular.z = 0.0

# Turn left while moving forward
cmd.linear.x = 0.5
cmd.angular.z = 0.3  # rad/s

# Spin in place (left)
cmd.linear.x = 0.0
cmd.angular.z = 0.5
```

### Odometry Output

The rover publishes to `/odom` (nav_msgs/Odometry):

```yaml
header:
  frame_id: odom
child_frame_id: base_link

pose:
  position: {x, y, z}
  orientation: {quaternion}

twist:
  linear: {x, y, z}  # Velocity
  angular: {x, y, z} # Angular velocity
```

### Safety Limits (Simulation)

| Parameter | Simulation Value | Reasoning |
|-----------|-----------------|-----------|
| Max Linear Velocity | 1.5 m/s (~3.4 mph) | Wheelchair motor limit |
| Max Angular Velocity | 1.0 rad/s (~57°/s) | Safe turning rate |
| Max Wheel Torque | 50 Nm | Adequate for 200 lb rover + terrain |
| Max Acceleration | 2.0 m/s² | Moderate for safety |

## Power Consumption Estimates

### Current Draw

| Condition | Current per Motor | Total Current | Power |
|-----------|------------------|---------------|-------|
| **Idle** | 0.5A | 2A | ~50W |
| **Cruising (flat)** | 3-5A | 12-20A | ~300-500W |
| **Climbing** | 8-10A | 32-40A | ~800-1000W |
| **Peak (startup)** | 15A | 60A | ~1500W |

### Battery Configuration

**Current Setup:**
- **Type**: 2× 12V Lead Acid (lawn equipment style)
- **Configuration**: Series (24V total)
- **Estimated Capacity**: ~20-35 Ah (typical for lawn equipment batteries)
- **Weight**: ~15-20 lbs each, ~30-40 lbs total
- **Location**: Inside chassis base (contributes to low center of mass)

### Runtime Estimates

For 24V system with typical capacity:

| Battery Capacity | Runtime (Cruising) | Runtime (Mixed) | Range (@ 2 mph) |
|-----------------|-------------------|----------------|-----------------|
| 20 Ah | ~1.5 hours | ~1 hour | ~3 miles |
| 35 Ah | ~2.5 hours | ~1.75 hours | ~5 miles |
| 50 Ah | ~3.5 hours | ~2.5 hours | ~7 miles |

**Note:** Actual runtime depends on:
- Terrain difficulty (grass vs. pavement)
- Speed
- Payload
- Temperature
- Battery age/condition
- Lead acid batteries lose capacity in cold weather

## Simulation Parameters

### Gazebo Configuration

From `jetson_rover_gazebo.xacro`:

```xml
<plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">
  <!-- 4WD: All wheels driven -->
  <left_joint>front_left_wheel_joint</left_joint>
  <left_joint>rear_left_wheel_joint</left_joint>
  <right_joint>front_right_wheel_joint</right_joint>
  <right_joint>rear_right_wheel_joint</right_joint>

  <!-- Kinematics -->
  <wheel_separation>0.60325</wheel_separation>
  <wheel_diameter>0.254</wheel_diameter>

  <!-- Motor limits -->
  <max_wheel_torque>50</max_wheel_torque>
  <max_wheel_acceleration>2.0</max_wheel_acceleration>

  <!-- Control -->
  <command_topic>cmd_vel</command_topic>
  <odometry_topic>odom</odometry_topic>
</plugin>
```

### Tuning for Real Hardware

When you get exact motor specs, update these parameters in `jetson_rover.urdf.xacro`:

```xml
<!-- Update these with actual values -->
<xacro:property name="chassis_mass" value="90.7"/>  <!-- Weigh actual rover -->
<xacro:property name="wheel_mass" value="4.5"/>     <!-- Weigh motor + wheel -->
```

And in `jetson_rover_gazebo.xacro`:

```xml
<!-- Match to real motor performance -->
<max_wheel_torque>XX</max_wheel_torque>  <!-- From motor specs -->
<max_wheel_acceleration>X.X</max_wheel_acceleration>  <!-- Test and measure -->
```

## Troubleshooting

### Rover Moves Slowly in Simulation

- Check `max_wheel_torque` - increase if too low
- Verify wheel friction (`mu1`, `mu2`) is adequate
- Check if mass is correct (too heavy = slow)

### Rover Slips During Turns

- Increase wheel friction coefficients
- Reduce `max_wheel_acceleration`
- Check surface friction in Gazebo world

### Unrealistic Movement

- Verify `wheel_separation` matches actual track width
- Check `wheel_diameter` matches actual wheels
- Ensure all 4 wheels are assigned to correct joints

### Motor Overheating (Real Robot)

- Reduce speed limits
- Add cooling fans
- Check for mechanical binding
- Verify motor controllers are properly rated

## Future Enhancements

### Planned Additions

- [ ] Current monitoring (via motor controllers)
- [ ] Battery voltage monitoring
- [ ] Motor temperature sensors
- [ ] Encoder feedback (if motors have encoders)
- [ ] Independent front/rear control (if needed)
- [ ] Torque vectoring for better turns

### Advanced Features

- **Terrain adaptation:** Adjust motor power based on slope
- **Traction control:** Detect wheel slip and compensate
- **Energy optimization:** Variable speed based on terrain
- **Regenerative braking:** If controllers support it

## Motor Driver Pinout (MDDS30)

### Connections (Typical)

```
MDDS30 Driver #1 (Front Wheels):
  PWM1  → Cube Orange PWM output (left front)
  PWM2  → Cube Orange PWM output (right front)
  GND   → Common ground
  VCC   → 5V logic supply
  V+    → Battery + (24V)
  M1+/- → Front left motor
  M2+/- → Front right motor

MDDS30 Driver #2 (Rear Wheels):
  PWM1  → Cube Orange PWM output (left rear)
  PWM2  → Cube Orange PWM output (right rear)
  GND   → Common ground
  VCC   → 5V logic supply
  V+    → Battery + (24V)
  M1+/- → Rear left motor
  M2+/- → Rear right motor
```

## References

- **MDDS30 Datasheet:** https://www.cytron.io/p-30amp-7v-30v-dc-motor-driver
- **ROS 2 Differential Drive Plugin:** http://wiki.ros.org/diff_drive_controller
- **Gazebo Motor Plugin:** http://gazebosim.org/tutorials?tut=ros_gzplugins

---

**Created:** November 8, 2025
**Status:** Simulation configured with estimated parameters
**Next Step:** Measure actual motor specs and update simulation for accuracy
