# Energia Rover Motor & Drive System

## Overview

The Energia rover uses a **4-wheel drive (4WD) skid-steer** configuration powered by wheelchair motors, with direct Jetson-to-hardware control via CAN bus.

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
┌─────────────────────────────────────────────────────────────┐
│                    Control System                            │
│                                                              │
│  ┌────────────────────┐     ┌────────────────────┐          │
│  │  Jetson Orin Nano  │     │   STM32 Nucleo     │          │
│  │  (Main Computer)   │     │   (Motor Control)   │          │
│  │  - ROS2 nodes      │     │  - CAN receiver     │          │
│  │  - /cmd_vel input  │     │  - PWM generation   │          │
│  └─────────┬──────────┘     └──────────┬─────────┘          │
│            │                           │                     │
│      USB-to-CAN               CAN Bus  │                     │
│            │                           │                     │
│            └─────────────┬─────────────┘                     │
│                          │                                   │
│              ┌───────────┴───────────┐                       │
│              │      CAN Messages     │                       │
│              │   (Throttle/Steering) │                       │
│              └───────────┬───────────┘                       │
│                          │                                   │
│         ┌────────────────┴────────────────┐                 │
│         │                                 │                  │
│         ▼                                 ▼                  │
│  ┌────────────┐                    ┌────────────┐           │
│  │  MDDS30 #1 │                    │  MDDS30 #2 │           │
│  │   (Front)  │                    │   (Rear)   │           │
│  │ Mixed R/C  │                    │ Mixed R/C  │           │
│  └─┬────────┬─┘                    └─┬────────┬─┘           │
│    │        │                        │        │              │
│    ▼        ▼                        ▼        ▼              │
│ ┌───┐    ┌───┐                    ┌───┐    ┌───┐            │
│ │ FL│    │ FR│                    │ RL│    │ RR│            │
│ │Mtr│    │Mtr│                    │Mtr│    │Mtr│            │
│ └─┬─┘    └─┬─┘                    └─┬─┘    └─┬─┘            │
│   │        │                        │        │               │
│   ▼        ▼                        ▼        ▼               │
│ Wheel    Wheel                    Wheel    Wheel             │
│ (10")    (10")                    (10")    (10")             │
└─────────────────────────────────────────────────────────────┘

FL = Front Left, FR = Front Right
RL = Rear Left, RR = Rear Right
```

### Motor Control Signal Flow

1. **Jetson Orin Nano** receives `/cmd_vel` commands from ROS2 nodes
2. **USB-to-CAN adapter** converts commands to CAN bus messages
3. **STM32 Nucleo** receives CAN messages and generates PWM signals
4. **MDDS30 drivers** in Mixed R/C mode receive:
   - Channel 1 (D6): Throttle (forward/backward)
   - Channel 2 (D5): Steering (differential turning)
5. PWM values: 1500 = stopped, 1700 = full forward, 1300 = full backward

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

From `energia_rover_gazebo.xacro`:

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

When you get exact motor specs, update these parameters in `energia_rover.urdf.xacro`:

```xml
<!-- Update these with actual values -->
<xacro:property name="chassis_mass" value="90.7"/>  <!-- Weigh actual rover -->
<xacro:property name="wheel_mass" value="4.5"/>     <!-- Weigh motor + wheel -->
```

And in `energia_rover_gazebo.xacro`:

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

## NUCLEO-F446RE Pin Definitions

### Board Overview

The **NUCLEO-F446RE** is the motor controller MCU that receives commands via CAN bus and generates PWM signals for the MDDS30 motor drivers.

| Parameter | Value |
|-----------|-------|
| **MCU** | STM32F446RET6 |
| **Core** | ARM Cortex-M4 @ 180 MHz |
| **Flash** | 512 KB |
| **RAM** | 128 KB |
| **CAN** | CAN1 (requires external transceiver) |
| **Timers** | TIM1-TIM14 (multiple PWM capable) |

### Pin Assignments

#### PWM Outputs (to MDDS30 Drivers)

| Function | STM32 Pin | Connector | Arduino Pin | Timer | Notes |
|----------|-----------|-----------|-------------|-------|-------|
| **PWM Throttle** | PA8 | CN9-8 | D7 | TIM1_CH1 | Forward/backward control |
| **PWM Steering** | PA9 | CN5-1 | D8 | TIM1_CH2 | Left/right differential |

#### CAN Bus (Seeed Studio MCP2515/SN65HVD230 Module via SPI)

The MCP2515 CAN controller communicates via **SPI**, not the STM32's native CAN pins.

| Function | STM32 Pin | Connector | Arduino Pin | MCP2515 Pin | Notes |
|----------|-----------|-----------|-------------|-------------|-------|
| **SPI1_SCK** | PA5 | CN5-6 | D13 | SCK | SPI Clock |
| **SPI1_MISO** | PA6 | CN5-5 | D12 | SO (MISO) | SPI Data Out |
| **SPI1_MOSI** | PA7 | CN5-4 | D11 | SI (MOSI) | SPI Data In |
| **SPI1_CS** | PB6 | CN5-3 | D10 | CS | Chip Select |
| **INT** | PB5 | CN9-6 | D4 | INT | Interrupt (active low) |

**Connector Notes:**
- **CN5** - 6-pin Arduino SPI header (directly above CN9)
- **CN9** - 8-pin Arduino digital header (D0-D7)
- **CN7/CN10** - 38-pin Morpho connectors (outer rows)

**Note:** PA5 is also the onboard LED (LD2). If you need the LED for status, use SPI2 instead (PB13=SCK, PB14=MISO, PB15=MOSI on CN10).

#### Status LEDs and Debug

| Function | STM32 Pin | Connector | Notes |
|----------|-----------|-----------|-------|
| **User LED** | PA5 | LD2 (onboard) | Green LED, shared with SPI1_SCK |
| **User Button** | PC13 | B1 (onboard) | Blue button, active low |

#### Optional: Kill Switch Input

| Function | STM32 Pin | Connector | Arduino Pin | Notes |
|----------|-----------|-----------|-------------|-------|
| **Kill Switch** | PC0 | CN7-38 | A5 | External kill relay signal (active low) |

### Seeed Studio MCP2515/SN65HVD230 Module Wiring

The Seeed Studio CAN module contains:
- **MCP2515** - CAN controller (SPI interface)
- **SN65HVD230** - CAN transceiver (3.3V compatible)

```
NUCLEO-F446RE                    Seeed MCP2515/SN65HVD230 Module
┌─────────────┐                  ┌─────────────────────────────┐
│             │                  │                             │
│  PA5 (D13) ─┼──────────────────┼── SCK     ┌───────────────┐│
│             │                  │           │   MCP2515     ││
│  PA6 (D12) ─┼──────────────────┼── SO      │  (CAN Ctrl)   ││
│             │                  │           └───────┬───────┘│
│  PA7 (D11) ─┼──────────────────┼── SI              │        │
│             │                  │                   │        │
│  PB6 (D10) ─┼──────────────────┼── CS      ┌───────┴───────┐│
│             │                  │           │  SN65HVD230   ││
│  PB5 (D4) ──┼──────────────────┼── INT     │ (Transceiver) ││
│             │                  │           └───────┬───────┘│
│  3.3V ──────┼──────────────────┼── VCC             │        │
│             │                  │                   │        │
│  GND ───────┼──────────────────┼── GND     CAN_H ──┼────────┼──► To CAN Bus
│             │                  │           CAN_L ──┼────────┼──► To CAN Bus
└─────────────┘                  │                             │
                                 └─────────────────────────────┘
```

**Module Pinout (typical 8-pin header):**

| Module Pin | Connect To | Description |
|------------|------------|-------------|
| VCC | 3.3V (CN6-4 or CN7-16) | Power supply - **use 3.3V** for NUCLEO compatibility |
| GND | GND (CN6-6 or CN5-7) | Ground |
| CS | PB6 (D10) | SPI Chip Select |
| SO | PA6 (D12) | SPI MISO (Master In, Slave Out) |
| SI | PA7 (D11) | SPI MOSI (Master Out, Slave In) |
| SCK | PA5 (D13) | SPI Clock |
| INT | PB5 (D4) | Interrupt output (active low) |
| CAN_H | CAN Bus | CAN High line |
| CAN_L | CAN Bus | CAN Low line |

**Point-to-Point Wiring (NUCLEO → Seeed Module):**

| NUCLEO Pin | Connector-Pin | Wire | Seeed Module Pin | Function |
|------------|---------------|------|------------------|----------|
| D13 | CN5-6 | White | SCK | SPI Clock |
| D12 | CN5-5 | Gray | SO (MISO) | Data from module |
| D11 | CN5-4 | Purple | SI (MOSI) | Data to module |
| D10 | CN5-3 | Blue | CS | Chip Select |
| D4 | CN9-6 | Green | INT | Interrupt |
| 3V3 | CN6-4 | Red | VCC | 3.3V Power |
| GND | CN6-6 | Black | GND | Ground |

**Important:**
- Add 120Ω termination resistor between CAN_H and CAN_L at each end of the bus
- **Use 3.3V power** - The SN65HVD230 transceiver is designed for 3.3V operation
- The MCP2515 also operates at 3.3V, matching STM32 logic levels (no level shifters needed)
- Power from NUCLEO: CN6-4 (3V3 near USB) or CN7-16 (Morpho connector) - both provide ~300mA

### MDDS30 Wiring (Mixed R/C Mode)

The MDDS30 operates in **Mixed R/C mode** where:
- **Channel 1 (IN1):** Throttle - controls forward/backward for both motors
- **Channel 2 (IN2):** Steering - controls differential (left/right speed difference)

```
NUCLEO-F446RE                         MDDS30 Driver #1 (Front)
┌─────────────┐                       ┌─────────────────────┐
│             │                       │                     │
│  PA8 (D7) ──┼───────┬───────────────┼── IN1 (Throttle)   │
│             │       │               │                     │
│  PA9 (D8) ──┼───┬───┼───────────────┼── IN2 (Steering)   │
│             │   │   │               │                     │
│  GND ───────┼───┼───┼───────────────┼── GND              │
│             │   │   │               │                     │
└─────────────┘   │   │               │  M1+/- ────────────┼──► Front Left Motor
                  │   │               │  M2+/- ────────────┼──► Front Right Motor
                  │   │               │  V+ ───────────────┼──► Battery + (24V)
                  │   │               │  GND ──────────────┼──► Battery -
                  │   │               └─────────────────────┘
                  │   │
                  │   │               MDDS30 Driver #2 (Rear)
                  │   │               ┌─────────────────────┐
                  │   │               │                     │
                  │   └───────────────┼── IN1 (Throttle)   │  [Parallel]
                  │                   │                     │
                  └───────────────────┼── IN2 (Steering)   │  [Parallel]
                                      │                     │
              GND ────────────────────┼── GND              │
                                      │                     │
                                      │  M1+/- ────────────┼──► Rear Left Motor
                                      │  M2+/- ────────────┼──► Rear Right Motor
                                      │  V+ ───────────────┼──► Battery + (24V)
                                      │  GND ──────────────┼──► Battery -
                                      └─────────────────────┘
```

### PWM Signal Specifications

The MDDS30 in Mixed R/C mode expects standard **RC servo PWM signals**:

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Frequency** | 50 Hz | 20ms period (standard RC) |
| **Pulse Width Range** | 1000-2000 μs | Full range |
| **Neutral/Stop** | 1500 μs | Motors stopped |
| **Full Forward** | 1700-2000 μs | Max forward speed |
| **Full Reverse** | 1000-1300 μs | Max reverse speed |
| **Dead Band** | ±50 μs around 1500 μs | Configurable on MDDS30 |

### STM32 Timer Configuration

For generating 50Hz RC PWM on TIM1:

```c
// Timer settings for 50Hz PWM (20ms period)
// Assuming 180MHz system clock with APB2 timer clock = 180MHz

#define PWM_FREQUENCY     50      // Hz
#define PWM_PERIOD_US     20000   // microseconds
#define TIMER_PRESCALER   180     // 180MHz / 180 = 1MHz timer clock
#define TIMER_PERIOD      20000   // 1MHz / 20000 = 50Hz

// PWM pulse width values (in timer counts = microseconds)
#define PWM_NEUTRAL       1500    // Stop
#define PWM_MAX_FORWARD   1700    // Full forward (conservative)
#define PWM_MAX_REVERSE   1300    // Full reverse (conservative)
#define PWM_ABSOLUTE_MAX  2000    // Hardware limit
#define PWM_ABSOLUTE_MIN  1000    // Hardware limit
```

### Complete Wiring Summary

```
┌──────────────────────────────────────────────────────────────────────────────────┐
│                          COMPLETE WIRING DIAGRAM                                  │
├──────────────────────────────────────────────────────────────────────────────────┤
│                                                                                   │
│  ┌──────────────┐      USB        ┌──────────────┐                               │
│  │ Jetson Orin  │◄───────────────►│ USB-to-CAN   │                               │
│  │    Nano      │                 │   Adapter    │                               │
│  └──────────────┘                 └──────┬───────┘                               │
│                                          │ CAN_H, CAN_L                          │
│                                          │                                        │
│                              ┌───────────┴───────────┐                           │
│                              │  Seeed MCP2515 Module │                           │
│                              │  (SN65HVD230)         │                           │
│                              │                       │                           │
│                              │  CAN_H ◄──────────────┼─── CAN Bus                │
│                              │  CAN_L ◄──────────────┼─── CAN Bus                │
│                              │                       │                           │
│                              │  SCK ─────────────────┼─── PA5 (D13)              │
│                              │  SO ──────────────────┼─── PA6 (D12)              │
│                              │  SI ──────────────────┼─── PA7 (D11)              │
│                              │  CS ──────────────────┼─── PB6 (D10)              │
│                              │  INT ─────────────────┼─── PB5 (D4)               │
│                              │  VCC ─────────────────┼─── 3.3V                   │
│                              │  GND ─────────────────┼─── GND                    │
│                              └───────────────────────┘                           │
│                                          │                                        │
│                                   ┌──────┴───────┐                               │
│                                   │ NUCLEO-F446RE│                               │
│                                   │              │                               │
│                                   │  SPI1 (CAN)  │                               │
│                                   │  PA5 = SCK   │                               │
│                                   │  PA6 = MISO  │                               │
│                                   │  PA7 = MOSI  │                               │
│                                   │  PB6 = CS    │                               │
│                                   │  PB5 = INT   │                               │
│                                   │              │                               │
│                                   │  PA8 ─► PWM1 ┼──┬──► MDDS30 #1 IN1          │
│                                   │  PA9 ─► PWM2 ┼──┼──► MDDS30 #1 IN2          │
│                                   │              │  │                            │
│                                   │  GND ────────┼──┼──► MDDS30 #1 GND          │
│                                   │              │  │                            │
│                                   │  PC13 = BTN  │  ├──► MDDS30 #2 IN1          │
│                                   │              │  └──► MDDS30 #2 IN2          │
│                                   └──────────────┘                               │
│                                                                                   │
│  MDDS30 #1 (Front)              MDDS30 #2 (Rear)                                 │
│  ┌────────────────┐             ┌────────────────┐                               │
│  │ M1+ ──► FL Motor             │ M1+ ──► RL Motor                               │
│  │ M1- ──► FL Motor             │ M1- ──► RL Motor                               │
│  │ M2+ ──► FR Motor             │ M2+ ──► RR Motor                               │
│  │ M2- ──► FR Motor             │ M2- ──► RR Motor                               │
│  │ V+  ◄── Battery 24V          │ V+  ◄── Battery 24V                            │
│  │ GND ◄── Battery GND          │ GND ◄── Battery GND                            │
│  └────────────────┘             └────────────────┘                               │
│                                                                                   │
│  FL = Front Left, FR = Front Right, RL = Rear Left, RR = Rear Right              │
└──────────────────────────────────────────────────────────────────────────────────┘
```

### Pin Quick Reference Table

| Connection | Arduino Pin | Connector | STM32 Pin | Wire Color (suggested) |
|------------|-------------|-----------|-----------|------------------------|
| **PWM (to MDDS30)** |
| PWM Throttle → MDDS30 IN1 | D7 | CN9-8 | PA8 | Yellow |
| PWM Steering → MDDS30 IN2 | D8 | CN5-1 | PA9 | Orange |
| **SPI (to MCP2515 CAN Module)** |
| SPI SCK → MCP2515 SCK | D13 | CN5-6 | PA5 | White |
| SPI MISO → MCP2515 SO | D12 | CN5-5 | PA6 | Gray |
| SPI MOSI → MCP2515 SI | D11 | CN5-4 | PA7 | Purple |
| SPI CS → MCP2515 CS | D10 | CN5-3 | PB6 | Blue |
| INT → MCP2515 INT | D4 | CN9-6 | PB5 | Green |
| **Power (to MCP2515 CAN Module)** |
| 3.3V → MCP2515 VCC | - | CN6-4 (near USB) | 3V3 | Red |
| GND → All devices | - | CN6-6 or CN5-7 | GND | Black |
| **Optional** |
| Kill Switch Input | A5 | CN7-38 | PC0 | Brown |

### MDDS30 DIP Switch Settings (Mixed R/C Mode)

Set the MDDS30 DIP switches for **Mixed R/C mode**:

| Switch | Position | Function |
|--------|----------|----------|
| SW1 | OFF | - |
| SW2 | ON | Mixed mode |
| SW3 | OFF | R/C input |
| SW4 | OFF | - |

Refer to [MDDS30 datasheet](https://www.cytron.io/p-30amp-7v-30v-dc-motor-driver) for exact switch positions.

## References

- **MDDS30 Datasheet:** https://www.cytron.io/p-30amp-7v-30v-dc-motor-driver
- **ROS 2 Differential Drive Plugin:** http://wiki.ros.org/diff_drive_controller
- **Gazebo Motor Plugin:** http://gazebosim.org/tutorials?tut=ros_gzplugins

---

**Created:** November 8, 2025
**Updated:** December 2025 - Migrated to Jetson → CAN → STM32 architecture
**Status:** Simulation configured with estimated parameters
**Next Step:** Complete STM32 firmware and test CAN bus communication
