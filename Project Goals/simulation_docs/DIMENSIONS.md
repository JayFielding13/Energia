# Jetson Rover Dimensions & Specifications

## Physical Dimensions

### Chassis
- **Length**: 27 inches (0.6858 m)
- **Width**: 23.75 inches (0.60325 m)
- **Height**: 12 inches (0.3048 m)
- **Mass**: 90.7 kg (200 lbs total rover weight)
- **Color**: Orange

### Electronics Box (Prototype)
- **Length**: 12 inches (0.3048 m)
- **Width**: 12 inches (0.3048 m)
- **Height**: 8 inches (0.2032 m)
- **Mass**: 5.0 kg (~11 lbs, estimated)
- **Position**: Centered on top of chassis
- **Contents**: Jetson computer, Cube Orange, MDDS30 motor drivers, power distribution
- **Color**: Dark gray

### Wheels
- **Diameter**: 10 inches (0.254 m)
- **Radius**: 5 inches (0.127 m)
- **Width**: 3 inches (0.0762 m)
- **Mass**: 4.5 kg each (~10 lbs with wheelchair motor)
- **Type**: Continuous rotation (skid-steer)
- **Material**: Black rubber
- **Friction coefficient**: 1.0 (high grip)

## Wheel Positioning

### Configuration
- **Drive Type**: 4-wheel differential drive (skid-steer)
- **Wheelbase**: 17 inches (0.4318 m) - distance from front to rear axle
- **Track Width**: 23.75 inches (0.60325 m) - distance between left and right wheels
- **Ground Clearance**: 3 inches (0.0762 m)

### Specific Positions (from chassis center)
```
                    FRONT
         ┌──────────────────────┐
         │         ▲            │
         │    10" forward       │
    ●────┤         │            ├────●  Front Wheels
         │    ─────┼─────       │      (x = -0.2129 m)
         │         │ CENTER     │
         │    ─────┼─────       │
    ●────┤         │            ├────●  Rear Wheels
         │    7" rearward       │      (x = -0.3429 m)
         │         ▼            │      FLUSH with back edge
         └──────────────────────┘
                   REAR
         <------ 27 inches ----->

Side View:
         Front          Rear
           ●              ●    <- Wheel centers
         ├────17"────┤
         ├──────27"─────────┤  <- Chassis length
```

### Coordinate Frame
- **Origin**: Center of chassis (base_link)
- **X-axis**: Forward (positive = front, negative = rear)
- **Y-axis**: Left (positive = left, negative = right)
- **Z-axis**: Up (positive = up, negative = down)

### Calculated Wheel Positions
| Wheel | X (m) | Y (m) | Z (m) | Notes |
|-------|-------|-------|-------|-------|
| Front Left | -0.2129 | +0.3016 | -0.0524 | 10" from center |
| Front Right | -0.2129 | -0.3016 | -0.0524 | 10" from center |
| Rear Left | -0.3429 | +0.3016 | -0.0524 | **Flush with back edge** |
| Rear Right | -0.3429 | -0.3016 | -0.0524 | **Flush with back edge** |

**Calculations:**
- Rear wheel X: -chassis_length/2 = -0.6858/2 = **-0.3429 m** (at back edge)
- Front wheel X: rear_x + wheelbase = -0.3429 + 0.4318 = **-0.2129 m**
- Wheel Y: ±track_width/2 = ±0.60325/2 = **±0.3016 m**
- Wheel Z: -chassis_height/2 + wheel_radius - ground_clearance
          = -0.1524 + 0.127 - 0.0762 = **-0.0524 m**

## Electronics Mounting

### Electronics Box
- **Position**: Centered on top of chassis
- **Offset from base_link**: (0, 0, 0.254) m (chassis_height/2 + ebox_height/2)
- **Total Height Above Ground**: ~20 inches (chassis + electronics box)
- **Contains**:
  - Jetson Orin/Nano compute module
  - Cube Orange flight controller
  - 2× MDDS30 motor drivers
  - Power distribution board
  - Additional electronics

### Sensors on Top of Electronics Box

**RP-LIDAR A1:**
- **Position**: Top center of electronics box, aligned with front edge
- **Height from ground**: ~20 inches
- **Orientation**: 0° points forward (X+ axis)
- **Offset from ebox center**: Front edge, centered in Y
- **Topic**: `/scan` (sensor_msgs/LaserScan)

**Logitech C920X Webcam:**
- **Position**: Top of electronics box, 4 inches LEFT of LiDAR
- **Height from ground**: ~20 inches
- **Orientation**: Facing forward
- **Offset from ebox center**: Front edge, +0.1016m in Y (left)
- **Topics**: `/camera/image_raw`, `/camera/camera_info`

### Sensors Inside Electronics Box

**HERE 3+ GPS Antenna:**
- **Position**: Inside box, near top, to the RIGHT of LiDAR
- **Offset from ebox center**: Near front, -0.08m in Y (right)
- **Purpose**: RTK GPS positioning
- **Topic**: `/gps/fix` (sensor_msgs/NavSatFix)

**Cube Orange Flight Controller:**
- **Position**: Inside electronics box, 6 inches BELOW LiDAR
- **Offset from ebox center**: Centered in X/Y, 0.1524m below top
- **Orientation**: IMU arrow facing FORWARD (X+ axis)
- **Size**: ~80mm x 80mm x 40mm
- **Color**: Blue (in simulation)

**Jetson Compute Module:**
- **Position**: Inside electronics box, lower section
- **Offset from ebox center**: Slightly rear, near bottom
- **Size**: ~100mm x 80mm x 30mm
- **Color**: Blue (in simulation)

## Center of Mass
- **Location**: (0, 0, -0.0508) m - 4 inches above bottom edge of chassis
- **Height from ground**: 7 inches (3" ground clearance + 4")
- **Total rover weight**: 200 lbs (90.7 kg)
- **Distribution**:
  - Chassis + batteries + motors: ~195 lbs
  - Electronics box: ~11 lbs (estimated)
- **Note**: CoM positioned low for stability with heavy batteries in chassis base

## Drive Characteristics

### Differential Drive
- Uses left/right wheel speed difference for turning
- All 4 wheels driven (front + rear on each side)
- Skid-steering: wheels slip sideways during turns

### ROS 2 Control Topics
- **Command**: `/cmd_vel` (geometry_msgs/Twist)
  - `linear.x`: Forward/backward speed (m/s)
  - `angular.z`: Turning rate (rad/s)
- **Feedback**: `/odom` (nav_msgs/Odometry)
  - Position (x, y, theta)
  - Velocity (linear, angular)

### Typical Speed Ranges
- **Max linear velocity**: ~1.5 m/s (~3.4 mph, wheelchair motor estimate)
- **Max angular velocity**: ~1.0 rad/s (adjust based on turning characteristics)
- **Wheel torque**: 50 Nm max (simulated, for 200 lb rover)
- **Wheel acceleration**: 2.0 m/s² (simulated)

## Overall Rover Dimensions

```
Side View (with electronics box):
                    ┌─────────────┐
                    │ Electronics │  8" tall
                    │     Box     │
    ┌───────────────┴─────────────┴───────────────┐
    │                                              │
    │          Main Chassis (12" tall)            │
    │                                              │
    └──────────────────────────────────────────────┘
  ●                                                  ●
Wheel (10" dia)                                Wheel (10" dia)
  └────── 3" ground clearance ──────────────────────┘
─────────────────────────────────────────────────────── Ground

Total Height: ~23 inches (3" clearance + 12" chassis + 8" box)
Total Length: 27 inches
Total Width: 23.75 inches
```

## Notes for Simulation Accuracy

1. **Mass Distribution**: Model includes realistic weight distribution:
   - Total weight: 200 lbs (90.7 kg)
   - CoM: 4 inches above chassis bottom (7 inches from ground)
   - Electronics box: ~11 lbs on top
   - Most weight (batteries, motors) in chassis base for stability

2. **Wheel Friction**: Current friction coefficients (mu1=1.0, mu2=1.0) are estimates
   - Test on actual terrain
   - Adjust based on wheel slip observations

3. **Ground Clearance**: 3 inches assumes level ground
   - Consider maximum obstacle height rover can traverse
   - Electronics box adds 8 inches of clearance needed for overhead obstacles

4. **Wheelbase Impact**: 17" wheelbase affects turning radius
   - Shorter = tighter turns, less stability
   - Longer = wider turns, more stability
   - Current setup balanced for outdoor navigation

## Completed Sensor Integration

- [x] Ultrasonic sensors (6x AJ-SR04M at 11" height) - **COMPLETED**
- [x] Electronics box (12x12x8 inches on top) - **COMPLETED**
- [x] RP-LIDAR A1 (top of electronics box) - **COMPLETED**
- [x] Logitech C920X camera (top of ebox, left of LiDAR) - **COMPLETED**
- [x] HERE 3+ GPS antenna (inside ebox, right of LiDAR) - **COMPLETED**
- [x] Cube Orange with IMU (inside ebox, 6" below LiDAR) - **COMPLETED**

## Future Additions

Planned components to add:
- [ ] Battery visualization (inside chassis base)
- [ ] Telemetry antenna mount (on electronics box)
- [ ] Power distribution board visualization
- [ ] Motor driver heat sinks
- [ ] Emergency stop button
- [ ] Status LED indicators
