# LiDAR, Camera, and GPS Sensors

## Overview

The Jetson Cube Orange rover features three primary navigation sensors mounted on and inside the electronics box:

1. **RP-LIDAR A1** - 360° 2D laser scanner
2. **Logitech C920X HD Webcam** - Visual camera
3. **HERE 3+ RTK GPS** - High-precision GPS antenna

## RP-LIDAR A1

### Hardware Specifications

| Parameter | Value |
|-----------|-------|
| **Model** | SLAMTEC RP-LIDAR A1 |
| **Range** | 0.15m - 12m |
| **Sample Rate** | 8000 samples/sec |
| **Scan Rate** | 5.5 Hz (standard), up to 10 Hz |
| **Angular Resolution** | ≤1° |
| **Accuracy** | ±5mm (typical) |
| **FOV** | 360° |
| **Laser Wavelength** | 775-795nm (near-infrared) |
| **Laser Power** | <5mW |
| **Weight** | 170g |
| **Dimensions** | Ø76mm × 39mm (height) |

### Mounting Position

- **Location**: Top of electronics box, centered
- **Alignment**: Front edge of electronics box
- **Height from ground**: ~20 inches (chassis + ebox top)
- **Orientation**: 0° points forward (X+ axis)
- **Parent frame**: `electronics_box`

### Position in Simulation

```
Top View of Electronics Box:
        FRONT (X+)
    ┌──────────────┐
    │   [LiDAR]    │  ← Centered on top, at front edge
    │      ●       │
    │              │
    │   Ebox Top   │
    └──────────────┘
        REAR
```

**Offset from electronics_box center:**
- X: `ebox_length/2 - lidar_radius` (at front edge)
- Y: 0 (centered)
- Z: `ebox_height/2 + lidar_height/2` (on top)

### ROS 2 Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | 360° laser scan data |

### LaserScan Message Format

```yaml
header:
  frame_id: rplidar_link
angle_min: 0.0          # Start angle (radians)
angle_max: 6.28319      # End angle (2π radians)
angle_increment: 0.0174533  # ~1° in radians
time_increment: ~0.0001  # Time between measurements
scan_time: 0.1          # Time for complete scan (10 Hz)
range_min: 0.15         # Minimum range (meters)
range_max: 12.0         # Maximum range (meters)
ranges: [...]           # Array of 360 distance measurements
intensities: [...]      # Reflectivity values (if available)
```

### Usage Examples

**View scan data:**
```bash
ros2 topic echo /scan
```

**Visualize in RViz2:**
```bash
rviz2
# Add -> By topic -> /scan -> LaserScan
# Set Fixed Frame to "rplidar_link" or "base_link"
```

**Subscribe in Python:**
```python
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # Get distance at 0° (straight ahead)
    front_distance = msg.ranges[0]

    # Get distance at 90° (left)
    left_distance = msg.ranges[90]

    # Find minimum distance in front 60° arc
    front_arc = msg.ranges[-30:] + msg.ranges[:30]
    min_front = min(front_arc)

    print(f"Front: {front_distance:.2f}m, Min ahead: {min_front:.2f}m")

self.create_subscription(LaserScan, '/scan', scan_callback, 10)
```

## Logitech C920X HD Webcam

### Hardware Specifications

| Parameter | Value |
|-----------|-------|
| **Model** | Logitech C920X HD Pro |
| **Resolution** | 1920×1080 @ 30fps (Full HD) |
| **FOV** | 78° diagonal (~62° horizontal) |
| **Lens Type** | Glass lens with autofocus |
| **Focus Range** | 20cm to infinity |
| **Video Compression** | H.264 |
| **Weight** | 162g |
| **Dimensions** | 94mm × 29mm × 43mm (W×H×D) |

### Mounting Position

- **Location**: Top of electronics box
- **Position**: 4 inches (0.1016m) to the LEFT of RP-LIDAR center
- **Alignment**: Front edge of electronics box
- **Height from ground**: ~20 inches
- **Orientation**: Facing forward (X+ axis)
- **Parent frame**: `electronics_box`

### Position in Simulation

```
Top View of Electronics Box:
        FRONT (X+)
    ┌──────────────┐
    │ [Cam] [LiDAR]│  ← Camera 4" left of LiDAR
    │   ●     ●    │
    │              │
    │   Ebox Top   │
    └──────────────┘
        REAR

  <---- 4" ---->
  (0.1016m)
```

**Offset from electronics_box center:**
- X: `ebox_length/2 - camera_depth/2` (at front edge)
- Y: +0.1016m (4 inches left, +Y direction)
- Z: `ebox_height/2 + camera_height/2` (on top)

### ROS 2 Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Raw camera images |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration data |

### Camera Frames

- **camera_link**: Physical camera mounting frame (X forward)
- **camera_optical_link**: Optical frame (Z forward, ROS convention)

### Usage Examples

**View camera feed:**
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

**Visualize in RViz2:**
```bash
rviz2
# Add -> By topic -> /camera/image_raw -> Image
```

**Subscribe in Python:**
```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

def image_callback(msg):
    # Convert ROS Image to OpenCV format
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Process image
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Display
    cv2.imshow("Camera", cv_image)
    cv2.waitKey(1)

self.create_subscription(Image, '/camera/image_raw', image_callback, 10)
```

## HERE 3+ RTK GPS

### Hardware Specifications

| Parameter | Value |
|-----------|-------|
| **Model** | HEX/ProfiCNC HERE 3+ |
| **GNSS** | GPS, GLONASS, Galileo, BeiDou |
| **Channels** | 184 |
| **Update Rate** | Up to 10 Hz |
| **Accuracy (Standalone)** | 2.5m CEP |
| **Accuracy (RTK)** | 0.01m + 1ppm (horizontal) |
| **Accuracy (RTK Vertical)** | 0.02m + 1ppm |
| **Time to First Fix** | ~29s (cold start) |
| **Compass** | Integrated magnetometer |
| **Weight** | ~50g |
| **Dimensions** | ~110mm diameter × 16mm (puck style) |

### Mounting Position

- **Location**: Inside electronics box (just below top surface)
- **Position**: To the RIGHT of RP-LIDAR
- **Height**: Near top of electronics box
- **Orientation**: Flat (optimal sky view through transparent top if applicable)
- **Parent frame**: `electronics_box`

### Position in Simulation

```
Cross-Section View (looking from front):
    ┌──────────────────┐
    │  [GPS]  [LiDAR]  │  ← GPS to right of LiDAR
    │    ●       ●     │     (both near top)
    │                  │
    │  Electronics Box │
    │                  │
    └──────────────────┘
```

**Offset from electronics_box center:**
- X: `ebox_length/2 - gps_radius` (near front)
- Y: -0.08m (to the right, -Y direction)
- Z: `ebox_height/2 - gps_height/2 - 0.01` (just below top)

### ROS 2 Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/gps/fix` | `sensor_msgs/NavSatFix` | GPS position fix |
| `/gps/vel` | `geometry_msgs/TwistStamped` | GPS velocity (if available) |

### NavSatFix Message Format

```yaml
header:
  frame_id: gps_link
status:
  status: 0           # 0=no fix, 1=fix, 2=SBAS fix, 3=GBAS/RTK fix
  service: 1          # GPS service
latitude: 37.7749     # Degrees
longitude: -122.4194  # Degrees
altitude: 10.0        # Meters (MSL)
position_covariance: [0.0001, 0, 0,  # 3×3 covariance matrix
                      0, 0.0001, 0,
                      0, 0, 0.0004]
position_covariance_type: 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN
```

### Usage Examples

**View GPS data:**
```bash
ros2 topic echo /gps/fix
```

**Subscribe in Python:**
```python
from sensor_msgs.msg import NavSatFix

def gps_callback(msg):
    lat = msg.latitude
    lon = msg.longitude
    alt = msg.altitude

    # Check fix quality
    if msg.status.status >= 1:
        print(f"GPS Fix: {lat:.6f}, {lon:.6f}, alt={alt:.2f}m")

        # RTK fix has status == 3
        if msg.status.status == 3:
            print("RTK fix active (cm-level accuracy)")
    else:
        print("No GPS fix")

self.create_subscription(NavSatFix, '/gps/fix', gps_callback, 10)
```

## Cube Orange (IMU/Flight Controller)

### Positioning

- **Location**: Inside electronics box
- **Position**: 6 inches (0.1524m) directly BELOW RP-LIDAR
- **Orientation**: IMU arrow facing FORWARD (X+ axis)
- **Purpose**: Provides IMU data (accelerometer, gyroscope, magnetometer)

**Offset from electronics_box center:**
- X: 0 (centered)
- Y: 0 (centered)
- Z: `ebox_height/2 - 0.1524` (6 inches below top)

This positions the Cube Orange:
- Below the LiDAR for optimal sensor fusion
- Centered for balanced IMU readings
- Away from magnetic interference (motors, power distribution)

## Sensor Layout Summary

```
Side View (X-Z plane):
                [LiDAR]
                   ●      ← Top of ebox
              ┌─────────┐
              │    ↓6"  │
              │  [Cube] │  ← Flight controller
              │    ●    │
              │         │
              │ [Jetson]│  ← Lower section
              └─────────┘

Front View (Y-Z plane):
    [Camera]  [LiDAR]  [GPS]
        ●         ●        ●   ← Top of ebox
    <---4"-->
        ┌──────────────────┐
        │                  │
        │  Electronics Box │
        │                  │
        └──────────────────┘
```

## Multi-Sensor Integration

### Sensor Fusion Strategy

The rover's sensors complement each other:

| Sensor | Primary Use | Range | Update Rate |
|--------|-------------|-------|-------------|
| **LiDAR** | Obstacle detection, mapping | 0.15-12m | 10 Hz |
| **Camera** | Visual identification, tracking | 0.2m-∞ | 30 Hz |
| **GPS** | Global positioning | Outdoor only | 10 Hz |
| **IMU (Cube)** | Orientation, motion | N/A | 50+ Hz |
| **Ultrasonics** | Close-range obstacles | 0.2-6m | 20 Hz |

### Typical Applications

**Obstacle Avoidance:**
- Ultrasonics: 0-2m (reactive avoidance)
- LiDAR: 2-10m (path planning)
- Camera: Visual confirmation

**Navigation:**
- GPS: Waypoint navigation (outdoor)
- LiDAR: Local mapping (SLAM)
- IMU: Dead reckoning between updates

**Localization:**
- GPS + IMU: Global position with drift correction
- LiDAR + IMU: Local SLAM
- Wheel odometry: Continuous position estimate

## Simulation Testing

### Launch with All Sensors

```bash
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_sim spawn_rover.launch.py
```

### Verify All Sensor Topics

```bash
# Check all topics are publishing
ros2 topic list | grep -E '(scan|camera|gps)'

# Expected output:
# /scan
# /camera/image_raw
# /camera/camera_info
# /gps/fix
```

### Visualize Everything in RViz2

```bash
rviz2
```

**RViz2 Configuration:**
1. Fixed Frame: `base_link` or `odom`
2. Add:
   - LaserScan (`/scan`) - Red color
   - Image (`/camera/image_raw`)
   - TF tree (show all frames)
   - RobotModel (show rover)

### Test Sensor Data

**LiDAR test:**
```bash
# Place obstacles in Gazebo near rover
# Watch scan data change
ros2 topic hz /scan  # Should show ~10 Hz
```

**Camera test:**
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
# Should see Gazebo world from rover's perspective
```

**GPS test:**
```bash
ros2 topic echo /gps/fix
# Should show current position in Gazebo world
```

## Hardware Integration Notes

### Real Robot Differences

When deploying to actual hardware:

1. **RP-LIDAR A1**: Connected via USB, typically `/dev/ttyUSB0`
   - Use `rplidar_ros` package
   - Adjust scan rate based on performance needs

2. **Logitech C920X**: Standard USB camera
   - Use `usb_cam` or `v4l2_camera` ROS 2 package
   - Configure resolution/framerate in launch file

3. **HERE 3+**: Connected to Cube Orange via CAN or I2C
   - GPS data published through MAVROS from Cube Orange
   - RTK corrections via telemetry link or Wi-Fi

4. **Cube Orange**: Main autopilot
   - MAVROS publishes IMU, GPS, and other sensor data
   - Flight controller handles sensor fusion

### Calibration

Before field deployment:

- [ ] **Camera calibration**: Use `camera_calibration` package
- [ ] **LiDAR alignment**: Verify 0° angle points forward
- [ ] **GPS/IMU fusion**: Configure in ArduPilot parameters
- [ ] **Coordinate frames**: Verify all TF transforms match physical mounting

## Troubleshooting

### LiDAR not publishing

- Check `/scan` topic exists: `ros2 topic list`
- Verify LiDAR spinning in Gazebo
- Check Gazebo sensor visualization (View → Contacts)

### Camera shows black image

- Verify Gazebo is rendering properly
- Check camera facing correct direction
- Increase Gazebo real-time factor if choppy

### GPS not publishing or wrong position

- Verify topic: `ros2 topic echo /gps/fix`
- Check Gazebo world origin (GPS needs global coordinates)
- Confirm GPS plugin loaded: `ros2 param list | grep gps`

### Sensor TF frames missing

```bash
# View TF tree
ros2 run tf2_tools view_frames
# Check generated PDF for missing transforms
```

## Performance Considerations

### Computational Load

| Sensor | CPU Usage | Bandwidth | Notes |
|--------|-----------|-----------|-------|
| LiDAR (10 Hz) | Low | ~50 KB/s | 360 points × 10 Hz |
| Camera (30 Hz, 1080p) | High | ~60 MB/s | Uncompressed RGB |
| GPS (10 Hz) | Minimal | ~1 KB/s | Simple position data |
| Ultrasonics (6×20 Hz) | Minimal | ~5 KB/s | Range-only data |

**Recommendations:**
- Use camera compression for network transmission
- Reduce LiDAR scan rate to 5 Hz if CPU-limited
- Process camera images at lower resolution for real-time tasks

## References

- **RP-LIDAR A1 Datasheet**: https://www.slamtec.com/en/Lidar/A1
- **Logitech C920X Specs**: https://www.logitech.com/en-us/products/webcams/c920x-pro-hd-webcam.html
- **HERE 3+ Documentation**: https://docs.cubepilot.org/user-guides/here-3
- **Gazebo ROS 2 Sensors**: https://github.com/ros-simulation/gazebo_ros_pkgs

---

**Created**: November 8, 2025
**Status**: Simulation configured with all sensors
**Next Step**: Test sensor fusion and navigation algorithms in simulation
