# Simulation vs Real World Differences

This document tracks known differences between the Gazebo simulation and real hardware that developers need to be aware of when transitioning code between environments.

## GPS Coordinate System

| Aspect | Gazebo Simulation | Real Hardware |
|--------|-------------------|---------------|
| GPS sign convention | **Inverted** - GPS plugin reports negative lat/lon when rover is at positive world coordinates | Normal - GPS reports actual lat/lon matching real position |
| Coordinate origin | Near (0, 0) in world frame | Real-world lat/lon (e.g., 37.7749, -122.4194) |
| RTK fix status | Always `status=0` (no fix simulation) | Real RTK status (0=no fix, 1=float, 2=fix) |

### Code Implications
In `navigation_controller.py`, the `gps_to_local()` function uses `from_sensor=True` to negate GPS readings in simulation. For real hardware, this should be `False` or controlled by a `simulation_mode` parameter.

```python
# Simulation: negate GPS sensor readings
current_x, current_y = self.gps_to_local(lat, lon, from_sensor=True)

# Real hardware: use GPS readings directly
current_x, current_y = self.gps_to_local(lat, lon, from_sensor=False)
```

## IMU / Heading

| Aspect | Gazebo Simulation | Real Hardware |
|--------|-------------------|---------------|
| Heading reference | 0 = +X axis (East), CCW positive | TBD - verify with actual IMU |
| Drift | None (perfect simulation) | May drift over time, needs calibration |
| Magnetic interference | None | Affected by nearby metal/electronics |

## Motor Control

| Aspect | Gazebo Simulation | Real Hardware |
|--------|-------------------|---------------|
| Response time | Instant | Slight delay from motor controllers |
| cmd_vel mapping | Direct to diff_drive plugin | Through MDDS30 motor controller |
| Speed limits | Enforced in navigation code | Also limited by physical motor capabilities |

## LiDAR (RP LiDAR A1)

| Aspect | Gazebo Simulation | Real Hardware |
|--------|-------------------|---------------|
| Driver | Gazebo `libgazebo_ros_ray_sensor.so` | `rplidar_driver` package |
| Topic | `/scan` (may contain inf/nan) | `/scan` (clean data) |
| Sanitization | Required - use `/scan_safe` or `/scan_safe_reliable` | Not required - use `/scan` directly |
| Scan rate | 10 Hz (simulated) | ~5.5 Hz (actual sensor rate) |
| Noise model | Gaussian (stddev 0.01m) | Real sensor noise characteristics |

### Code Implications

**Simulation:** Use the sanitized topics to avoid inf/nan issues:
```python
# Subscribe to sanitized scan in simulation
self.create_subscription(LaserScan, '/scan_safe_reliable', callback, 10)
```

**Real Hardware:** Subscribe directly to `/scan`:
```python
# Subscribe to raw scan on real hardware (no sanitization needed)
self.create_subscription(LaserScan, '/scan', callback, sensor_qos)
```

**Launch Configuration:**
```python
# Simulation: use sanitized topic
remappings=[('/scan_input', '/scan_safe_reliable')]

# Real hardware: use raw topic
remappings=[('/scan_input', '/scan')]
```

## Other Sensors

| Aspect | Gazebo Simulation | Real Hardware |
|--------|-------------------|---------------|
| Ultrasonic | Simulated range sensors | HC-SR04 sensors with timing |
| Camera | Gazebo camera plugin | Real USB/CSI camera |

## Network / API

| Aspect | Gazebo Simulation | Real Hardware |
|--------|-------------------|---------------|
| HTTP Bridge IP | Desktop (100.73.129.15:5001) | Jetson (100.91.191.47:5001) |
| `/api/health` response | `simulation: true` | `simulation: false` |
| Latency | Local/LAN only | May include Tailscale VPN overhead |

## Waypoint Marker

| Aspect | Gazebo Simulation | Real Hardware |
|--------|-------------------|---------------|
| Visual marker | Blue diamond spawned in Gazebo via SpawnEntity service | N/A - no visual marker in real world |
| Marker services | `/spawn_entity`, `/delete_entity` available | Services don't exist |

### Code Implications
The Gazebo marker spawning code will fail silently on real hardware (service not available). This is acceptable behavior - the marker is purely for visualization during simulation testing.

## Recommended Configuration

Add a `simulation_mode` parameter to nodes that need different behavior:

```python
self.declare_parameter('simulation_mode', False)
self.simulation_mode = self.get_parameter('simulation_mode').value
```

Launch file configuration:
```python
# Simulation launch
parameters=[{'simulation_mode': True}]

# Real hardware launch
parameters=[{'simulation_mode': False}]
```

---

## Change Log

| Date | Item Added | Notes |
|------|------------|-------|
| 2025-12-04 | LiDAR differences | Sanitization required in sim, not on real hardware |
| 2025-12-03 | GPS sign inversion | Gazebo GPS plugin reports inverted coordinates |
| 2025-12-03 | Waypoint marker | Gazebo-only visual marker via SpawnEntity |
| 2025-12-03 | Initial document | Created during Phase 2 navigation development |
