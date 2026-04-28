# AGV Sightseeing Vehicle — System Architecture

## Overview
The AGV Sightseeing Vehicle is a fully autonomous electric vehicle that guides passengers
through pre-defined tour routes in parks, museums, and outdoor venues. The software stack
is built on ROS 2 Humble using a modular, safety-first architecture.

## Package Architecture

```
agv-sightseeing-ws/
├── src/
│   ├── agv_msgs/          # Custom message/service definitions
│   ├── agv_bringup/       # Launch files and system config
│   ├── agv_localization/  # Sensor fusion & pose estimation
│   ├── agv_perception/    # Obstacle detection & tracking
│   ├── agv_mapping/       # SLAM & map management
│   ├── agv_planning/      # Path planning & behavior control
│   ├── agv_control/       # Low-level motor & steering control
│   ├── agv_safety/        # Safety monitoring & e-stop
│   ├── agv_tour_manager/  # Tour orchestration & media
│   ├── agv_teleop/        # Manual override interfaces
│   ├── agv_dashboard/     # Web UI & telemetry logging
│   └── agv_simulation/    # Gazebo simulation assets
```

## Topic Graph (Key Topics)

| Topic | Type | Publisher | Subscribers |
|-------|------|-----------|-------------|
| `/agv/pose` | `geometry_msgs/PoseStamped` | ukf_node | planning, safety, dashboard |
| `/agv/odometry` | `nav_msgs/Odometry` | ukf_node | behavior_tree |
| `/agv/gps_enu` | `geometry_msgs/PointStamped` | gps_converter | ukf_node |
| `/agv/wheel_odometry` | `nav_msgs/Odometry` | wheel_odometry | ukf_node |
| `/agv/obstacles` | `agv_msgs/ObstacleArray` | lidar_obstacle_detector | safety, planning, dashboard |
| `/agv/tracked_obstacles` | `agv_msgs/ObstacleArray` | object_tracker | mpc_local_planner |
| `/agv/planned_path` | `nav_msgs/Path` | global_route_manager | mpc_local_planner |
| `/agv/local_path` | `nav_msgs/Path` | mpc_local_planner | lqr_controller |
| `/agv/vehicle_command` | `agv_msgs/VehicleCommand` | mpc_local_planner | safety_arbiter |
| `/agv/safe_command` | `agv_msgs/VehicleCommand` | safety_arbiter | can_bus_interface |
| `/agv/status` | `agv_msgs/AGVStatus` | behavior_tree | dashboard |
| `/agv/safety_level` | `std_msgs/String` | safety_arbiter | behavior_tree |
| `/agv/tour_stop` | `agv_msgs/TourStop` | tour_manager | behavior_tree, display |
| `/agv/current_tour_stop` | `std_msgs/Int32` | behavior_tree | tour_manager |

## Node Descriptions

### agv_localization
- **ukf_node**: 15-DOF Unscented Kalman Filter fusing IMU, GPS (ENU), and wheel odometry
- **gps_converter_node**: WGS84 → ENU coordinate conversion using Haversine formula
- **wheel_odometry_node**: Ackermann drive odometry from wheel encoder ticks
- **lidar_odometry_node**: 2D ICP scan matching for dead-reckoning backup

### agv_perception
- **lidar_obstacle_detector**: Euclidean clustering on 3D LiDAR point clouds
- **object_tracker_node**: SORT-style Kalman filter multi-object tracking
- **camera_perception_node**: Lane detection (Hough) + obstacle classification
- **driveable_surface_node**: RANSAC ground plane estimation

### agv_mapping
- **ndt_mapping_node**: NDT-based voxel map building for pre-mapped environments
- **octomap_node**: 3D occupancy grid with log-odds updates
- **map_server_node**: Serves `nav_msgs/OccupancyGrid` from PGM+YAML files

### agv_planning
- **global_route_manager**: Tour stop sequencing + global path computation via A*
- **hybrid_astar_planner**: Hybrid A* with Ackermann kinematic constraints for parking/tight spaces
- **mpc_local_planner**: Model Predictive Control for obstacle-aware trajectory following
- **velocity_profiler**: Accel/decel/curvature speed limit computation
- **behavior_tree_node**: Mission state machine: IDLE → NAVIGATE → DWELL → NEXT_STOP

### agv_control
- **lqr_controller**: Linear Quadratic Regulator for lateral path tracking
- **can_bus_interface**: CAN bus hardware abstraction (socket CAN + simulation fallback)
- **motor_controller_node**: PID-based speed control per wheel
- **steering_controller**: Rate-limited servo steering with PWM output

### agv_safety
- **safety_arbiter_node**: Central command filtering with 4-level safety escalation
- **emergency_stop_node**: Hardware GPIO e-stop with RPi.GPIO interface
- **collision_monitor_node**: Time-to-collision calculation and warning
- **system_watchdog_node**: Topic heartbeat monitoring with timeout detection

### agv_tour_manager
- **tour_manager_node**: Loads `tour_stops.yaml`, sequences audio and display triggers
- **audio_player_node**: Subprocess-based audio playback (non-blocking, threaded)
- **passenger_display_node**: JSON content publisher for screen/tablet displays

### agv_dashboard
- **web_bridge_node**: AsyncIO WebSocket server (port 9090) bridging ROS 2 → browser
- **telemetry_logger_node**: CSV telemetry logging with automatic file rotation

## Safety Architecture

The safety system uses a layered approach:

```
Hardware E-Stop (GPIO) ──────────────────────────► EMERGENCY
                                                        │
System Watchdog (topic timeouts) ───────────────► WARNING/EMERGENCY
                                                        │
Collision Monitor (TTC < 1s) ───────────────────► EMERGENCY
                                                        │
Safety Arbiter (obstacle distance) ─────────────► CAUTION/WARNING
                                                        │
                                              All commands filtered
                                              before reaching motors
```

## Coordinate Frames (TF Tree)

```
map
└── odom
    └── base_link
        ├── lidar_link
        ├── camera_link
        ├── imu_link
        └── gps_link
```
