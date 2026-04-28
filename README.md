# AGV Sightseeing Vehicle — ROS 2 Workspace

A production-grade autonomous sightseeing vehicle platform built on **ROS 2 Humble**. The system autonomously navigates predefined tour routes, providing passengers with audio commentary, real-time telemetry, and a live web dashboard.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    AGV Sightseeing Vehicle Stack                │
├──────────────┬──────────────┬──────────────┬────────────────────┤
│ Localization │  Perception  │   Planning   │      Safety        │
│  UKF (15DOF) │ LiDAR Clust │  Hybrid A*   │  Safety Arbiter     │
│  GPS ENU     │ SORT Tracker │  MPC Planner │  E-Stop (GPIO)     │
│  Wheel Odom  │ Lane Detect  │  Behavior SM │  Watchdog          │
│  LiDAR ICP   │ RANSAC Gnd  │  Vel Profile │  Collision Mon      │
├──────────────┴──────────────┴──────────────┴────────────────────┤
│   Control: LQR + CAN Bus + PID Motors + Steering Controller     │
├─────────────────────────────────────────────────────────────────┤
│   Tour: Manager + Audio Player + Passenger Display              │
├─────────────────────────────────────────────────────────────────┤
│   Dashboard: WebSocket Bridge + Telemetry Logger + Web UI       │
└─────────────────────────────────────────────────────────────────┘
```

## Packages

| Package | Description |
|---------|-------------|
| `agv_msgs` | Custom ROS 2 messages & services |
| `agv_bringup` | Launch files and system parameters |
| `agv_localization` | UKF sensor fusion, GPS, wheel & LiDAR odometry |
| `agv_perception` | LiDAR clustering, object tracking, lane detection |
| `agv_mapping` | NDT mapping, OctoMap, map server |
| `agv_planning` | Global route, Hybrid A*, MPC, behavior state machine |
| `agv_control` | LQR, CAN bus interface, motor & steering controllers |
| `agv_safety` | Safety arbiter, emergency stop, collision monitor, watchdog |
| `agv_tour_manager` | Tour orchestration, audio playback, display publisher |
| `agv_teleop` | Joystick & keyboard manual override |
| `agv_dashboard` | Web UI (WebSocket + canvas map + speed gauge) |
| `agv_simulation` | Gazebo URDF, world file, simulation launch |

## Quick Start

### Simulation
```bash
# Prerequisites: ROS 2 Humble + Gazebo
source /opt/ros/humble/setup.bash
cd agv-sightseeing-ws

# Build
./scripts/build.sh

# Run simulation
./scripts/run_simulation.sh
```

### Real Hardware
```bash
source install/setup.bash
ros2 launch agv_bringup agv_full.launch.py
```

### Docker
```bash
docker compose build
docker compose up -d
# With simulation:
docker compose --profile simulation up
```

### Dashboard
Open `http://localhost:8080` in a browser after launching the stack.

## Configuration

- **System parameters**: `config/agv_params.yaml`
- **Tour stops**: `config/tour_stops.yaml`
- **Maps**: `maps/` (PGM + YAML format)

## Documentation

- [System Architecture](docs/architecture.md)
- [Deployment Guide](docs/deployment.md)
- [Map Directory Guide](maps/README.md)

## Safety

The system implements a 4-level safety escalation:
1. **SAFE** — Normal operation
2. **CAUTION** — Obstacle < 5m, speed reduced
3. **WARNING** — Obstacle < 2m or watchdog timeout, near stop
4. **EMERGENCY** — Obstacle < 0.5m, hardware e-stop, full stop

All commands pass through the `safety_arbiter_node` before reaching motors.

## License

Apache License 2.0
