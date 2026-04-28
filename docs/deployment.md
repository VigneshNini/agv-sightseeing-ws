# AGV Sightseeing Vehicle — Deployment Guide

## Hardware Requirements

### Minimum Configuration
- **Compute**: NVIDIA Jetson Xavier NX (8GB) or equivalent x86_64 with 16GB RAM
- **LiDAR**: Velodyne VLP-16 (or compatible 3D LiDAR)
- **Camera**: USB/MIPI RGB camera, 1080p, 30fps
- **IMU**: 6-DOF IMU (e.g., Xsens MTi-3, Vectornav VN-100)
- **GPS**: u-blox F9P with RTK for cm-level accuracy
- **CAN Bus**: USB-CAN adapter (e.g., Peak PCAN-USB) connected to motor controllers

### Recommended Hardware
- **Compute**: NVIDIA Orin NX 16GB
- **LiDAR**: Velodyne VLP-32 or Ouster OS1-64
- **Camera**: Intel RealSense D435i (RGB-D + IMU)
- **GPS**: u-blox F9P + external RTK correction

## Software Prerequisites

### Install ROS 2 Humble
```bash
# Ubuntu 22.04 (Jammy)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg]     http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |     sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-desktop python3-rosdep
sudo rosdep init && rosdep update
```

### Install System Dependencies
```bash
sudo apt install -y     ros-humble-tf2-ros ros-humble-nav-msgs ros-humble-sensor-msgs     ros-humble-cv-bridge ros-humble-xacro ros-humble-robot-state-publisher     python3-scipy python3-numpy python3-yaml can-utils

pip3 install websockets
```

## Build the Workspace

```bash
cd ~/agv-sightseeing-ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Running on Real Hardware

### 1. Configure CAN Bus
```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

### 2. Source and Launch
```bash
source install/setup.bash
ros2 launch agv_bringup agv_full.launch.py
```

### 3. Dashboard
Open browser: `http://<vehicle-ip>:8080`

## Running in Simulation

```bash
./scripts/run_simulation.sh
```

Or with Docker:
```bash
docker compose --profile simulation up
```

## Docker Deployment

### Build Images
```bash
docker compose build
```

### Start Core Stack
```bash
docker compose up -d ros2-core localization perception planning safety tour_manager dashboard
```

### Start with Simulation
```bash
docker compose --profile simulation up
```

## Configuration

Edit `config/agv_params.yaml` for tuning:
- UKF noise covariances (Q, R matrices)
- Safety distance thresholds
- MPC prediction horizon and weights
- Tour stop dwell times

Edit `config/tour_stops.yaml` for tour content:
- GPS coordinates (lat/lon or ENU x/y)
- Dwell times per stop
- Audio file paths
- Description text

## Monitoring

### Check All Nodes Running
```bash
ros2 node list
```

### Monitor Safety Level
```bash
ros2 topic echo /agv/safety_level
```

### View AGV Status
```bash
ros2 topic echo /agv/status
```

### View Dashboard
Navigate to `http://localhost:8080` in a browser.

## Troubleshooting

| Issue | Likely Cause | Fix |
|-------|-------------|-----|
| CAN not connecting | Interface down | `sudo ip link set up can0` |
| GPS no fix | Antenna blocked | Move vehicle outdoors |
| High localization drift | IMU bias | Restart ukf_node |
| E-stop triggered | Safety threshold crossed | Clear obstacles, reset via service |
| WebSocket disconnected | Network issue | Dashboard auto-reconnects in 3s |
