# AGV Sightseeing Vehicle — Setup Guide

## Prerequisites

- Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS 2 Humble Hawksbill
- NVIDIA Jetson Orin or x86_64 with CUDA GPU (for perception)
- Python 3.10+

---

## 1. Hardware Setup

### 1.1 Mount Sensors

Mount sensors according to the positions in `config/sensors.yaml`:

| Sensor | Location | Orientation |
|--------|----------|-------------|
| Velodyne VLP-16 | Roof center | Upright, 0° heading |
| u-blox ZED-F9P | Roof (near center) | GPS antenna skyward |
| VectorNav VN-100 | Vehicle center (low) | Level, aligned with forward |
| RealSense D435i | Front, 1m height | Slightly downward (-5°) |

### 1.2 Network Setup

1. Connect the Velodyne LiDAR via Ethernet:
   ```bash
   sudo ip addr add 192.168.1.100/24 dev eth0
   sudo ip link set eth0 up
   # Verify LiDAR is reachable
   ping 192.168.1.201
   ```

2. Set up CAN bus:
   ```bash
   sudo ip link set can0 up type can bitrate 500000
   ip link show can0  # Should show UP
   ```

---

## 2. Software Installation

### 2.1 Install ROS 2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] \
    http://packages.ros.org/ros2/ubuntu jammy main" > \
    /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Desktop
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions -y
```

### 2.2 Clone and Install Dependencies

```bash
git clone https://github.com/VigneshNini/agv-sightseeing-ws.git
cd agv-sightseeing-ws

# Run the dependency installer
chmod +x scripts/install_deps.sh
./scripts/install_deps.sh
```

### 2.3 Build the Workspace

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 3. Configuration

### 3.1 GPS Datum Setup

Set the local origin (datum) for ENU coordinate conversion:

1. Drive the vehicle to the map origin (typically the start of the tour)
2. Record the GPS coordinates from `/fix` topic
3. Update `config/sensors.yaml`:
   ```yaml
   gps:
     datum:
       latitude: <your_latitude>
       longitude: <your_longitude>
       altitude: <your_altitude>
   ```
4. Also update `src/agv_bringup/config/agv_params.yaml`:
   ```yaml
   gps_converter:
     ros__parameters:
       datum_latitude: <your_latitude>
       datum_longitude: <your_longitude>
   ```

### 3.2 Tour Stop Configuration

Edit `src/agv_tour_manager/config/tour_config.yaml`:

```yaml
tour_stops:
  - stop_id: 1
    name: "Main Entrance"
    latitude: <lat>       # GPS coordinates of each stop
    longitude: <lon>
    dwell_time: 45.0      # Seconds to pause at stop
    audio_file: "audio/01_entrance.mp3"
```

### 3.3 Sensor Calibration

Run the calibration tool:
```bash
./scripts/calibrate_sensors.sh
```

Follow the prompts for each sensor.

---

## 4. Map Building

### 4.1 Drive the Tour Route for SLAM

```bash
# Terminal 1: Start sensors only
ros2 launch agv_bringup agv_localization.launch.py

# Terminal 2: Start map building
./scripts/build_map.sh my_tour_map

# Terminal 3: Drive manually using keyboard
ros2 run agv_teleop keyboard_teleop_node
```

Drive the entire tour route slowly (< 1 m/s). Press CTRL+C in Terminal 2 when complete.

### 4.2 Verify the Map

```bash
# Open RViz2 and load the saved map
ros2 run rviz2 rviz2
# Add: Map display, set topic to /map
```

---

## 5. First Run (Simulation)

```bash
# Launch full system in simulation
./scripts/run_simulation.sh

# Or manually:
source install/setup.bash
ros2 launch agv_bringup agv_simulation.launch.py
```

Open the web dashboard at `http://localhost:8080`

---

## 6. First Run (Real Hardware)

### 6.1 Pre-flight Checks

```bash
# Check all hardware is connected
./scripts/start_agv.sh real

# The script will report status of:
# - ROS 2 environment
# - AGV packages
# - CAN bus
# - GPS device
# - IMU device
```

### 6.2 Verify Sensor Data

```bash
# In separate terminals:
ros2 topic echo /fix                    # GPS fix
ros2 topic echo /imu/data               # IMU
ros2 topic hz /velodyne_points          # LiDAR (should be ~10 Hz)
ros2 topic echo /agv/pose               # Localization
```

### 6.3 Test Manual Control

```bash
ros2 run agv_teleop keyboard_teleop_node
```

Keys: `w/s` = forward/back, `a/d` = left/right, `space` = stop

### 6.4 Start Autonomous Tour

```bash
# Set start mode to autonomous
ros2 service call /agv/set_tour_stop agv_msgs/srv/SetTourStop \
    "{stop_id: 1, stop_name: 'Main Entrance'}"

# The vehicle will begin the tour automatically
```

---

## 7. Docker Deployment

### 7.1 Build Images

```bash
docker compose build
```

### 7.2 Start Full Stack

```bash
docker compose up -d
```

Services started:
- `ros2` — Main ROS 2 system
- `dashboard` — Web UI on port 8080
- `influxdb` — Telemetry database on port 8086
- `grafana` — Analytics dashboard on port 3000
- `mosquitto` — MQTT broker on port 1883

### 7.3 Start with Simulation

```bash
docker compose --profile simulation up -d
```

---

## 8. Monitoring

### Web Dashboard
- URL: `http://<vehicle-ip>:8080`
- Shows: Live map, speed, battery, camera feed, tour status
- WebSocket: `ws://<vehicle-ip>:9090`

### Grafana Analytics
- URL: `http://<vehicle-ip>:3000`
- Default credentials: admin/agv2024
- Pre-configured dashboard: "AGV Sightseeing Vehicle Telemetry"

### ROS 2 Monitoring
```bash
# Node graph
ros2 run rqt_graph rqt_graph

# Topic monitor
ros2 run rqt_topic rqt_topic

# TF tree
ros2 run tf2_tools view_frames
```

---

## 9. Troubleshooting

### Vehicle doesn't move
1. Check E-stop: `ros2 topic echo /agv/emergency_stop`
2. Check CAN bus: `candump can0`
3. Check safety arbiter: `ros2 topic echo /agv/system_health`

### Poor localization
1. Check GPS fix quality: `ros2 topic echo /fix | grep status`
2. Verify RTK corrections received
3. Check IMU calibration
4. Ensure LiDAR scan matching is working

### Obstacle detection issues
1. Verify LiDAR data: `ros2 topic hz /velodyne_points`
2. Check ground removal parameters in `agv_params.yaml`
3. Visualize in RViz2: `/agv/obstacles` as MarkerArray

### Camera not detected
1. Check USB connection: `ls /dev/video*`
2. Verify RealSense SDK: `realsense-viewer`
3. Check topic: `ros2 topic hz /camera/color/image_raw`

---

## 10. Maintenance

### Battery Care
- Charge after each tour day (LiFePO4: charge to 100%, store at 50%)
- Never discharge below 10%
- Check BMS status daily

### Software Updates
```bash
cd agv-sightseeing-ws
git pull
colcon build --symlink-install
```

### Log Management
```bash
# View ROS 2 logs
~/.ros/log/

# InfluxDB data retention
# Default: 30 days (configured in docker-compose.yml)
```
