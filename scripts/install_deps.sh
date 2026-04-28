#!/usr/bin/env bash
# install_deps.sh — Full dependency installation for AGV Sightseeing Vehicle
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo "AGV Sightseeing Vehicle - Dependency Installer"
echo "=============================================="

# Check Ubuntu version
if [[ "$(lsb_release -rs)" != "22.04" ]]; then
    echo "WARNING: This script is tested on Ubuntu 22.04 (ROS 2 Humble)"
fi

# -------------------------------------------------------
# System dependencies
# -------------------------------------------------------
echo "[1/6] Installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y \
    curl wget git build-essential cmake \
    python3-pip python3-dev python3-venv \
    python3-colcon-common-extensions \
    libeigen3-dev libpcl-dev \
    libopencv-dev python3-opencv \
    portaudio19-dev libsndfile1-dev \
    can-utils iproute2 \
    libgpiod-dev \
    influxdb influxdb-client \
    mosquitto mosquitto-clients \
    nginx

# -------------------------------------------------------
# ROS 2 Humble
# -------------------------------------------------------
echo "[2/6] Installing ROS 2 Humble..."
if ! command -v ros2 &> /dev/null; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
        sudo apt-key add -
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > \
        /etc/apt/sources.list.d/ros2-latest.list'
    sudo apt-get update -qq
    sudo apt-get install -y ros-humble-desktop
else
    echo "  ROS 2 Humble already installed"
fi

# -------------------------------------------------------
# ROS 2 packages
# -------------------------------------------------------
echo "[3/6] Installing ROS 2 packages..."
sudo apt-get install -y \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav2-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-velodyne \
    ros-humble-velodyne-pointcloud \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    python3-colcon-common-extensions

# -------------------------------------------------------
# Python dependencies
# -------------------------------------------------------
echo "[4/6] Installing Python dependencies..."
pip3 install --upgrade pip
pip3 install \
    numpy scipy \
    casadi \
    filterpy \
    scikit-learn \
    ultralytics \
    opencv-python \
    open3d \
    pyserial \
    python-can \
    influxdb-client \
    websockets \
    aiohttp \
    pyyaml \
    transforms3d \
    pyaudio \
    pydub \
    RPi.GPIO \
    pyproj

# -------------------------------------------------------
# CAN bus setup
# -------------------------------------------------------
echo "[5/6] Setting up CAN bus..."
if ! grep -q "can" /etc/modules; then
    echo "can" | sudo tee -a /etc/modules
    echo "can_raw" | sudo tee -a /etc/modules
    echo "can_dev" | sudo tee -a /etc/modules
fi

sudo modprobe can can_raw can_dev 2>/dev/null || true

# Create systemd service for CAN bus
sudo tee /etc/systemd/system/agv-can.service > /dev/null <<'EOF'
[Unit]
Description=AGV CAN Bus Setup
After=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/ip link set can0 up type can bitrate 500000
ExecStop=/sbin/ip link set can0 down

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable agv-can.service 2>/dev/null || true

# -------------------------------------------------------
# Build workspace
# -------------------------------------------------------
echo "[6/6] Building ROS 2 workspace..."
source /opt/ros/humble/setup.bash
cd "$WORKSPACE_DIR"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "=============================================="
echo "Installation complete!"
echo ""
echo "To use the workspace:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
echo ""
echo "To start the AGV:"
echo "  ./scripts/start_agv.sh"
echo "=============================================="
