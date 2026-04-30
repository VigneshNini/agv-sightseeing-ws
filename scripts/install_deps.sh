#!/usr/bin/env bash
# =============================================================================
#  AGV Sightseeing Vehicle — Full Installation Script
#  Tested on: Ubuntu 22.04.3 LTS (VMware / Native / Jetson)
#  Usage:
#    chmod +x install_deps.sh
#    ./install_deps.sh
# =============================================================================
set -eo pipefail
# Note: -u (unbound variable check) is intentionally omitted at the top level
# because ROS 2 setup scripts reference variables before defining them.
# We re-enable it after all sourcing is done.

# ── Colours ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
LOG_FILE="$HOME/agv_install.log"

step()  { echo -e "\n${CYAN}${BOLD}[$1]${NC} $2"; }
ok()    { echo -e "  ${GREEN}✔${NC} $1"; }
warn()  { echo -e "  ${YELLOW}⚠${NC}  $1"; }
die()   { echo -e "  ${RED}✘ ERROR:${NC} $1"; exit 1; }

echo -e "${BOLD}"
echo "╔══════════════════════════════════════════════════════╗"
echo "║   AGV Sightseeing Vehicle — Complete Installer       ║"
echo "║   Ubuntu 22.04 LTS + ROS 2 Humble + All Deps        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo "  Log file: $LOG_FILE"
echo "  Workspace: $WORKSPACE_DIR"
echo ""

# ── Pre-flight checks ────────────────────────────────────────────────────────
step "0/9" "Pre-flight checks"

# Ubuntu version check
UBUNTU_VER=$(lsb_release -rs 2>/dev/null || echo "unknown")
if [[ "$UBUNTU_VER" != "22.04" ]]; then
    warn "This script targets Ubuntu 22.04. Detected: $UBUNTU_VER"
else
    ok "Ubuntu 22.04 detected"
fi

# Architecture
ARCH=$(dpkg --print-architecture)
ok "Architecture: $ARCH"

# Internet check using ping (works even without curl)
if ping -c 1 -W 5 8.8.8.8 &>/dev/null; then
    ok "Internet connection OK"
else
    die "No internet. Fix: VM Settings → Network Adapter → NAT → OK, then retry."
fi

# Disk space check
FREE_GB=$(df -BG "$HOME" | awk 'NR==2 {print $4}' | tr -d 'G')
if [[ "$FREE_GB" -lt 20 ]]; then
    warn "Less than 20 GB free (${FREE_GB} GB). Recommend 80+ GB VM disk."
else
    ok "Disk space: ${FREE_GB} GB free"
fi

# ── 1. System Base Packages ───────────────────────────────────────────────────
step "1/9" "Installing system base packages"

sudo apt-get update -qq 2>&1 | tail -1

# Install curl + wget first
sudo apt-get install -y curl wget 2>&1 >> "$LOG_FILE"
ok "curl/wget installed"

sudo apt-get install -y \
    git \
    build-essential cmake ninja-build \
    software-properties-common \
    lsb-release gnupg2 ca-certificates \
    python3-pip python3-dev python3-venv python3-setuptools python3-wheel \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev python3-opencv \
    portaudio19-dev libsndfile1-dev \
    can-utils iproute2 net-tools \
    htop tmux nano vim \
    libusb-1.0-0-dev \
    mosquitto mosquitto-clients \
    nginx \
    v4l-utils \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    2>&1 >> "$LOG_FILE"

sudo apt-get install -y influxdb influxdb-client 2>&1 >> "$LOG_FILE" \
    || warn "influxdb not in apt — will be available via Docker instead"

ok "System packages installed"

# ── 2. ROS 2 Humble ──────────────────────────────────────────────────────────
step "2/9" "Installing ROS 2 Humble"

if command -v ros2 &>/dev/null; then
    ok "ROS 2 already installed — skipping"
else
    sudo add-apt-repository universe -y 2>&1 >> "$LOG_FILE"

    # Modern GPG method (no deprecated apt-key)
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt-get update -qq 2>&1 >> "$LOG_FILE"
    sudo apt-get install -y ros-humble-desktop-full 2>&1 >> "$LOG_FILE"
    ok "ROS 2 Humble installed"
fi

# ROS setup scripts reference variables before defining them (e.g. AMENT_PYTHON_EXECUTABLE).
# Temporarily disable -u to prevent 'unbound variable' crash.
set +u
source /opt/ros/humble/setup.bash
set -u

if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ok "Added ROS 2 to ~/.bashrc"
fi
ok "ROS 2 Humble sourced"

# ── 3. ROS 2 Packages ────────────────────────────────────────────────────────
step "3/9" "Installing ROS 2 packages"

sudo apt-get install -y \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-nav2-msgs \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros2-control \
    ros-humble-velodyne \
    ros-humble-velodyne-pointcloud \
    ros-humble-velodyne-laserscan \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard \
    ros-humble-robot-localization \
    ros-humble-imu-tools \
    ros-humble-nmea-msgs \
    ros-humble-gps-msgs \
    ros-humble-rosbridge-server \
    ros-humble-rosbridge-suite \
    ros-humble-octomap-ros \
    ros-humble-octomap-msgs \
    ros-humble-slam-toolbox \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    2>&1 >> "$LOG_FILE"
ok "ROS 2 packages installed"

# ── 4. Python Dependencies ────────────────────────────────────────────────────
step "4/9" "Installing Python dependencies"

pip3 install --upgrade pip setuptools wheel 2>&1 >> "$LOG_FILE"

pip3 install \
    "numpy>=1.21,<2.0" \
    scipy \
    casadi \
    filterpy \
    scikit-learn \
    matplotlib \
    pandas \
    ultralytics \
    opencv-python-headless \
    open3d \
    pyserial \
    python-can \
    influxdb-client \
    websockets \
    aiohttp \
    aiofiles \
    pyyaml \
    transforms3d \
    pyaudio \
    pydub \
    pyproj \
    utm \
    smbus2 \
    dynamixel-sdk \
    2>&1 >> "$LOG_FILE"

ok "Python packages installed"

python3 -c "import casadi; print('    version:', casadi.__version__)" 2>/dev/null \
    && ok "CasADi OK" || warn "CasADi not found — MPC will use sampling fallback"
python3 -c "import numpy" 2>/dev/null && ok "NumPy OK"
python3 -c "import scipy" 2>/dev/null && ok "SciPy OK"
python3 -c "import cv2"   2>/dev/null && ok "OpenCV OK"

# ── 5. Docker ────────────────────────────────────────────────────────────────
step "5/9" "Installing Docker + Docker Compose"

if command -v docker &>/dev/null; then
    ok "Docker already installed — skipping"
else
    curl -fsSL https://get.docker.com -o /tmp/get-docker.sh
    sudo sh /tmp/get-docker.sh 2>&1 >> "$LOG_FILE"
    sudo usermod -aG docker "$USER"
    ok "Docker installed (logout & back in to use without sudo)"
fi

if ! docker compose version &>/dev/null 2>&1; then
    sudo apt-get install -y docker-compose-plugin 2>&1 >> "$LOG_FILE"
fi
ok "Docker Compose OK"

# ── 6. CAN Bus ───────────────────────────────────────────────────────────────
step "6/9" "Configuring CAN bus"

for mod in can can_raw can_dev vcan; do
    sudo modprobe $mod 2>/dev/null \
        && ok "$mod loaded" \
        || warn "$mod not available (normal in VM)"
done

if ! grep -q "^can$" /etc/modules 2>/dev/null; then
    printf "can\ncan_raw\ncan_dev\n" | sudo tee -a /etc/modules > /dev/null
fi

sudo ip link add dev vcan0 type vcan 2>/dev/null \
    && sudo ip link set up vcan0 \
    && ok "vcan0 created for VM testing" \
    || ok "vcan0 already exists"

sudo tee /etc/systemd/system/agv-can.service > /dev/null <<'CANSVC'
[Unit]
Description=AGV CAN Bus Interface
After=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c '/sbin/ip link set can0 up type can bitrate 500000 || true'
ExecStop=/bin/bash -c '/sbin/ip link set can0 down || true'

[Install]
WantedBy=multi-user.target
CANSVC

sudo systemctl daemon-reload
sudo systemctl enable agv-can.service 2>/dev/null || true
ok "CAN systemd service installed"

# ── 7. rosdep ────────────────────────────────────────────────────────────────
step "7/9" "Initialising rosdep"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init 2>&1 >> "$LOG_FILE"
fi
rosdep update 2>&1 >> "$LOG_FILE"
ok "rosdep updated"

cd "$WORKSPACE_DIR"
if [ -d src ]; then
    rosdep install --from-paths src --ignore-src -r -y 2>&1 >> "$LOG_FILE" \
        && ok "Workspace ROS deps installed" \
        || warn "Some rosdep packages missing — check $LOG_FILE"
fi

# ── 8. Build the AGV Workspace ────────────────────────────────────────────────
step "8/9" "Building AGV workspace"

cd "$WORKSPACE_DIR"
set +u
source /opt/ros/humble/setup.bash
set -u

colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    2>&1 | tee -a "$LOG_FILE" | grep -E "(Starting|Finished|Failed|ERROR)" || true

SETUP_LINE="source $WORKSPACE_DIR/install/setup.bash"
if ! grep -qF "$SETUP_LINE" ~/.bashrc; then
    echo "$SETUP_LINE" >> ~/.bashrc
fi
set +u
source "$WORKSPACE_DIR/install/setup.bash" 2>/dev/null || true
set -u
ok "Workspace built and sourced"

# ── 9. Verify ────────────────────────────────────────────────────────────────
step "9/9" "Verifying installation"

echo ""
echo -e "  ${BOLD}Component Status:${NC}"
printf "  %-28s" "Ubuntu:";  lsb_release -d | cut -f2
printf "  %-28s" "ROS 2:";   ros2 --version 2>/dev/null || echo "NOT FOUND"
printf "  %-28s" "Python:";  python3 --version
printf "  %-28s" "CasADi:";  python3 -c "import casadi; print(casadi.__version__)" 2>/dev/null || echo "NOT FOUND"
printf "  %-28s" "OpenCV:";  python3 -c "import cv2; print(cv2.__version__)" 2>/dev/null || echo "NOT FOUND"
printf "  %-28s" "Docker:";  docker --version 2>/dev/null || echo "NOT FOUND"

echo ""
echo -e "  ${BOLD}AGV Packages:${NC}"
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
source "$WORKSPACE_DIR/install/setup.bash" 2>/dev/null || true
set -u
if ros2 pkg list 2>/dev/null | grep -q agv; then
    ros2 pkg list 2>/dev/null | grep agv | while read pkg; do
        echo -e "  ${GREEN}✔${NC} $pkg"
    done
else
    warn "AGV packages not found — check $LOG_FILE"
fi

# ── Done ─────────────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}${BOLD}"
echo "╔══════════════════════════════════════════════════════╗"
echo "║        ✔  Installation Complete!                    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo "  Next steps:"
echo ""
echo -e "  ${BOLD}1. Reload terminal:${NC}  source ~/.bashrc"
echo -e "  ${BOLD}2. Run simulation:${NC}   ros2 launch agv_bringup agv_simulation.launch.py"
echo -e "  ${BOLD}3. Start full AGV:${NC}   ros2 launch agv_bringup agv_full.launch.py"
echo ""
echo "  Full install log: $LOG_FILE"
echo ""
