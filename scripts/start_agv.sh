#!/usr/bin/env bash
# start_agv.sh — One-command system start for AGV Sightseeing Vehicle
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo "AGV Sightseeing Vehicle — System Startup"
echo "=============================================="

# Source ROS 2
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash" 2>/dev/null || {
    echo "ERROR: Workspace not built. Run ./scripts/build.sh first"
    echo "Building now..."
    bash "${SCRIPT_DIR}/build.sh"
    source "${WORKSPACE_DIR}/install/setup.bash"
}

# Parse arguments
MODE="${1:-real}"
LAUNCH_ARGS=""

case "$MODE" in
    sim|simulation)
        echo "Mode: SIMULATION"
        LAUNCH_FILE="agv_simulation.launch.py"
        LAUNCH_ARGS="use_sim_time:=true"
        ;;
    real|hardware)
        echo "Mode: REAL HARDWARE"
        LAUNCH_FILE="agv_full.launch.py"
        LAUNCH_ARGS="use_sim_time:=false"
        ;;
    localization)
        echo "Mode: LOCALIZATION ONLY"
        LAUNCH_FILE="agv_localization.launch.py"
        ;;
    perception)
        echo "Mode: PERCEPTION ONLY"
        LAUNCH_FILE="agv_perception.launch.py"
        ;;
    *)
        echo "Usage: $0 [sim|real|localization|perception]"
        exit 1
        ;;
esac

# Setup CAN bus for real hardware
if [[ "$MODE" == "real" || "$MODE" == "hardware" ]]; then
    echo "Setting up CAN bus..."
    sudo ip link set can0 up type can bitrate 500000 2>/dev/null || {
        echo "WARNING: CAN bus setup failed (is hardware connected?)"
    }
fi

# System health checks
echo ""
echo "Pre-flight checks:"
echo -n "  ROS 2 environment... "
ros2 --help > /dev/null 2>&1 && echo "OK" || { echo "FAIL"; exit 1; }

echo -n "  AGV packages... "
ros2 pkg list | grep -q agv_bringup && echo "OK" || { echo "FAIL (run build.sh)"; exit 1; }

if [[ "$MODE" == "real" || "$MODE" == "hardware" ]]; then
    echo -n "  CAN bus... "
    ip link show can0 > /dev/null 2>&1 && echo "OK" || echo "WARNING (check wiring)"

    echo -n "  GPS device... "
    ls /dev/ttyUSB0 > /dev/null 2>&1 && echo "OK" || echo "WARNING (not found)"

    echo -n "  IMU device... "
    ls /dev/ttyUSB1 > /dev/null 2>&1 && echo "OK" || echo "WARNING (not found)"
fi

echo ""
echo "Starting AGV system..."
echo "Launch: ${LAUNCH_FILE} ${LAUNCH_ARGS}"
echo "Press CTRL+C to stop."
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down AGV system..."
    # Send emergency stop before killing
    ros2 topic pub -1 /agv/emergency_stop std_msgs/msg/Bool "data: true" \
        2>/dev/null || true
    sleep 1
    # CAN bus cleanup
    if [[ "$MODE" == "real" || "$MODE" == "hardware" ]]; then
        sudo ip link set can0 down 2>/dev/null || true
    fi
    echo "AGV system stopped safely."
    exit 0
}

trap cleanup INT TERM

# Launch the system
ros2 launch agv_bringup "${LAUNCH_FILE}" ${LAUNCH_ARGS}
