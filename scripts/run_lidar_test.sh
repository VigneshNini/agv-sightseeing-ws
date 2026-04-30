#!/bin/bash
# =============================================================================
#  Automated LiDAR Test — One command to start everything
#  Starts: fake LiDAR publisher + RViz2 (pre-configured)
#
#  Usage:
#    chmod +x scripts/run_lidar_test.sh
#    ./scripts/run_lidar_test.sh
# =============================================================================
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.."\ && pwd)"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║   AGV Fake LiDAR Test — Automated Startup           ║"
echo "║   Publishes /scan and opens RViz2 automatically     ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    set +u
    source "$WORKSPACE_DIR/install/setup.bash"
    set -u
else
    echo "  ⚠  Workspace not built yet. Run: ./scripts/build.sh"
    exit 1
fi

echo "  ✔ ROS 2 Humble sourced"
echo "  ✔ Workspace sourced"
echo "  ✔ Launching fake LiDAR test..."
echo ""
echo "  Press Ctrl+C to stop everything."
echo ""

# Launch everything via ROS 2 launch
ros2 launch agv_bringup fake_lidar_test.launch.py
