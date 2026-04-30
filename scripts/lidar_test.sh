#!/bin/bash
# =========================================================
#  AGV LiDAR Test — One script to see LiDAR in RViz2
#  Usage: ./scripts/lidar_test.sh
# =========================================================

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.."\ && pwd)"

source /opt/ros/humble/setup.bash

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║   AGV Fake LiDAR Test                               ║"
echo "║   /scan → RViz2  (laser_frame)                     ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# 1. Publish static TF: map -> laser_frame
#    Without this RViz2 shows 'No tf data' and nothing renders
echo "  ✔ Starting TF publisher  (map → laser_frame) ..."
ros2 run tf2_ros static_transform_publisher \
    --frame-id map \
    --child-frame-id laser_frame \
    --x 0 --y 0 --z 0.3 \
    --roll 0 --pitch 0 --yaw 0 &
TF_PID=$!

sleep 1

# 2. Start fake LiDAR publisher
echo "  ✔ Starting Fake LiDAR publisher on /scan ..."
python3 "$WS/scripts/fake_lidar_publisher.py" &
LIDAR_PID=$!

sleep 2

# 3. Open RViz2 with pre-configured layout
echo "  ✔ Opening RViz2 ..."
echo "  Press Ctrl+C or close RViz2 window to stop."
echo ""
ros2 run rviz2 rviz2 -d "$WS/config/lidar_rviz.rviz"

# Cleanup when RViz2 closes
kill $LIDAR_PID 2>/dev/null
kill $TF_PID 2>/dev/null
echo ""
echo "  Done. All processes stopped."
