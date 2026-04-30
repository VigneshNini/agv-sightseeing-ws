#!/bin/bash
# =========================================================
#  AGV LiDAR Test — One script to see LiDAR in RViz2
#  Usage: ./scripts/lidar_test.sh
# =========================================================

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.."\ && pwd)"

source /opt/ros/humble/setup.bash

echo ""
echo "  Starting Fake LiDAR publisher on /scan ..."
python3 "$WS/scripts/fake_lidar_publisher.py" &
LIDAR_PID=$!

sleep 2

echo "  Opening RViz2 ..."
ros2 run rviz2 rviz2 -d "$WS/config/lidar_rviz.rviz"

# When RViz2 closes, kill the publisher too
kill $LIDAR_PID 2>/dev/null
echo "  Done."
