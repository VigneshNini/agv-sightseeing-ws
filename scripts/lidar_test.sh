#!/bin/bash
# =========================================================
#  AGV LiDAR Test — One script to see LiDAR in RViz2
#  Usage: ./scripts/lidar_test.sh
# =========================================================

RVIZ_CONFIG="$HOME/agv-sightseeing-ws/config/lidar_rviz.rviz"
LIDAR_SCRIPT="$HOME/agv-sightseeing-ws/scripts/fake_lidar_publisher.py"

source /opt/ros/humble/setup.bash

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║   AGV Fake LiDAR Test                           ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""

# Check config file exists
if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "  ✘ RViz config not found: $RVIZ_CONFIG"
    echo "    Run: cd ~/agv-sightseeing-ws && git pull"
    exit 1
fi

# Check publisher script exists
if [ ! -f "$LIDAR_SCRIPT" ]; then
    echo "  ✘ Publisher not found: $LIDAR_SCRIPT"
    echo "    Run: cd ~/agv-sightseeing-ws && git pull"
    exit 1
fi

echo "  ✔ Config  : $RVIZ_CONFIG"
echo "  ✔ Script  : $LIDAR_SCRIPT"
echo ""

# 1. Publish static TF: map -> laser_frame
echo "  ▶ Starting TF publisher (map → laser_frame) ..."
ros2 run tf2_ros static_transform_publisher \
    --frame-id map \
    --child-frame-id laser_frame \
    --x 0 --y 0 --z 0.3 \
    --roll 0 --pitch 0 --yaw 0 &
TF_PID=$!
sleep 1

# 2. Start fake LiDAR publisher
echo "  ▶ Starting Fake LiDAR on /scan ..."
python3 "$LIDAR_SCRIPT" &
LIDAR_PID=$!
sleep 2

# 3. Open RViz2 with saved config using exact full path
echo ""
echo "  ▶ Opening RViz2 with saved config..."
echo "     $RVIZ_CONFIG"
echo ""
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG"

# Cleanup on exit
kill $LIDAR_PID 2>/dev/null
kill $TF_PID 2>/dev/null
echo ""
echo "  Done. All processes stopped."
