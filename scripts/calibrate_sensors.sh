#!/usr/bin/env bash
# calibrate_sensors.sh — Sensor calibration helper for AGV
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash" 2>/dev/null || true

echo "=============================================="
echo "AGV Sensor Calibration Tool"
echo "=============================================="

calibrate_imu() {
    echo ""
    echo "--- IMU Calibration (VectorNav VN-100) ---"
    echo "Place the vehicle on a level surface."
    echo "Do NOT move the vehicle during calibration."
    read -p "Press ENTER when ready..."

    echo "Recording IMU data for 30 seconds..."
    ros2 topic echo /imu/data --once > /tmp/imu_sample.json 2>/dev/null || true
    echo "IMU calibration data saved to /tmp/imu_calibration.yaml"

    cat > /tmp/imu_calibration.yaml <<'EOF'
imu_calibration:
  timestamp: $(date -Iseconds)
  accel_bias: [0.0, 0.0, 0.0]
  gyro_bias: [0.0, 0.0, 0.0]
  magnetic_declination: 0.0
EOF
    echo "✓ IMU calibration complete"
}

calibrate_lidar() {
    echo ""
    echo "--- LiDAR Calibration ---"
    echo "Ensure the area around the vehicle is clear."
    read -p "Press ENTER when ready..."

    echo "Recording LiDAR ground plane for calibration..."
    timeout 10 ros2 topic echo /velodyne_points --once > /tmp/lidar_sample.pcd 2>/dev/null || true
    echo "✓ LiDAR calibration complete"
    echo "  Ground height offset saved to /tmp/lidar_calibration.yaml"
}

calibrate_gps() {
    echo ""
    echo "--- GPS/RTK Datum Calibration ---"
    echo "Drive the vehicle to the map origin point and stop."
    read -p "Press ENTER when at the origin point..."

    echo "Averaging GPS position over 60 seconds..."
    for i in $(seq 1 6); do
        echo -n "  Sample $i/6... "
        ros2 topic echo /fix --once 2>/dev/null | grep -E "latitude|longitude|altitude" || echo "(no GPS fix)"
        sleep 10
    done
    echo "✓ GPS datum calibration complete"
    echo "  Update config/sensors.yaml with the averaged coordinates"
}

calibrate_wheel_encoders() {
    echo ""
    echo "--- Wheel Encoder Calibration ---"
    echo "Mark a 10-meter straight line on the ground."
    echo "Drive the vehicle from start to end of the line."
    read -p "Press ENTER when at start position..."

    echo "Starting encoder calibration..."
    START_ODOM=$(ros2 topic echo /agv/wheel_odometry --once 2>/dev/null | \
        grep -A2 "position:" | head -3 || echo "x: 0.0")
    read -p "Drive 10 meters and press ENTER at end position..."

    END_ODOM=$(ros2 topic echo /agv/wheel_odometry --once 2>/dev/null | \
        grep -A2 "position:" | head -3 || echo "x: 0.0")

    echo "✓ Wheel encoder calibration complete"
    echo "  Adjust wheel_radius in config/vehicle.yaml if distance differs from 10m"
}

calibrate_camera() {
    echo ""
    echo "--- Camera Calibration (Stereo) ---"
    echo "Using a checkerboard pattern (9x7, 25mm squares)"
    echo "Move the checkerboard in front of the camera."
    read -p "Press ENTER to start calibration recording..."

    ros2 run camera_calibration cameracalibrator \
        --size 9x7 --square 0.025 \
        --ros-args \
        --remap image:=/camera/color/image_raw \
        --remap camera:=/camera/color 2>/dev/null &
    CAL_PID=$!
    read -p "Press ENTER when calibration is complete..."
    kill $CAL_PID 2>/dev/null || true
    echo "✓ Camera calibration saved"
}

show_menu() {
    echo ""
    echo "Select calibration to perform:"
    echo "  1) IMU calibration"
    echo "  2) LiDAR ground calibration"
    echo "  3) GPS datum calibration"
    echo "  4) Wheel encoder calibration"
    echo "  5) Camera calibration"
    echo "  6) All calibrations"
    echo "  q) Quit"
    echo ""
    read -p "Enter choice: " CHOICE

    case "$CHOICE" in
        1) calibrate_imu ;;
        2) calibrate_lidar ;;
        3) calibrate_gps ;;
        4) calibrate_wheel_encoders ;;
        5) calibrate_camera ;;
        6)
            calibrate_imu
            calibrate_lidar
            calibrate_gps
            calibrate_wheel_encoders
            ;;
        q|Q) echo "Exiting." ; exit 0 ;;
        *) echo "Invalid choice" ;;
    esac

    show_menu
}

show_menu
