#!/usr/bin/env bash
# build_map.sh — Run SLAM to build and save occupancy grid and 3D map
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
MAPS_DIR="${WORKSPACE_DIR}/maps"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash" 2>/dev/null || {
    echo "ERROR: Workspace not built. Run ./scripts/build.sh first"
    exit 1
}

echo "=============================================="
echo "AGV Map Builder"
echo "=============================================="
echo ""
echo "This tool runs SLAM to build a map of the environment."
echo "Drive the vehicle slowly through the entire tour route."
echo ""

MAP_NAME="${1:-agv_map_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$MAPS_DIR"

echo "Map will be saved as: ${MAPS_DIR}/${MAP_NAME}"
echo ""

# Launch SLAM Toolbox for 2D map building
echo "[1/3] Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="${WORKSPACE_DIR}/config/slam_params.yaml" \
    use_sim_time:=false &
SLAM_PID=$!

# Start NDT mapping for 3D map
echo "[2/3] Starting NDT 3D mapping..."
ros2 run agv_mapping ndt_mapping_node \
    --ros-args -p map_save_path:="${MAPS_DIR}/${MAP_NAME}_3d" &
NDT_PID=$!

echo ""
echo "=============================================="
echo "SLAM is running. Drive the vehicle through"
echo "the entire tour route at low speed (<1 m/s)."
echo ""
echo "Press CTRL+C when mapping is complete."
echo "=============================================="
echo ""

save_map() {
    echo ""
    echo "[3/3] Saving maps..."

    # Save 2D occupancy grid
    ros2 run nav2_map_server map_saver_cli \
        -f "${MAPS_DIR}/${MAP_NAME}" \
        --ros-args -p save_map_timeout:=5.0 2>/dev/null || true

    # Kill mapping nodes
    kill $SLAM_PID 2>/dev/null || true
    kill $NDT_PID 2>/dev/null || true
    wait 2>/dev/null || true

    echo ""
    echo "Maps saved:"
    ls -la "${MAPS_DIR}/${MAP_NAME}"* 2>/dev/null || echo "  (check ${MAPS_DIR}/)"
    echo ""
    echo "To use this map, update agv_params.yaml:"
    echo "  map_server:"
    echo "    map_file: ${MAPS_DIR}/${MAP_NAME}.yaml"
    echo ""
    echo "✓ Map building complete"
    exit 0
}

trap save_map INT TERM

# Wait for SLAM processes
wait $SLAM_PID $NDT_PID 2>/dev/null || save_map
