#!/bin/bash
set -e
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
export GAZEBO_MODEL_PATH="$WORKSPACE_DIR/install/agv_simulation/share/agv_simulation:$GAZEBO_MODEL_PATH"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
ros2 launch agv_bringup agv_simulation.launch.py "$@"
