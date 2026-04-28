#!/bin/bash
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
[ -f "$WORKSPACE_DIR/install/setup.bash" ] && source "$WORKSPACE_DIR/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export AGV_WORKSPACE="$WORKSPACE_DIR"
export AGV_CONFIG="$WORKSPACE_DIR/config"
export AGV_MAPS="$WORKSPACE_DIR/maps"
echo "AGV env ready. ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
