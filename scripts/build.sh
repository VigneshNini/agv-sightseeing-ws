#!/bin/bash
set -e
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y || true
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) "$@"
echo "Build complete. Source: source $WORKSPACE_DIR/install/setup.bash"
