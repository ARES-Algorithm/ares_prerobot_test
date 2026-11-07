#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 /usr/bin/colcon build --base-paths src --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
