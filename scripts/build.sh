#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
if [[ -z "${ROS_DISTRO:-}" ]]; then
  source /etc/os-release
  case "$VERSION_ID" in
    22.04) distro=humble ;;
    24.04) distro=jazzy ;;
    *) echo "Supported Ubuntu versions: 22.04 and 24.04" >&2; exit 1 ;;
  esac
else
  distro="$ROS_DISTRO"
fi
source "/opt/ros/$distro/setup.bash"
colcon build --base-paths src --symlink-install
