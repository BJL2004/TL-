#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRIVER_DIR="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"

source /opt/ros/humble/setup.bash
if [[ -f "${DRIVER_DIR}/install/setup.bash" ]]; then
  source "${DRIVER_DIR}/install/setup.bash"
else
  echo "Missing Tianlian driver install/setup.bash: ${DRIVER_DIR}" >&2
  exit 2
fi

cd "${PROJECT_DIR}"
colcon build --symlink-install
echo "Build complete. Run: source ${PROJECT_DIR}/install/setup.bash"
