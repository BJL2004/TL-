#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m compileall -q "${PROJECT_DIR}/src/tl_hand_deploy/tl_hand_deploy"
PYTHONPATH="${PROJECT_DIR}/src/tl_hand_deploy:${PYTHONPATH:-}" python3 -m unittest discover -s "${PROJECT_DIR}/src/tl_hand_deploy/test" -v
echo "Static checks passed. ROS imports are checked on Jetson after sourcing the vendor workspace."
