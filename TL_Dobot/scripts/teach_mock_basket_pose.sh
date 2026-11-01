#!/usr/bin/env bash
set -eo pipefail

pose_name="${1:-}"
if [[ -z "$pose_name" ]]; then
    echo "Usage: $0 basket_approach|basket_release|observation"
    exit 2
fi

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
export PYTHONPATH="$project_dir/src/tl_hand_deploy:${PYTHONPATH:-}"

python3 -m tl_hand_deploy.mock_place_teach \
    --name "$pose_name" \
    --output "${TL_PLACE_FILE:-$HOME/tl_mock_pick_place/place_pose.yaml}"
