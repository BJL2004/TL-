#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

export TL_MOTION_BACKEND=movej
export TL_USE_HAND=0
export TL_TARGET_ONLY=1
export TL_USE_FIXED_OBSERVATION=1
export TL_MOVEJ_VELOCITY="${TL_MOVEJ_VELOCITY:-10}"
export TL_MOVEJ_ACCELERATION="${TL_MOVEJ_ACCELERATION:-10}"
export TL_MOVEJ_DECELERATION="${TL_MOVEJ_DECELERATION:-10}"
export TL_MOTION_TIMEOUT="${TL_MOTION_TIMEOUT:-180}"
export TL_MOVEJ_MAX_RETRIES="${TL_MOVEJ_MAX_RETRIES:-2}"
export TL_MOVEJ_RETRY_MAX_ERROR_DEG="${TL_MOVEJ_RETRY_MAX_ERROR_DEG:-10}"
export TL_BASKET_ARUCO_FILE=""

echo "Top-down fake-hand target test using native TL MoveJ."
echo "The arm first moves to the fixed observation pose."
echo "Then: click target | C calculate | M MoveJ to approach and target | R return | Q quit."
echo "Initial MoveJ velocity/acc/dec=${TL_MOVEJ_VELOCITY}/${TL_MOVEJ_ACCELERATION}/${TL_MOVEJ_DECELERATION}."
echo "The program switches controller mode 0 (teach) -> 2 (run) before MoveJ and restores it on exit."
echo "MoveJ is a joint-space path. Keep the fake hand clear and keep the physical E-stop ready."

exec bash "$project_dir/scripts/run_mock_pick_place_basket_test.sh"
