#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
export TL_BASKET_ARUCO_FILE="${TL_BASKET_ARUCO_FILE:-$HOME/tl_mock_pick_place/basket_aruco.yaml}"
export TL_BASKET_NUMBER="${TL_BASKET_NUMBER:-}"
export TL_USE_HAND="${TL_USE_HAND:-0}"
export TL_BASKET_MOTION_MODE="${TL_BASKET_MOTION_MODE:-translation}"

echo "Dynamic ArUco basket pick-place test."
if [[ -n "$TL_BASKET_NUMBER" ]]; then
    echo "Fixed basket number: ${TL_BASKET_NUMBER}"
else
    echo "Basket selection: autonomous scan across all six taught rack views."
fi
echo "Simple target test: click object, C calculates, M moves, R returns. No basket localization required."
echo "Advanced basket cycle: B only tests localization; G auto-localizes when needed and runs the full cycle."
echo "Keep TL_USE_HAND=0 for the fake-hand stage."
echo "Basket motion mode: ${TL_BASKET_MOTION_MODE} (recommended for rack translation)."

exec bash "$project_dir/scripts/run_mock_pick_place_basket_test.sh"
