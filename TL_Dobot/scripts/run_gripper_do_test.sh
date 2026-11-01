#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
export PYTHONPATH="$project_dir/src/tl_hand_deploy:${PYTHONPATH:-}"

source /opt/ros/humble/setup.bash
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
source "$driver_dir/install/setup.bash"

do_a="${TL_GRIPPER_DO_A:-3}"
do_b="${TL_GRIPPER_DO_B:-4}"
initial_high="${TL_GRIPPER_INITIAL_HIGH:-3}"
pulse_sec="${TL_GRIPPER_PULSE_SEC:-0.5}"
output="${TL_GRIPPER_MAP_FILE:-$HOME/tl_gripper_io/gripper_do_map.yaml}"

echo "Gripper digital-output identification test."
echo "Only DO${do_a}/DO${do_b} are tested; no robot motion is sent."
echo "The test keeps one output HIGH and the other LOW."
echo "Keep the gripper empty, reduce pneumatic pressure, and keep E-stop ready."

python3 -m tl_hand_deploy.gripper_do_test \
    --do-a "$do_a" \
    --do-b "$do_b" \
    --initial-high "$initial_high" \
    --pulse-sec "$pulse_sec" \
    --output "$output" \
    --execute
