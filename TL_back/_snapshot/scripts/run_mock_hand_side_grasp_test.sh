#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

# Seeded from the user's manually dragged flat-grasp pose:
# TCP ~= [358.35, 39.13, -4.56, -2.043, 1.471, -1.025].
# The offset is an initial estimate from the last clicked target and must be
# refined with the fake hand installed.
export TL_BASE_XY_BIAS_MM="${TL_BASE_XY_BIAS_MM:--53.2 -34.2}"
export TL_GRASP_RPY_RAD="${TL_GRASP_RPY_RAD:--2.043 1.471 -1.025}"
export TL_OBSERVE_JOINT_DEG="${TL_OBSERVE_JOINT_DEG:--1.521 -1.225 35.678 3.856 31.471 -3.533}"
export TL_FINAL_GRASP_Z_MM="${TL_FINAL_GRASP_Z_MM:-69.62}"
export TL_MOCK_GRASP_OFFSET_MM="${TL_MOCK_GRASP_OFFSET_MM:--6.3 34.0 42.5}"
export TL_MOCK_GRASP_CLEARANCE_MM="${TL_MOCK_GRASP_CLEARANCE_MM:-0}"
export TL_SERVOJ_POINTS="${TL_SERVOJ_POINTS:-0}"
export TL_SERVOJ_SPEED_DEG_S="${TL_SERVOJ_SPEED_DEG_S:-20}"
export TL_SERVOJ_PRECISION_SPEED_DEG_S="${TL_SERVOJ_PRECISION_SPEED_DEG_S:-8}"
export TL_MOTION_TIMEOUT="${TL_MOTION_TIMEOUT:-90}"
export TL_MAX_JOINT_DELTA_DEG="${TL_MAX_JOINT_DELTA_DEG:-220}"
export TL_AUTO_LIFT_TO_REACHABLE="${TL_AUTO_LIFT_TO_REACHABLE:-1}"
export TL_MAX_AUTO_LIFT_MM="${TL_MAX_AUTO_LIFT_MM:-200}"
export TL_FINE_Z_STEP_MM="${TL_FINE_Z_STEP_MM:-2}"

echo "Mock-hand side grasp seed:"
echo "  TL_BASE_XY_BIAS_MM=${TL_BASE_XY_BIAS_MM}"
echo "  TL_GRASP_RPY_RAD=${TL_GRASP_RPY_RAD}"
echo "  TL_MOCK_GRASP_OFFSET_MM=${TL_MOCK_GRASP_OFFSET_MM}"
echo "  TL_MOCK_GRASP_CLEARANCE_MM=${TL_MOCK_GRASP_CLEARANCE_MM}"
echo "  TL_MAX_JOINT_DELTA_DEG=${TL_MAX_JOINT_DELTA_DEG}"
echo "  TL_AUTO_LIFT_TO_REACHABLE=${TL_AUTO_LIFT_TO_REACHABLE}"
echo "  TL_MAX_AUTO_LIFT_MM=${TL_MAX_AUTO_LIFT_MM}"
echo "  TL_FINE_Z_STEP_MM=${TL_FINE_Z_STEP_MM}"
echo "  TL_FINAL_GRASP_Z_MM=${TL_FINAL_GRASP_Z_MM}"
echo "Note: TL_FINAL_GRASP_Z_MM default is the tuned real object height (69.62)."

exec bash "$project_dir/scripts/run_no_gripper_hover_test.sh"
