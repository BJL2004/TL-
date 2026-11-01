#!/usr/bin/env python3
"""
ARM版 眼在手上标定求解
用法: python3 compute_in_hand_arm.py <数据目录>
示例: python3 compute_in_hand_arm.py ~/eye_hand_data/20260721_143000
"""

import sys, os, cv2, yaml, numpy as np
from scipy.spatial.transform import Rotation as R

np.set_printoptions(precision=8, suppress=True)


def euler_to_rot(rx, ry, rz):
    """intrinsic XYZ (Rx@Ry@Rz), 匹配ROS2 /tcp_pose发布格式"""
    return R.from_euler('XYZ', [rx, ry, rz]).as_matrix()


def pose_to_htm(pose):
    """将 [x,y,z,rx,ry,rz] 转为 4x4 齐次变换矩阵"""
    R_mat = euler_to_rot(pose[3], pose[4], pose[5])
    H = np.eye(4)
    H[:3,:3] = R_mat
    H[:3,3] = pose[:3]
    return H


# ===== 读取参数 =====
data_path = sys.argv[1] if len(sys.argv) > 1 else None
if data_path is None:
    # 自动找最新
    base = os.path.expanduser('~/eye_hand_data')
    dirs = sorted([d for d in os.listdir(base) if os.path.isdir(os.path.join(base,d))])
    if not dirs:
        print("未找到数据目录, 用法: python3 compute_in_hand_arm.py <目录>")
        sys.exit(1)
    data_path = os.path.join(base, dirs[-1])

print(f"数据目录: {data_path}")

# 标定板参数 (config.yaml)
config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config.yaml')
if os.path.exists(config_path):
    with open(config_path) as f:
        cfg = yaml.safe_load(f)['checkerboard_args']
    XX, YY, L = cfg['XX'], cfg['YY'], cfg['L']
else:
    XX, YY, L = 9, 6, 0.025  # 默认 9x6 格子 25mm

print(f"棋盘格: {XX}x{YY}, 格子{L*1000:.0f}mm")

# ===== 读数据 =====
objp = np.zeros((XX*YY, 3), np.float32)
objp[:,:2] = np.mgrid[0:XX,0:YY].T.reshape(-1,2) * L

obj_points, img_points = [], []
criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

# 读位姿
pose_file = os.path.join(data_path, 'poses.txt')
if not os.path.exists(pose_file):
    print(f"未找到 {pose_file}")
    sys.exit(1)

with open(pose_file) as f:
    poses_raw = [[float(x) for x in line.strip().split(',')] for line in f if line.strip()]

R_tool, t_tool = [], []
valid_indices = []

for i, pose in enumerate(poses_raw):
    img_path = os.path.join(data_path, f'{i+1}.jpg')
    if not os.path.exists(img_path):
        print(f"跳过: {img_path} 不存在")
        continue

    img = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)

    if not ret:
        print(f"[{i+1}] 棋盘格未检测到, 跳过")
        continue

    obj_points.append(objp)
    corners2 = cv2.cornerSubPix(gray, corners, (5,5), (-1,-1), criteria)
    img_points.append(corners2 if corners2 is not None else corners)

    # 位姿 → 旋转矩阵+平移
    H = pose_to_htm(pose)
    R_tool.append(H[:3,:3])
    t_tool.append(H[:3,3])
    valid_indices.append(i+1)
    print(f"[{i+1}] 棋盘格 OK")

if len(obj_points) < 5:
    print(f"最少需要5组有效数据, 当前{len(obj_points)}")
    sys.exit(1)

print(f"有效: {len(obj_points)}/{len(poses_raw)} 组")

# ===== 相机标定 =====
size = gray.shape[::-1]
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)
print(f"相机标定 RMS: {ret:.3f}")

# ===== 手眼标定 =====
R_cam2end, t_cam2end = cv2.calibrateHandEye(R_tool, t_tool, rvecs, tvecs, cv2.CALIB_HAND_EYE_TSAI)

print("\n" + "=" * 50)
print("  手眼标定结果 (T_tool_camera)")

T = np.eye(4)
T[:3,:3] = R_cam2end
T[:3,3] = t_cam2end.flatten()
print(T)

# 展开为16个值 (row-major)
flat = T.flatten()
print(f"\nhandeye_matrix 参数 (填入 control_node.yaml):")
print(f"  [{flat[0]:.10f}, {flat[1]:.10f}, {flat[2]:.10f}, {flat[3]:.10f},")
print(f"   {flat[4]:.10f}, {flat[5]:.10f}, {flat[6]:.10f}, {flat[7]:.10f},")
print(f"   {flat[8]:.10f}, {flat[9]:.10f}, {flat[10]:.10f}, {flat[11]:.10f},")
print(f"   {flat[12]:.1f}, {flat[13]:.1f}, {flat[14]:.1f}, {flat[15]:.1f}]")
