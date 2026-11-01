#!/usr/bin/env python3
"""列出本机所有已示教的盆和点位（种子 / 叉子通用）。

只读两份示教 YAML，不连接机械臂、不发送任何运动。

用法（在 Nano 上）：
    bash ~/Desktop/TL_lowerobs_test/scripts/list_taught.sh               # 全部列出
    bash ~/Desktop/TL_lowerobs_test/scripts/list_taught.sh --full        # 附 TCP 坐标
    bash ~/Desktop/TL_lowerobs_test/scripts/list_taught.sh --basket A2-06  # 只看一盆

贴反排换算：脚本默认带 --reversed-rows A3（TL_REVERSED_ROWS 可覆盖/关闭），
A3 排的"位置列"直接显示可用命令名，--basket 也按换算查询。
"""

import argparse
from pathlib import Path

import yaml

ROW_NAMES = ["A1", "A2", "A3", "A4", "B1", "B2", "B3", "B4"]


def apply_reversed_rows(number, reversed_rows):
    """贴反排换算：位置号 <-> 该格实际贴的码号（对合函数，套两次回原值）。"""
    number = int(number)
    row_index = (number - 1) // 24
    if ROW_NAMES[row_index] not in reversed_rows:
        return number
    index_in_row = (number - 1) % 24
    return row_index * 24 + (24 - index_in_row)


POSE_LABELS = {
    "search_pose": "检索位",
    "basket_approach": "落种接近位(A)",
    "basket_release": "落种释放位(P)",
    "fork_approach": "叉接近位(F)",
    "fork_insert": "水平插入位(I)",
    "fork_lift": "竖直抬起位(L)",
}

GLOBAL_LABELS = {
    "carry_waypoint": "W 高位载运点",
    "fork_transfer_pose": "T 暂放位",
    "carry_waypoint_lower": "W 高位载运点(下排专用)",
    "fork_transfer_pose_lower": "T 暂放位(下排专用)",
    "observation_pose": "O 待机观测位",
    "lower_observation_pose_a": "下排观测位 A",
    "lower_observation_pose_b": "下排观测位 B",
}

SKIP_KEYS = {"number", "aruco_id", "updated_utc"}


def basket_name(number):
    """1..192 -> A2-06 这样的架子编号。"""
    number = int(number) - 1
    return f"{ROW_NAMES[number // 24]}-{number % 24 + 1:02d}"


def parse_basket(text):
    text = str(text).strip().upper()
    if "-" in text:
        row, index = text.split("-", 1)
        if row not in ROW_NAMES or not index.isdigit() or not 1 <= int(index) <= 24:
            raise SystemExit(f"无法识别的盆编号: {text}（示例：A2-06 或 30）")
        return ROW_NAMES.index(row) * 24 + int(index)
    return int(text)


def joints_of(value):
    if not isinstance(value, dict):
        return None
    for key in ("taught_joint", "joint"):
        if isinstance(value.get(key), list) and value[key]:
            return value[key]
    return None


def tcp_of(value):
    if not isinstance(value, dict):
        return None
    for key in ("taught_tcp", "tcp"):
        if isinstance(value.get(key), list) and value[key]:
            return value[key]
    return None


def short(values, digits=1):
    return "[" + ", ".join(f"{float(v):.{digits}f}" for v in values) + "]"


def print_pose(label, value, full):
    joints = joints_of(value)
    text = f"      {label}"
    if joints:
        text += f"  关节={short(joints)}"
    if full:
        tcp = tcp_of(value)
        if tcp:
            text += f"  TCP={short(tcp, 2)}"
    print(text)


def list_file(path, kind, only, full, reversed_rows):
    print(f"\n===== {kind} =====\n文件: {path}")
    if not path.exists():
        print("  （文件不存在）")
        return
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as error:
        print(f"  （YAML 读取失败: {error}）")
        return
    markers = data.get("markers") or {}

    if only is not None:
        effective = apply_reversed_rows(only, reversed_rows)
        if effective != only:
            print(f"（贴反排换算：命令 {basket_name(only)} → 实际使用码号 {effective} / ID{effective - 1}）")
        record = markers.get(str(effective - 1))
        print(f"盆 {only}（{basket_name(only)}）:")
        if not isinstance(record, dict):
            print("  本文件里没有记录（未示教）")
            return
        for key, value in record.items():
            if key in SKIP_KEYS:
                continue
            label = POSE_LABELS.get(key, key)
            if isinstance(value, dict):
                print_pose(label, value, True)
            else:
                print(f"      {label} = {value}")
        return

    numbers = sorted(int(k) for k in markers.keys() if str(k).isdigit())
    if not numbers:
        print("  还没有示教任何盆")
    else:
        print(f"  已示教 {len(numbers)} / 192 盆")
    active_rows = [row for row in reversed_rows if row in ROW_NAMES]
    if active_rows:
        print("  （已启用贴反排换算：" + "/".join(active_rows) + "；位置列直接显示可用命令名）")
    for index in numbers:
        record = markers[str(index)]
        command_number = apply_reversed_rows(index + 1, reversed_rows)
        print(f"  * {index + 1:>3}  {basket_name(command_number)}")
        if not isinstance(record, dict):
            continue
        for key, value in record.items():
            if key in SKIP_KEYS:
                continue
            label = POSE_LABELS.get(key, key)
            if isinstance(value, dict):
                print_pose(label, value, full)
            else:
                print(f"      {label} = {value}")
    for key, value in data.items():
        if key in GLOBAL_LABELS and isinstance(value, dict):
            print_pose(f"[全局] {GLOBAL_LABELS[key]}", value, full)


def main():
    parser = argparse.ArgumentParser(description="列出已示教的盆和点位")
    parser.add_argument(
        "--seed-file",
        type=Path,
        default=Path.home() / "tl_mock_pick_place" / "basket_aruco.yaml",
    )
    parser.add_argument(
        "--fork-file",
        type=Path,
        default=Path.home() / "tl_fork_teach" / "fork_basket_teach.yaml",
    )
    parser.add_argument("--basket", help="只看一盆：1..192 或 A2-06")
    parser.add_argument(
        "--reversed-rows",
        nargs="*",
        default=[],
        metavar="ROW",
        help="贴反排（如 A3）：位置列/查点按实际格子换算",
    )
    parser.add_argument("--full", action="store_true", help="附 TCP 坐标等完整信息")
    args = parser.parse_args()

    only = parse_basket(args.basket) if args.basket else None
    list_file(
        args.seed_file.expanduser(),
        "种子示教（basket_aruco.yaml）",
        only,
        args.full,
        args.reversed_rows,
    )
    list_file(
        args.fork_file.expanduser(),
        "叉子示教（fork_basket_teach.yaml）",
        only,
        args.full,
        args.reversed_rows,
    )


if __name__ == "__main__":
    main()
