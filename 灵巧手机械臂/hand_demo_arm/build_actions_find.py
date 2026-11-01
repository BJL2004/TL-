#!/usr/bin/env python3
"""
TCB605 找寻目标 action 生成（位姿差分）
=======================================
读取 episode 的 metadata.jsonl，对相邻帧 tcp_pose 的 xyz 做差分，
生成 action 标签 [dx, dy, dz]，clip 限幅后写入 actions.jsonl。

用法:
  # 单条 episode
  python3 build_actions_find.py --episode-dir ~/tcb_find_data/raw/episode_000001

  # 批量处理所有 episode
  python3 build_actions_find.py --data-dir ~/tcb_find_data/raw --max-step-mm 30
"""

import os
import json
import glob
import argparse
import numpy as np


def build_actions(episode_dir, max_step_mm=30.0):
    meta_path = os.path.join(episode_dir, 'metadata.jsonl')
    if not os.path.exists(meta_path):
        print(f"[跳过] 找不到 {meta_path}")
        return False

    with open(meta_path, 'r', encoding='utf-8') as f:
        records = [json.loads(line) for line in f if line.strip()]

    if len(records) < 2:
        print(f"[跳过] 帧数不足: {len(records)} ({episode_dir})")
        return False

    actions = []
    for i in range(len(records) - 1):
        p1 = records[i]['tcp_pose'][:3]  # [x, y, z] mm
        p2 = records[i + 1]['tcp_pose'][:3]
        dx = float(p2[0] - p1[0])
        dy = float(p2[1] - p1[1])
        dz = float(p2[2] - p1[2])
        # clip 到 ±max_step_mm
        dx = max(-max_step_mm, min(max_step_mm, dx))
        dy = max(-max_step_mm, min(max_step_mm, dy))
        dz = max(-max_step_mm, min(max_step_mm, dz))
        actions.append({
            'frame_index': i,
            'action': [dx, dy, dz],
        })

    # 最后一帧 action = [0, 0, 0]（终止信号）
    actions.append({
        'frame_index': len(records) - 1,
        'action': [0.0, 0.0, 0.0],
    })

    # 写 actions.jsonl
    out_path = os.path.join(episode_dir, 'actions.jsonl')
    with open(out_path, 'w', encoding='utf-8') as f:
        for a in actions:
            f.write(json.dumps(a, ensure_ascii=False) + '\n')

    # 统计
    arr = np.array([a['action'] for a in actions])
    summary = {
        'episode_dir': os.path.basename(episode_dir),
        'total_frames': len(records),
        'total_actions': len(actions),
        'max_step_mm': max_step_mm,
        'dx': {'mean': float(arr[:, 0].mean()), 'std': float(arr[:, 0].std()),
               'min': float(arr[:, 0].min()), 'max': float(arr[:, 0].max())},
        'dy': {'mean': float(arr[:, 1].mean()), 'std': float(arr[:, 1].std()),
               'min': float(arr[:, 1].min()), 'max': float(arr[:, 1].max())},
        'dz': {'mean': float(arr[:, 2].mean()), 'std': float(arr[:, 2].std()),
               'min': float(arr[:, 2].min()), 'max': float(arr[:, 2].max())},
    }
    with open(os.path.join(episode_dir, 'action_summary.json'), 'w',
              encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print(f"[OK] {os.path.basename(episode_dir)}: "
          f"{len(actions)} actions | "
          f"dx={summary['dx']['mean']:.2f}±{summary['dx']['std']:.2f} "
          f"dy={summary['dy']['mean']:.2f}±{summary['dy']['std']:.2f} "
          f"dz={summary['dz']['mean']:.2f}±{summary['dz']['std']:.2f}")
    return True


def main():
    parser = argparse.ArgumentParser(description='TCB605 找寻目标 action 生成')
    parser.add_argument('--episode-dir', help='单条 episode 目录')
    parser.add_argument('--data-dir', help='批量处理: 数据根目录(含 episode_*)')
    parser.add_argument('--max-step-mm', type=float, default=30.0,
                        help='单步最大位移 mm (默认 30)')
    args = parser.parse_args()

    if args.data_dir:
        eps = sorted(glob.glob(os.path.join(args.data_dir, 'episode_*')))
        if not eps:
            print(f"未找到 episode: {args.data_dir}")
            return
        print(f"批量处理 {len(eps)} 条 episode...")
        ok = 0
        for ep in eps:
            if os.path.isdir(ep):
                if build_actions(ep, args.max_step_mm):
                    ok += 1
        print(f"\n完成: {ok}/{len(eps)} 条成功")
    elif args.episode_dir:
        build_actions(args.episode_dir, args.max_step_mm)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
