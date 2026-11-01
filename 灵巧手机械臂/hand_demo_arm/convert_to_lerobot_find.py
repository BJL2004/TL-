#!/usr/bin/env python3
"""
TCB605 找寻目标 → LeRobot Dataset 转换（含归一化）
=================================================
把采集的 flat files (rgb/*.jpg + metadata.jsonl + actions.jsonl) 转为
LeRobot v2 Dataset 格式 (parquet)，含 observation.state 和 action 的归一化。

用法:
  python3 convert_to_lerobot_find.py \
    --data-dir ~/tcb_find_data/raw \
    --output-dir ~/tcb_find_data/lerobot_dataset \
    --task "接近红色方块"

依赖: pip3 install pyarrow
"""

import os
import json
import glob
import shutil
import argparse

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq


def load_episode(ep_dir):
    meta_path = os.path.join(ep_dir, 'metadata.jsonl')
    act_path = os.path.join(ep_dir, 'actions.jsonl')
    if not os.path.exists(meta_path):
        return None
    with open(meta_path, 'r', encoding='utf-8') as f:
        records = [json.loads(line) for line in f if line.strip()]
    actions = []
    if os.path.exists(act_path):
        with open(act_path, 'r', encoding='utf-8') as f:
            actions = [json.loads(line) for line in f if line.strip()]
    return records, actions


def main():
    parser = argparse.ArgumentParser(description='TCB605 找寻目标 → LeRobot Dataset')
    parser.add_argument('--data-dir', required=True, help='原始数据根目录 (含 episode_*)')
    parser.add_argument('--output-dir', required=True, help='LeRobot 输出目录')
    parser.add_argument('--task', default='接近红色方块')
    args = parser.parse_args()

    eps = sorted(glob.glob(os.path.join(args.data_dir, 'episode_*')))
    eps = [e for e in eps if os.path.isdir(e)]
    if not eps:
        print(f'未找到 episode: {args.data_dir}')
        return

    print(f'找到 {len(eps)} 条 episode')

    # ===== 第1遍: 收集所有 state/action 计算归一化参数 =====
    all_states = []
    all_actions = []
    ep_data = []
    for ep in eps:
        result = load_episode(ep)
        if result is None:
            continue
        records, actions = result
        if len(records) < 2:
            print(f'[跳过] {os.path.basename(ep)}: 帧数不足')
            continue
        # 补齐 action 长度
        while len(actions) < len(records):
            actions.append({'action': [0.0, 0.0, 0.0]})
        for i, r in enumerate(records):
            tcp = r['tcp_pose']          # [x,y,z,rx,ry,rz]
            joint = r['joint_angle']      # [j1-j6]
            state = tcp + joint           # 12维
            action = actions[i]['action'] if i < len(actions) else [0, 0, 0]
            all_states.append(state)
            all_actions.append(action)
        ep_data.append((ep, records, actions))

    if not ep_data:
        print('无有效 episode')
        return

    all_states = np.array(all_states, dtype=np.float32)
    all_actions = np.array(all_actions, dtype=np.float32)

    state_mean = all_states.mean(axis=0)
    state_std = all_states.std(axis=0) + 1e-6
    action_mean = all_actions.mean(axis=0)
    action_std = all_actions.std(axis=0) + 1e-6

    print(f'state: shape={all_states.shape}')
    print(f'  tcp_xyz mean={state_mean[:3]} std={state_std[:3]}')
    print(f'action: mean={action_mean} std={action_std}')

    # ===== 创建输出目录 =====
    os.makedirs(os.path.join(args.output_dir, 'data', 'chunk-000'), exist_ok=True)
    os.makedirs(os.path.join(args.output_dir, 'meta'), exist_ok=True)
    img_base = os.path.join(args.output_dir, 'images')
    os.makedirs(img_base, exist_ok=True)

    # ===== 第2遍: 写 parquet + 复制图像 =====
    episodes_meta = []
    total_frames = 0
    for ep_idx, (ep, records, actions) in enumerate(ep_data):
        ep_name = os.path.basename(ep)
        rows = []
        for i, r in enumerate(records):
            tcp = r['tcp_pose']
            joint = r['joint_angle']
            state = np.array(tcp + joint, dtype=np.float32)
            state_norm = ((state - state_mean) / state_std).tolist()
            action = np.array(
                actions[i]['action'] if i < len(actions) else [0, 0, 0],
                dtype=np.float32)
            action_norm = ((action - action_mean) / action_std).tolist()
            # 复制图像
            src_img = os.path.join(ep, r['rgb'])
            dst_img_name = f'{ep_name}_frame_{i:06d}.jpg'
            dst_img = os.path.join(img_base, dst_img_name)
            if os.path.exists(src_img):
                shutil.copy(src_img, dst_img)
            rows.append({
                'frame_index': i,
                'episode_index': ep_idx,
                'observation.images.wrist': dst_img_name,
                'observation.state': state_norm,
                'action': action_norm,
                'task': args.task,
                'timestamp': float(i) / 8.0,
            })
        table = pa.Table.from_pylist(rows)
        parquet_path = os.path.join(
            args.output_dir, 'data', 'chunk-000', f'episode_{ep_idx:06d}.parquet')
        pq.write_table(table, parquet_path)
        episodes_meta.append({
            'episode_index': ep_idx, 'task': args.task, 'frames': len(rows)})
        total_frames += len(rows)
        print(f'[OK] {ep_name}: {len(rows)} frames -> {parquet_path}')

    # ===== 写 meta/info.json =====
    info = {
        'codebase_version': 'v2.0',
        'robot_type': 'tcb605_find_target',
        'fps': 8,
        'task': args.task,
        'total_episodes': len(ep_data),
        'total_frames': total_frames,
        'features': {
            'observation.images.wrist': {'dtype': 'image', 'shape': [224, 224, 3]},
            'observation.state': {'dtype': 'float32', 'shape': [12]},
            'action': {'dtype': 'float32', 'shape': [3]},
            'task': {'dtype': 'string'},
            'episode_index': {'dtype': 'int64'},
            'frame_index': {'dtype': 'int64'},
            'timestamp': {'dtype': 'float32'},
        },
        'norm_stats': {
            'observation.state': {
                'mean': state_mean.tolist(), 'std': state_std.tolist()},
            'action': {
                'mean': action_mean.tolist(), 'std': action_std.tolist()},
        },
    }
    with open(os.path.join(args.output_dir, 'meta', 'info.json'), 'w',
              encoding='utf-8') as f:
        json.dump(info, f, ensure_ascii=False, indent=2)

    # ===== 写 meta/episodes.jsonl =====
    with open(os.path.join(args.output_dir, 'meta', 'episodes.jsonl'), 'w',
              encoding='utf-8') as f:
        for em in episodes_meta:
            f.write(json.dumps(em, ensure_ascii=False) + '\n')

    # ===== 写 meta/stats.json (推理服务用) =====
    stats = {
        'observation.state': {
            'mean': state_mean.tolist(), 'std': state_std.tolist()},
        'action': {
            'mean': action_mean.tolist(), 'std': action_std.tolist()},
    }
    with open(os.path.join(args.output_dir, 'meta', 'stats.json'), 'w',
              encoding='utf-8') as f:
        json.dump(stats, f, ensure_ascii=False, indent=2)

    print(f'\n转换完成: {len(ep_data)} episodes, {total_frames} frames')
    print(f'输出: {args.output_dir}')
    print(f'归一化参数: meta/info.json + meta/stats.json')
    print(f'\n注意: LeRobot 版本不同, dataset 结构可能需微调')
    print(f'训练前用 `lerobot-train --help` 确认参数名')


if __name__ == '__main__':
    main()
