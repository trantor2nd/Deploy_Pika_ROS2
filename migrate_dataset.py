#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
迁移旧格式数据集 → lerobot v3 兼容格式
==============================================
将旧版 data_recorder.py 产生的数据集修复为正确的 lerobot v3 格式：

  旧格式问题：
    1. meta/tasks.jsonl       → 需要 meta/tasks.parquet
    2. meta/episodes.jsonl    → 需要 meta/episodes/chunk-000/file-000.parquet
    3. videos/chunk-{n}/{cam}/episode_{n}.mp4
                              → 需要 videos/{cam}/chunk-{n}/episode_{n}.mp4
    4. info.json 缺少 chunks_size / data_files_size_in_mb / video_files_size_in_mb
       以及路径格式参数名不兼容

用法：
    python3 migrate_dataset.py --root /home/data/Dataset/piper_lerobot_dataset
    python3 migrate_dataset.py --root /path/to/dataset --dry-run   # 仅打印，不修改
"""

import argparse
import json
import shutil
from pathlib import Path

import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq


def migrate(root: Path, dry_run: bool = False) -> None:
    meta = root / "meta"

    # ------------------------------------------------------------------ #
    #  1. tasks.jsonl → tasks.parquet
    # ------------------------------------------------------------------ #
    tasks_jsonl = meta / "tasks.jsonl"
    tasks_pq    = meta / "tasks.parquet"

    tasks_map: dict[str, int] = {}
    if tasks_jsonl.exists():
        with open(tasks_jsonl) as f:
            for line in f:
                line = line.strip()
                if line:
                    obj = json.loads(line)
                    tasks_map[obj["task"]] = obj["task_index"]
        print(f"[tasks] 读取 {len(tasks_map)} 条任务 from {tasks_jsonl}")

        if not dry_run:
            df = pd.DataFrame(
                {"task_index": list(tasks_map.values())},
                index=pd.Index(list(tasks_map.keys()), name="task"),
            )
            df.to_parquet(tasks_pq)
            print(f"[tasks] 写入 {tasks_pq}")
    elif tasks_pq.exists():
        print(f"[tasks] tasks.parquet 已存在，跳过")
        df = pd.read_parquet(tasks_pq)
        for task_text, row in df.iterrows():
            tasks_map[task_text] = int(row["task_index"])
    else:
        print("[tasks] 警告: 未找到 tasks.jsonl 或 tasks.parquet")

    # ------------------------------------------------------------------ #
    #  2. episodes.jsonl → meta/episodes/chunk-000/file-000.parquet
    # ------------------------------------------------------------------ #
    eps_jsonl = meta / "episodes.jsonl"
    eps_dir   = meta / "episodes" / "chunk-000"
    eps_pq    = eps_dir / "file-000.parquet"

    episodes: list[dict] = []
    if eps_jsonl.exists():
        with open(eps_jsonl) as f:
            for line in f:
                line = line.strip()
                if line:
                    episodes.append(json.loads(line))
        print(f"[episodes] 读取 {len(episodes)} 条 episodes from {eps_jsonl}")
    elif eps_pq.exists():
        print(f"[episodes] episodes parquet 已存在，跳过元数据迁移")
    else:
        print("[episodes] 警告: 未找到 episodes.jsonl 或 episodes parquet")

    if episodes and not dry_run:
        eps_dir.mkdir(parents=True, exist_ok=True)

        # 检测活跃相机（从 info.json features）
        info_path = meta / "info.json"
        active_cams: list[str] = []
        if info_path.exists():
            with open(info_path) as f:
                info = json.load(f)
            for key in info.get("features", {}):
                if key.startswith("observation.images."):
                    cam = key[len("observation.images."):]
                    active_cams.append(cam)
        print(f"[episodes] 活跃相机: {active_cams}")

        ep_records = []
        for ep in episodes:
            ep_idx = int(ep["episode_index"])
            chunk  = ep_idx // 1000
            record: dict = {
                "episode_index":    ep_idx,
                "tasks":            list(ep.get("tasks", [])),
                "length":           int(ep.get("length", 0)),
                "data/chunk_index": chunk,
                "data/file_index":  ep_idx,
            }
            for cam in active_cams:
                vid_key = f"observation.images.{cam}"
                record[f"videos/{vid_key}/chunk_index"]    = chunk
                record[f"videos/{vid_key}/file_index"]     = ep_idx
                record[f"videos/{vid_key}/from_timestamp"] = 0.0
            ep_records.append(record)

        schema_fields = [
            pa.field("episode_index",    pa.int64()),
            pa.field("tasks",            pa.list_(pa.string())),
            pa.field("length",           pa.int64()),
            pa.field("data/chunk_index", pa.int64()),
            pa.field("data/file_index",  pa.int64()),
        ]
        for cam in active_cams:
            vid_key = f"observation.images.{cam}"
            schema_fields += [
                pa.field(f"videos/{vid_key}/chunk_index",    pa.int64()),
                pa.field(f"videos/{vid_key}/file_index",     pa.int64()),
                pa.field(f"videos/{vid_key}/from_timestamp", pa.float64()),
            ]
        table = pa.Table.from_pylist(ep_records, schema=pa.schema(schema_fields))
        pq.write_table(table, eps_pq)
        print(f"[episodes] 写入 {eps_pq}")

    # ------------------------------------------------------------------ #
    #  3. 移动视频文件
    #     旧: videos/chunk-{n:03d}/observation.images.{cam}/episode_{ep:06d}.mp4
    #     新: videos/observation.images.{cam}/chunk-{n:03d}/episode_{ep:06d}.mp4
    # ------------------------------------------------------------------ #
    videos_root = root / "videos"
    if videos_root.exists():
        moved = 0
        for old_file in list(videos_root.glob("chunk-*/observation.images.*/*.mp4")):
            # old_file = videos/chunk-000/observation.images.fisheye_rgb/episode_000000.mp4
            parts = old_file.parts
            # 找到 chunk-xxx 和 cam 部分
            rel = old_file.relative_to(videos_root)
            rel_parts = rel.parts  # ('chunk-000', 'observation.images.fisheye_rgb', 'episode_000000.mp4')
            if len(rel_parts) != 3:
                continue
            chunk_part, cam_part, fname = rel_parts
            new_path = videos_root / cam_part / chunk_part / fname
            if not dry_run:
                new_path.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(old_file), str(new_path))
            print(f"[video] {'[DRY]' if dry_run else ''} {old_file.relative_to(root)} → {new_path.relative_to(root)}")
            moved += 1
        if moved == 0:
            print("[video] 未找到需要移动的视频文件（可能已是新格式）")
        else:
            print(f"[video] 共{'模拟' if dry_run else ''}移动 {moved} 个文件")

    # ------------------------------------------------------------------ #
    #  4. 更新 info.json
    # ------------------------------------------------------------------ #
    info_path = meta / "info.json"
    if info_path.exists() and not dry_run:
        with open(info_path) as f:
            info = json.load(f)

        # 修正路径格式参数名
        info["data_path"]  = "data/chunk-{chunk_index:03d}/episode_{file_index:06d}.parquet"
        info["video_path"] = "videos/{video_key}/chunk-{chunk_index:03d}/episode_{file_index:06d}.mp4"

        # 补充缺失字段
        info.setdefault("chunks_size",            1000)
        info.setdefault("data_files_size_in_mb",  100)
        info.setdefault("video_files_size_in_mb", 200)

        # 补充 DEFAULT_FEATURES（timestamp / frame_index 等），lerobot 读取时需要
        default_features = {
            "timestamp":     {"dtype": "float32", "shape": [1], "names": None},
            "frame_index":   {"dtype": "int64",   "shape": [1], "names": None},
            "episode_index": {"dtype": "int64",   "shape": [1], "names": None},
            "index":         {"dtype": "int64",   "shape": [1], "names": None},
            "task_index":    {"dtype": "int64",   "shape": [1], "names": None},
        }
        for k, v in default_features.items():
            info["features"].setdefault(k, v)

        with open(info_path, "w") as f:
            json.dump(info, f, indent=2, ensure_ascii=False)
        print(f"[info] 更新 {info_path}")
    elif dry_run:
        print(f"[info] [DRY] 将更新 {info_path}")

    print("\n✅ 迁移完成" if not dry_run else "\n✅ DRY RUN 完成（未修改任何文件）")
    if not dry_run:
        print("\n可用以下命令验证：")
        print(f"  lerobot-dataset-viz --repo-id local/piper --root {root} --episode-index 0")


def main() -> None:
    parser = argparse.ArgumentParser(description="迁移数据集到 lerobot v3 格式")
    parser.add_argument("--root", required=True, help="数据集根目录")
    parser.add_argument("--dry-run", action="store_true", help="仅打印，不修改文件")
    args = parser.parse_args()
    migrate(Path(args.root), dry_run=args.dry_run)


if __name__ == "__main__":
    main()
