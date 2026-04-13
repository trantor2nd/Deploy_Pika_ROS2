#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Process C: 数据采集器（直写 LeRobot v3 格式）
============================================================
职责:
  - 订阅 /record_cmd，响应开始/停止/退出指令
  - 直接访问鱼眼相机与 RealSense 相机，以 MP4 视频形式存储
  - 以 Parquet 文件存储关节状态与动作
  - 所有数据直接写入 LeRobot v3 格式，**无需后处理转换**

LeRobot v3 目录结构:
  <save_root>/
    meta/
      info.json         数据集元信息
      episodes.jsonl    每条 episode 的元信息
      tasks.jsonl       任务索引表
    data/
      chunk-000/
        episode_000000.parquet
        ...
    videos/
      chunk-000/
        observation.images.fisheye_rgb/
          episode_000000.mp4
        observation.images.realsense_rgb/
          episode_000000.mp4

使用方式:
  python3 data_recorder.py --save-root /path/to/dataset --task "任务指令"

运行环境: py310 conda + ROS2 Humble
"""

import argparse
import json
import logging
import os
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import pandas as pd

# pyarrow 用于写 Parquet
import pyarrow as pa
import pyarrow.parquet as pq

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import (
    FISHEYE_INDEX, FISHEYE_WIDTH, FISHEYE_HEIGHT, FISHEYE_FPS,
    REALSENSE_SN, REALSENSE_WIDTH, REALSENSE_HEIGHT, REALSENSE_FPS,
    CAPTURE_HZ, DEFAULT_SAVE_ROOT, DEFAULT_TASK, HOME_POS,
    TOPIC_JOINT_STATES, TOPIC_JOINT_CMD,
    TOPIC_GRIPPER_STATE, TOPIC_GRIPPER_CMD,
    TOPIC_RECORD_CMD,
    TELEOP_TOPIC_ACTION, TELEOP_FISHEYE_TOPIC, TELEOP_REALSENSE_TOPIC,
    EPISODES_PER_CHUNK,
)

logger = logging.getLogger("data_recorder")


# ====================================================================== #
#  命令行参数
# ====================================================================== #

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Piper 数据采集器 – 直写 LeRobot v3 格式"
    )
    parser.add_argument(
        "--save-root", type=str, default=DEFAULT_SAVE_ROOT,
        help=f"数据集保存根目录（默认: {DEFAULT_SAVE_ROOT}）",
    )
    parser.add_argument(
        "--task", type=str, default=DEFAULT_TASK,
        help=f"任务指令文本（默认: {DEFAULT_TASK}）",
    )
    parser.add_argument(
        "--fps", type=float, default=CAPTURE_HZ,
        help=f"采集帧率 Hz（默认: {CAPTURE_HZ}）",
    )
    parser.add_argument(
        "--mode", type=str, choices=["keyboard", "teleop"], default="keyboard",
        help="采集模式: keyboard=键盘直控（默认）, teleop=遥操作",
    )
    return parser.parse_args()


def _ros_image_to_bgr(msg: Image) -> np.ndarray:
    """Convert sensor_msgs/Image to BGR numpy array without cv_bridge."""
    if msg.encoding == "bgr8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        ).copy()
    elif msg.encoding == "rgb8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )[:, :, ::-1].copy()
    elif msg.encoding == "bgra8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 4
        )[:, :, :3].copy()
    elif msg.encoding == "rgba8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 4
        )[:, :, 2::-1].copy()
    else:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")


# ====================================================================== #
#  LeRobot v3 数据集写入器
# ====================================================================== #

class LeRobotDatasetWriter:
    """
    高效直写 LeRobot v3 格式数据集。

    在录制过程中：
      - 用 cv2.VideoWriter 实时编码 MP4 视频（零后处理）
      - 将 state/action 缓存在内存列表中

    每条 episode 结束时：
      - 释放视频文件句柄，完成 MP4 写入
      - 将内存中的状态数据一次性写为 Parquet 文件
      - 更新 meta/*.jsonl / info.json
    """

    # 可能可用的相机列表；运行时动态确认
    CAMERA_SHAPES = {
        "fisheye_rgb":   (FISHEYE_HEIGHT,   FISHEYE_WIDTH,   3),
        "realsense_rgb": (REALSENSE_HEIGHT,  REALSENSE_WIDTH, 3),
    }
    STATE_DIM   = 7  # 6 关节 + 1 夹爪
    STATE_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6", "gripper"]

    def __init__(self, root: str, task: str, fps: float,
                 active_cameras: list[str]) -> None:
        self.root           = Path(root)
        self.default_task   = task
        self.fps            = fps
        self.active_cameras = active_cameras   # 实际可用的相机名列表

        self._ensure_dirs()
        self._load_meta()

    # ------------------------------------------------------------------ #

    def _ensure_dirs(self) -> None:
        (self.root / "meta").mkdir(parents=True, exist_ok=True)

    def _chunk_dir(self, episode_index: int) -> int:
        return episode_index // EPISODES_PER_CHUNK

    def _data_path(self, episode_index: int) -> Path:
        chunk = self._chunk_dir(episode_index)
        path  = self.root / "data" / f"chunk-{chunk:03d}"
        path.mkdir(parents=True, exist_ok=True)
        return path / f"episode_{episode_index:06d}.parquet"

    def _video_path(self, episode_index: int, cam: str) -> Path:
        chunk = self._chunk_dir(episode_index)
        # lerobot v3 期望: videos/{video_key}/chunk-{n:03d}/episode_{ep:06d}.mp4
        path = self.root / "videos" / f"observation.images.{cam}" / f"chunk-{chunk:03d}"
        path.mkdir(parents=True, exist_ok=True)
        return path / f"episode_{episode_index:06d}.mp4"

    # ------------------------------------------------------------------ #

    def _load_meta(self) -> None:
        """读取已有的 episodes / tasks，以便追加录制。
        优先读取 lerobot v3 Parquet 格式，回退到旧 JSONL 格式。
        """
        eps_dir      = self.root / "meta" / "episodes"
        tasks_pq     = self.root / "meta" / "tasks.parquet"
        eps_jsonl    = self.root / "meta" / "episodes.jsonl"
        tasks_jsonl  = self.root / "meta" / "tasks.jsonl"

        # --- Episodes ---
        self.episodes: list[dict] = []
        if eps_dir.exists():
            for pq_file in sorted(eps_dir.glob("*/*.parquet")):
                df = pd.read_parquet(pq_file)
                for _, row in df.iterrows():
                    ep = row.to_dict()
                    if "tasks" in ep and not isinstance(ep["tasks"], list):
                        ep["tasks"] = list(ep["tasks"])
                    self.episodes.append(ep)
        elif eps_jsonl.exists():
            with open(eps_jsonl) as f:
                for line in f:
                    line = line.strip()
                    if line:
                        self.episodes.append(json.loads(line))

        # --- Tasks ---
        self.tasks_map: dict[str, int] = {}  # task_text -> task_index
        if tasks_pq.exists():
            df = pd.read_parquet(tasks_pq)
            for task_text, row in df.iterrows():
                self.tasks_map[task_text] = int(row["task_index"])
        elif tasks_jsonl.exists():
            with open(tasks_jsonl) as f:
                for line in f:
                    line = line.strip()
                    if line:
                        obj = json.loads(line)
                        self.tasks_map[obj["task"]] = obj["task_index"]

        self.total_frames       = sum(int(ep.get("length", 0)) for ep in self.episodes)
        self.next_episode_index = len(self.episodes)

        logger.info(
            "数据集加载完成: %d episodes, %d frames",
            self.next_episode_index, self.total_frames,
        )

    # ------------------------------------------------------------------ #

    def _get_task_index(self, task_text: str) -> int:
        if task_text not in self.tasks_map:
            self.tasks_map[task_text] = len(self.tasks_map)
        return self.tasks_map[task_text]

    # ------------------------------------------------------------------ #

    def create_episode(self, task_text: Optional[str] = None) -> "EpisodeWriter":
        return EpisodeWriter(self, task_text or self.default_task)

    # ------------------------------------------------------------------ #

    def finalize_episode(self, ep: "EpisodeWriter") -> None:
        """由 EpisodeWriter 在 close() 时调用。"""
        ep_idx    = ep.episode_index
        n_frames  = ep.frame_count
        task_text = ep.task_text
        task_idx  = self._get_task_index(task_text)

        if n_frames == 0:
            logger.warning("Episode %d 无有效帧，跳过保存", ep_idx)
            return

        # ---------- 写 Parquet ----------
        self._write_parquet(ep, ep_idx, task_idx)

        # ---------- 更新 meta ----------
        self.episodes.append({
            "episode_index": ep_idx,
            "tasks":  [task_text],
            "length": n_frames,
        })
        self.total_frames       += n_frames
        self.next_episode_index += 1
        self._write_meta()

        logger.info(
            "Episode %d 已保存: %d 帧, 任务: %s", ep_idx, n_frames, task_text
        )

    # ------------------------------------------------------------------ #

    def _write_parquet(self, ep: "EpisodeWriter", ep_idx: int,
                       task_idx: int) -> None:
        n      = ep.frame_count
        states  = np.stack(ep.states,  axis=0)   # (N, 7): 实际关节+夹爪
        actions = np.stack(ep.actions, axis=0)   # (N, 7): 指令关节+夹爪
        timestamps = ep.timestamps

        schema = pa.schema([
            pa.field("observation.state", pa.list_(pa.float32())),
            pa.field("action",            pa.list_(pa.float32())),
            pa.field("timestamp",         pa.float32()),
            pa.field("frame_index",       pa.int64()),
            pa.field("episode_index",     pa.int64()),
            pa.field("index",             pa.int64()),
            pa.field("task_index",        pa.int64()),
        ])

        table = pa.table(
            {
                "observation.state": [states[i].tolist()  for i in range(n)],
                "action":            [actions[i].tolist() for i in range(n)],
                "timestamp":         [float(t) for t in timestamps],
                "frame_index":       list(range(n)),
                "episode_index":     [ep_idx] * n,
                "index":             list(range(self.total_frames, self.total_frames + n)),
                "task_index":        [task_idx] * n,
            },
            schema=schema,
        )
        pq.write_table(table, self._data_path(ep_idx))

    # ------------------------------------------------------------------ #

    def _write_meta(self) -> None:
        # --- meta/tasks.parquet (lerobot v3 格式) ---
        tasks_df = pd.DataFrame(
            {"task_index": list(self.tasks_map.values())},
            index=pd.Index(list(self.tasks_map.keys()), name="task"),
        )
        tasks_df.to_parquet(self.root / "meta" / "tasks.parquet")

        # --- meta/episodes/chunk-000/file-000.parquet (lerobot v3 格式) ---
        if self.episodes:
            eps_dir = self.root / "meta" / "episodes" / "chunk-000"
            eps_dir.mkdir(parents=True, exist_ok=True)

            ep_records = []
            for ep in self.episodes:
                ep_idx = int(ep["episode_index"])
                chunk  = self._chunk_dir(ep_idx)
                record: dict = {
                    "episode_index":    ep_idx,
                    "tasks":            list(ep.get("tasks", [])),
                    "length":           int(ep.get("length", 0)),
                    "data/chunk_index": chunk,
                    "data/file_index":  ep_idx,
                }
                for cam in self.active_cameras:
                    vid_key = f"observation.images.{cam}"
                    record[f"videos/{vid_key}/chunk_index"]    = chunk
                    record[f"videos/{vid_key}/file_index"]     = ep_idx
                    # 每个 episode 独立 MP4，起始时间戳始终为 0
                    record[f"videos/{vid_key}/from_timestamp"] = 0.0
                ep_records.append(record)

            # 构建 PyArrow 表（明确指定 schema 确保类型正确）
            schema_fields = [
                pa.field("episode_index",    pa.int64()),
                pa.field("tasks",            pa.list_(pa.string())),
                pa.field("length",           pa.int64()),
                pa.field("data/chunk_index", pa.int64()),
                pa.field("data/file_index",  pa.int64()),
            ]
            for cam in self.active_cameras:
                vid_key = f"observation.images.{cam}"
                schema_fields += [
                    pa.field(f"videos/{vid_key}/chunk_index",    pa.int64()),
                    pa.field(f"videos/{vid_key}/file_index",     pa.int64()),
                    pa.field(f"videos/{vid_key}/from_timestamp", pa.float64()),
                ]
            table = pa.Table.from_pylist(ep_records, schema=pa.schema(schema_fields))
            pq.write_table(table, eps_dir / "file-000.parquet")

        # --- meta/info.json ---
        # DEFAULT_FEATURES 必须写入 features，否则 lerobot 的 get_hf_features_from_features
        # 无法在 Dataset.from_parquet 时匹配 parquet 中的 timestamp/frame_index 等列
        features: dict = {
            "observation.state": {
                "dtype": "float32",
                "shape": [self.STATE_DIM],
                "names": self.STATE_NAMES,
            },
            "action": {
                "dtype": "float32",
                "shape": [self.STATE_DIM],
                "names": self.STATE_NAMES,
            },
            "timestamp":     {"dtype": "float32", "shape": [1], "names": None},
            "frame_index":   {"dtype": "int64",   "shape": [1], "names": None},
            "episode_index": {"dtype": "int64",   "shape": [1], "names": None},
            "index":         {"dtype": "int64",   "shape": [1], "names": None},
            "task_index":    {"dtype": "int64",   "shape": [1], "names": None},
        }
        for cam in self.active_cameras:
            h, w, _ = self.CAMERA_SHAPES[cam]
            features[f"observation.images.{cam}"] = {
                "dtype": "video",
                "shape": [h, w, 3],
                "names": ["height", "width", "channels"],
                "video_info": {
                    "video.fps":          self.fps,
                    "video.codec":        "mp4v",
                    "video.pix_fmt":      "yuv420p",
                    "video.is_depth_map": False,
                },
            }

        info = {
            "codebase_version":      "v3.0",
            "robot_type":            "piper",
            "fps":                   self.fps,
            "features":              features,
            "total_episodes":        len(self.episodes),
            "total_frames":          self.total_frames,
            "total_tasks":           len(self.tasks_map),
            "chunks_size":           EPISODES_PER_CHUNK,
            "data_files_size_in_mb": 100,
            "video_files_size_in_mb": 200,
            "splits":                {"train": f"0:{len(self.episodes)}"},
            # 使用 chunk_index / file_index 作为格式参数（与 lerobot v3 兼容）
            "data_path":  "data/chunk-{chunk_index:03d}/episode_{file_index:06d}.parquet",
            "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/episode_{file_index:06d}.mp4",
        }
        with open(self.root / "meta" / "info.json", "w") as f:
            json.dump(info, f, indent=2, ensure_ascii=False)


# ====================================================================== #
#  单条 Episode 写入器
# ====================================================================== #

class EpisodeWriter:
    """管理单条 episode 的内存缓冲与视频流写入。"""

    def __init__(self, parent: LeRobotDatasetWriter, task_text: str) -> None:
        self.parent        = parent
        self.task_text     = task_text
        self.episode_index = parent.next_episode_index
        self.frame_count   = 0
        self.start_time    = time.time()

        # 状态缓冲
        self.states:     list[np.ndarray] = []   # observation.state（实际位置）
        self.actions:    list[np.ndarray] = []   # action（键盘指令位置）
        self.timestamps: list[float]      = []

        # 视频写入器：每个活跃相机一个
        self._writers: dict[str, cv2.VideoWriter] = {}
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        for cam in parent.active_cameras:
            h, w, _ = parent.CAMERA_SHAPES[cam]
            path    = str(parent._video_path(self.episode_index, cam))
            writer  = cv2.VideoWriter(path, fourcc, parent.fps, (w, h))
            if not writer.isOpened():
                logger.error("无法创建视频文件: %s", path)
            self._writers[cam] = writer

        logger.info(
            "Episode %d 开始录制, task: %s",
            self.episode_index, task_text,
        )

    # ------------------------------------------------------------------ #

    def add_frame(
        self,
        joints:          list[float],
        gripper:         float,
        joint_cmd:       list[float],
        gripper_cmd:     float,
        fisheye_frame:   Optional[np.ndarray] = None,
        realsense_frame: Optional[np.ndarray] = None,
    ) -> None:
        """写入一帧数据。
        joints/gripper   → observation.state（来自 piper_single_ctrl 的实际反馈）
        joint_cmd/gripper_cmd → action（来自键盘控制器的指令值）
        """
        # 使用帧对齐时间戳（frame_index / fps），与 cv2.VideoWriter 编码帧严格对齐
        # 避免 lerobot 读取时因 wall-clock 偏差超出 tolerance_s 导致解码失败
        t = self.frame_count / self.parent.fps
        state  = np.array(list(joints[:6])    + [gripper],     dtype=np.float32)
        action = np.array(list(joint_cmd[:6]) + [gripper_cmd], dtype=np.float32)
        self.states.append(state)
        self.actions.append(action)
        self.timestamps.append(t)

        frames_map = {
            "fisheye_rgb":   fisheye_frame,
            "realsense_rgb": realsense_frame,
        }
        for cam, writer in self._writers.items():
            frame = frames_map.get(cam)
            if frame is not None and writer.isOpened():
                writer.write(frame)

        self.frame_count += 1

    # ------------------------------------------------------------------ #

    def close(self) -> None:
        """释放视频写入器，触发 Parquet 保存与 meta 更新。"""
        for writer in self._writers.values():
            writer.release()
        self.parent.finalize_episode(self)


# ====================================================================== #
#  ROS2 数据采集节点
# ====================================================================== #

class DataRecorder(Node):

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("data_recorder")

        self.fps  = args.fps
        self.task = args.task
        self.teleop_mode = (args.mode == "teleop")

        # ---- 相机初始化（模式相关）----
        self.fisheye   = None
        self.realsense = None
        # 遥操作模式：通过 ROS2 Image 话题获取图像
        self._teleop_fisheye_frame:   Optional[np.ndarray] = None
        self._teleop_realsense_frame: Optional[np.ndarray] = None

        if self.teleop_mode:
            active_cameras = self._init_teleop_cameras()
        else:
            logging.getLogger("pika.serial_comm").setLevel(logging.ERROR)
            logging.getLogger("pika.camera.realsense").setLevel(logging.ERROR)
            self._connect_cameras()
            active_cameras = []
            if self.fisheye:
                active_cameras.append("fisheye_rgb")
            if self.realsense:
                active_cameras.append("realsense_rgb")

        if not active_cameras:
            self.get_logger().warning("未连接任何相机，将只记录关节/夹爪状态")

        # LeRobot 写入器
        self.writer = LeRobotDatasetWriter(
            root=args.save_root,
            task=args.task,
            fps=args.fps,
            active_cameras=active_cameras,
        )

        # ---- ROS2 订阅（共用）----
        self.create_subscription(
            JointState, TOPIC_JOINT_STATES, self._joint_cb, 10
        )
        self.create_subscription(
            String, TOPIC_RECORD_CMD, self._cmd_cb, 10
        )

        # ---- ROS2 订阅（模式相关）----
        if self.teleop_mode:
            # 遥操作：action 来自 /joint_states_gripper (arm+gripper 指令)
            self.create_subscription(
                JointState, TELEOP_TOPIC_ACTION, self._teleop_action_cb, 10
            )
        else:
            # 键盘：action 来自 Process B 的 /arm/joint_cmd + /arm/gripper_cmd
            self.create_subscription(
                JointState, TOPIC_JOINT_CMD, self._joint_cmd_cb, 10
            )
            self.create_subscription(
                Float64, TOPIC_GRIPPER_STATE, self._gripper_cb, 10
            )
            self.create_subscription(
                Float64, TOPIC_GRIPPER_CMD, self._gripper_cmd_cb, 10
            )

        # 本地状态缓存
        # joint_state:     实际关节位置（来自 piper_single_ctrl → /joint_states_single）
        # joint_cmd:       指令位置 — 键盘模式来自 /arm/joint_cmd，遥操作来自 /joint_states_gripper
        self.joint_state:    list[float] = list(HOME_POS)
        self.joint_cmd:      list[float] = list(HOME_POS)
        self.gripper_actual: float       = 0.0
        self.gripper_cmd:    float       = 0.0
        # 标记是否收到过真实的关节状态反馈（CAN → piper_single_ctrl → /joint_states_single）
        # 若从未收到，采集时用 joint_cmd 代替 joint_state（指令位置近似实际位置）
        self._joint_state_received: bool = False
        self.recording:      bool        = False
        self.running:        bool        = True
        self.current_episode: Optional[EpisodeWriter] = None

        # 待完成的写入线程列表（非 daemon，确保进程退出前完成写入）
        self._write_threads: list[threading.Thread] = []

        # 采集线程
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True
        )
        self._capture_thread.start()

        mode_str = "遥操作" if self.teleop_mode else "键盘"
        self.get_logger().info(
            f"DataRecorder 节点启动 ✅  mode={mode_str}  save_root={args.save_root}  task={args.task}"
        )

    # ------------------------------------------------------------------ #
    #  相机连接
    # ------------------------------------------------------------------ #

    def _connect_cameras(self) -> None:
        """键盘模式: 通过 Pika SDK 直接打开相机。"""
        from pika.camera.fisheye import FisheyeCamera
        from pika.camera.realsense import RealSenseCamera

        # 鱼眼相机
        try:
            cam = FisheyeCamera(
                camera_width=FISHEYE_WIDTH,
                camera_height=FISHEYE_HEIGHT,
                camera_fps=FISHEYE_FPS,
                device_id=FISHEYE_INDEX,
                fisheye_thread_fps=60,
            )
            if cam.connect():
                self.fisheye = cam
                self.get_logger().info("鱼眼相机已连接 ✅")
            else:
                self.get_logger().warning("鱼眼相机连接失败")
        except Exception as exc:
            self.get_logger().warning(f"鱼眼相机异常: {exc}")

        # RealSense 相机
        try:
            cam = RealSenseCamera(
                camera_width=REALSENSE_WIDTH,
                camera_height=REALSENSE_HEIGHT,
                camera_fps=REALSENSE_FPS,
                serial_number=REALSENSE_SN,
            )
            if cam.connect():
                self.realsense = cam
                self.get_logger().info("RealSense 相机已连接 ✅")
            else:
                # 尝试自动匹配
                cam2 = RealSenseCamera(
                    camera_width=REALSENSE_WIDTH,
                    camera_height=REALSENSE_HEIGHT,
                    camera_fps=REALSENSE_FPS,
                    serial_number=None,
                )
                if cam2.connect():
                    self.realsense = cam2
                    self.get_logger().info("RealSense 相机（自动匹配）已连接 ✅")
                else:
                    self.get_logger().warning("RealSense 相机连接失败")
        except Exception as exc:
            self.get_logger().warning(f"RealSense 相机异常: {exc}")

    def _init_teleop_cameras(self) -> list[str]:
        """遥操作模式：订阅 ROS2 Image 话题获取相机图像。"""
        active_cameras = []
        self.create_subscription(
            Image, TELEOP_FISHEYE_TOPIC, self._teleop_fisheye_cb, 10
        )
        active_cameras.append("fisheye_rgb")
        self.get_logger().info(
            f"遥操作模式: 订阅鱼眼话题 {TELEOP_FISHEYE_TOPIC}"
        )
        self.create_subscription(
            Image, TELEOP_REALSENSE_TOPIC, self._teleop_realsense_cb, 10
        )
        active_cameras.append("realsense_rgb")
        self.get_logger().info(
            f"遥操作模式: 订阅 RealSense 话题 {TELEOP_REALSENSE_TOPIC}"
        )
        return active_cameras

    def _teleop_fisheye_cb(self, msg: Image) -> None:
        try:
            self._teleop_fisheye_frame = _ros_image_to_bgr(msg)
        except Exception as exc:
            logger.warning("鱼眼图像转换失败: %s", exc)

    def _teleop_realsense_cb(self, msg: Image) -> None:
        try:
            self._teleop_realsense_frame = _ros_image_to_bgr(msg)
        except Exception as exc:
            logger.warning("RealSense 图像转换失败: %s", exc)

    def _disconnect_cameras(self) -> None:
        if self.fisheye:
            try:
                self.fisheye.disconnect()
            except Exception:
                pass
        if self.realsense:
            try:
                self.realsense.disconnect()
            except Exception:
                pass

    # ------------------------------------------------------------------ #
    #  ROS2 回调
    # ------------------------------------------------------------------ #

    def _joint_cb(self, msg: JointState) -> None:
        """实际关节位置反馈（来自 piper_single_ctrl）→ observation.state"""
        if len(msg.position) >= 6:
            self.joint_state = list(msg.position[:6])
            self._joint_state_received = True
            # 遥操作模式：夹爪状态也从 /joint_states_single position[6] 获取（CAN 反馈）
            if self.teleop_mode and len(msg.position) >= 7:
                self.gripper_actual = float(msg.position[6])

    def _teleop_action_cb(self, msg: JointState) -> None:
        """遥操作模式: 来自 /joint_states_gripper 的 arm+gripper 指令 → action"""
        if len(msg.position) >= 6:
            self.joint_cmd = list(msg.position[:6])
        if len(msg.position) >= 7:
            self.gripper_cmd = float(msg.position[6])

    def _joint_cmd_cb(self, msg: JointState) -> None:
        """键盘发出的关节指令（来自 Process B）→ action"""
        if len(msg.position) >= 6:
            self.joint_cmd = list(msg.position[:6])

    def _gripper_cb(self, msg: Float64) -> None:
        """实际夹爪位置反馈 → observation.state[6]"""
        self.gripper_actual = msg.data

    def _gripper_cmd_cb(self, msg: Float64) -> None:
        """键盘发出的夹爪指令 → action[6]"""
        self.gripper_cmd = msg.data

    def _cmd_cb(self, msg: String) -> None:
        cmd = msg.data.lower().strip()
        if cmd == "start" and not self.recording:
            self._start_recording()
        elif cmd == "stop" and self.recording:
            self._stop_recording()
        elif cmd == "discard":
            self._discard_recording()
        elif cmd == "exit":
            if self.recording:
                self._stop_recording()
            self.running = False

    # ------------------------------------------------------------------ #
    #  录制控制
    # ------------------------------------------------------------------ #

    def _start_recording(self) -> None:
        if not self._joint_state_received:
            self.get_logger().warning(
                "⚠️  未收到 /joint_states_single（piper_single_ctrl 未运行？），"
                "observation.state 将使用指令位置代替实际位置"
            )
        self.current_episode = self.writer.create_episode(self.task)
        self.recording       = True

    def _stop_recording(self) -> None:
        self.recording = False
        ep = self.current_episode
        self.current_episode = None
        if ep:
            # 非 daemon 线程：确保进程退出前完成 Parquet/MP4 写入
            t = threading.Thread(target=ep.close, daemon=False)
            t.start()
            self._write_threads.append(t)

    def _discard_recording(self) -> None:
        """丢弃 episode：正在录制则丢弃当前，否则丢弃上一条已完成的。"""
        if self.recording:
            # ---- 正在录制，丢弃当前 episode ----
            self.recording = False
            ep = self.current_episode
            self.current_episode = None
            if ep is None:
                return
            ep_idx = ep.episode_index
            # 释放视频写入器（关闭文件句柄，不调用 finalize_episode）
            for writer in ep._writers.values():
                writer.release()
            # 删除已创建的视频文件
            for cam in self.writer.active_cameras:
                vpath = self.writer._video_path(ep_idx, cam)
                if vpath.exists():
                    vpath.unlink()
                    logger.info("已删除视频: %s", vpath)
            self.get_logger().info(
                f"Episode {ep_idx} 已丢弃（录制中）"
            )
        else:
            # ---- 未在录制，丢弃上一条已完成的 episode ----
            if not self.writer.episodes:
                self.get_logger().warning("没有可丢弃的 episode")
                return
            # 等待后台写入线程完成，确保文件已落盘
            for t in self._write_threads:
                t.join(timeout=60)
            self._write_threads.clear()
            # 取出最后一条 episode
            last_ep = self.writer.episodes.pop()
            ep_idx = int(last_ep["episode_index"])
            n_frames = int(last_ep.get("length", 0))
            # 更新 writer 状态
            self.writer.total_frames -= n_frames
            self.writer.next_episode_index -= 1
            # 删除 parquet 数据文件
            dpath = self.writer._data_path(ep_idx)
            if dpath.exists():
                dpath.unlink()
                logger.info("已删除数据: %s", dpath)
            # 删除视频文件
            for cam in self.writer.active_cameras:
                vpath = self.writer._video_path(ep_idx, cam)
                if vpath.exists():
                    vpath.unlink()
                    logger.info("已删除视频: %s", vpath)
            # 重写 metadata
            self.writer._write_meta()
            self.get_logger().info(
                f"Episode {ep_idx} 已丢弃（已完成, {n_frames} 帧）"
            )

    # ------------------------------------------------------------------ #
    #  采集循环（独立线程，与 ROS 回调并发）
    # ------------------------------------------------------------------ #

    def _capture_loop(self) -> None:
        interval = 1.0 / self.fps
        while self.running:
            try:
                ep = self.current_episode
                if self.recording and ep is not None:
                    # 获取相机帧
                    if self.teleop_mode:
                        fisheye_frame   = self._teleop_fisheye_frame
                        realsense_frame = self._teleop_realsense_frame
                    else:
                        fisheye_frame = None
                        if self.fisheye:
                            try:
                                ok, frame = self.fisheye.get_frame()
                                if ok and frame is not None:
                                    fisheye_frame = frame
                            except Exception as cam_exc:
                                logger.warning("鱼眼相机获取帧失败: %s", cam_exc)

                        realsense_frame = None
                        if self.realsense:
                            try:
                                ok, frame = self.realsense.get_color_frame()
                                if ok and frame is not None:
                                    realsense_frame = frame
                            except Exception as cam_exc:
                                logger.warning("RealSense 获取帧失败: %s", cam_exc)

                    # 若 piper_single_ctrl 未运行/CAN 无反馈，用指令位置代替实际位置
                    obs_joints  = self.joint_state if self._joint_state_received \
                                  else self.joint_cmd
                    obs_gripper = self.gripper_actual
                    ep.add_frame(
                        joints=obs_joints,
                        gripper=obs_gripper,
                        joint_cmd=self.joint_cmd,
                        gripper_cmd=self.gripper_cmd,
                        fisheye_frame=fisheye_frame,
                        realsense_frame=realsense_frame,
                    )
            except Exception as exc:
                logger.error("采集循环异常（已跳过本帧）: %s", exc, exc_info=True)
            time.sleep(interval)

        if not self.teleop_mode:
            self._disconnect_cameras()
        logger.info("采集线程已安全退出 ✅")


# ====================================================================== #
#  入口
# ====================================================================== #

def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s - %(message)s",
    )

    rclpy.init()
    node = DataRecorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("用户中断采集")
    finally:
        node.running = False
        # 若正在录制，先停止并触发写入
        if node.current_episode:
            node._stop_recording()
        # 等待所有后台写入线程完成（最长 60s）
        for t in node._write_threads:
            t.join(timeout=60)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
