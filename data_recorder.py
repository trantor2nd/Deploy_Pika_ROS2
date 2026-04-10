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

# pyarrow 用于写 Parquet
import pyarrow as pa
import pyarrow.parquet as pq

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from pika.camera.fisheye import FisheyeCamera
from pika.camera.realsense import RealSenseCamera

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import (
    FISHEYE_INDEX, FISHEYE_WIDTH, FISHEYE_HEIGHT, FISHEYE_FPS,
    REALSENSE_SN, REALSENSE_WIDTH, REALSENSE_HEIGHT, REALSENSE_FPS,
    CAPTURE_HZ, DEFAULT_SAVE_ROOT, DEFAULT_TASK, HOME_POS,
    TOPIC_JOINT_STATES, TOPIC_GRIPPER_STATE, TOPIC_RECORD_CMD,
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
    return parser.parse_args()


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
        path  = self.root / "videos" / f"chunk-{chunk:03d}" / \
                f"observation.images.{cam}"
        path.mkdir(parents=True, exist_ok=True)
        return path / f"episode_{episode_index:06d}.mp4"

    # ------------------------------------------------------------------ #

    def _load_meta(self) -> None:
        """读取已有的 episodes / tasks，以便追加录制。"""
        eps_file   = self.root / "meta" / "episodes.jsonl"
        tasks_file = self.root / "meta" / "tasks.jsonl"

        self.episodes: list[dict] = []
        if eps_file.exists():
            with open(eps_file) as f:
                for line in f:
                    line = line.strip()
                    if line:
                        self.episodes.append(json.loads(line))

        self.tasks_map: dict[str, int] = {}  # task_text -> task_index
        if tasks_file.exists():
            with open(tasks_file) as f:
                for line in f:
                    line = line.strip()
                    if line:
                        obj = json.loads(line)
                        self.tasks_map[obj["task"]] = obj["task_index"]

        self.total_frames        = sum(ep.get("length", 0) for ep in self.episodes)
        self.next_episode_index  = len(self.episodes)

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
        n = ep.frame_count
        states  = np.stack(ep.states, axis=0)       # (N, 7)
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
                "observation.state": [states[i].tolist() for i in range(n)],
                "action":            [states[i].tolist() for i in range(n)],
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
        # episodes.jsonl
        with open(self.root / "meta" / "episodes.jsonl", "w") as f:
            for ep in self.episodes:
                f.write(json.dumps(ep, ensure_ascii=False) + "\n")

        # tasks.jsonl
        with open(self.root / "meta" / "tasks.jsonl", "w") as f:
            for task_text, task_idx in self.tasks_map.items():
                f.write(json.dumps(
                    {"task_index": task_idx, "task": task_text},
                    ensure_ascii=False,
                ) + "\n")

        # info.json
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
            "codebase_version": "v3.0",
            "robot_type":       "piper",
            "fps":              self.fps,
            "features":         features,
            "total_episodes":   len(self.episodes),
            "total_frames":     self.total_frames,
            "total_tasks":      len(self.tasks_map),
            "splits":           {"train": f"0:{len(self.episodes)}"},
            "data_path":        "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path":       "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
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
        self.states:     list[np.ndarray] = []
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
        fisheye_frame:   Optional[np.ndarray] = None,
        realsense_frame: Optional[np.ndarray] = None,
    ) -> None:
        """写入一帧数据。"""
        t = time.time() - self.start_time
        state = np.array(list(joints[:6]) + [gripper], dtype=np.float32)
        self.states.append(state)
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
        logging.getLogger("pika.serial_comm").setLevel(logging.ERROR)
        logging.getLogger("pika.camera.realsense").setLevel(logging.ERROR)

        self.fps  = args.fps
        self.task = args.task

        # 连接相机，确定可用相机列表
        self.fisheye   = None
        self.realsense = None
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

        # ROS2 订阅
        self.create_subscription(
            JointState, TOPIC_JOINT_STATES, self._joint_cb, 10
        )
        self.create_subscription(
            Float64, TOPIC_GRIPPER_STATE, self._gripper_cb, 10
        )
        self.create_subscription(
            String, TOPIC_RECORD_CMD, self._cmd_cb, 10
        )

        # 本地状态缓存
        self.joint_state:    list[float] = list(HOME_POS)
        self.gripper_actual: float       = 0.0
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

        self.get_logger().info(
            f"DataRecorder 节点启动 ✅  save_root={args.save_root}  task={args.task}"
        )

    # ------------------------------------------------------------------ #
    #  相机连接
    # ------------------------------------------------------------------ #

    def _connect_cameras(self) -> None:
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
        if len(msg.position) >= 6:
            self.joint_state = list(msg.position[:6])

    def _gripper_cb(self, msg: Float64) -> None:
        self.gripper_actual = msg.data

    def _cmd_cb(self, msg: String) -> None:
        cmd = msg.data.lower().strip()
        if cmd == "start" and not self.recording:
            self._start_recording()
        elif cmd == "stop" and self.recording:
            self._stop_recording()
        elif cmd == "exit":
            if self.recording:
                self._stop_recording()
            self.running = False

    # ------------------------------------------------------------------ #
    #  录制控制
    # ------------------------------------------------------------------ #

    def _start_recording(self) -> None:
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

    # ------------------------------------------------------------------ #
    #  采集循环（独立线程，与 ROS 回调并发）
    # ------------------------------------------------------------------ #

    def _capture_loop(self) -> None:
        interval = 1.0 / self.fps
        while self.running:
            try:
                ep = self.current_episode
                if self.recording and ep is not None:
                    # 获取相机帧（单独 try，避免相机异常杀死整个线程）
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

                    ep.add_frame(
                        joints=self.joint_state,
                        gripper=self.gripper_actual,
                        fisheye_frame=fisheye_frame,
                        realsense_frame=realsense_frame,
                    )
            except Exception as exc:
                logger.error("采集循环异常（已跳过本帧）: %s", exc, exc_info=True)
            time.sleep(interval)

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
