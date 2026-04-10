#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Process B: 键盘控制器
============================================================
职责:
  - 独立于 py310 环境，通过键盘发送控制信号
  - 14 个按键控制 6-DOF 关节 + 夹爪（各 2 个方向）
  - 开始/停止数据采集
  - 复位（回 HOME）
  - 安全退出（先回 HOME，再失能）

键位映射:
  关节控制 (各 +/-):
    j1: a / d       j2: w / s
    j3: u / j       j4: r / f
    j5: t / g       j6: e / q
  夹爪: h (张开) / k (闭合)
  采集: o (开始) / p (停止)
  复位: z
  使能: n  失能: m
  退出: 空格 (安全退出)

运行环境: 任意 Python3 + ROS2 Humble
  source /opt/ros/humble/setup.bash
  source <piper_ws>/install/setup.bash
  python3 keyboard_controller.py
"""

import os
import select
import sys
import termios
import time
import tty

# 将当前目录加入 Python 路径，确保能 import config
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from config import (
    JOINT_LIMITS, HOME_POS, JOINT_STEP,
    GRIPPER_MIN, GRIPPER_MAX, GRIPPER_STEP,
    TOPIC_JOINT_CMD, TOPIC_GRIPPER_CMD, TOPIC_CONTROL_CMD,
    TOPIC_RECORD_CMD, TOPIC_JOINT_STATES,
)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class KeyboardController(Node):
    """
    键盘控制器节点。

    将键盘输入转换为 ROS2 话题消息，驱动机械臂控制器（Process A）
    和数据采集器（Process C）。
    """

    def __init__(self) -> None:
        super().__init__("keyboard_controller")

        # ---------- 发布者 ----------
        self.pub_joint   = self.create_publisher(JointState, TOPIC_JOINT_CMD,   10)
        self.pub_gripper = self.create_publisher(Float64,    TOPIC_GRIPPER_CMD, 10)
        self.pub_control = self.create_publisher(String,     TOPIC_CONTROL_CMD, 10)
        self.pub_record  = self.create_publisher(String,     TOPIC_RECORD_CMD,  10)

        # ---------- 反馈订阅（同步本地关节缓存） ----------
        self.create_subscription(
            JointState, TOPIC_JOINT_STATES, self._feedback_cb, 10
        )

        # ---------- 本地状态 ----------
        self.joint_positions = list(HOME_POS)
        self.gripper_pos     = 0.0
        self.recording       = False
        self._running        = True

        # 每个关节的按键 -> (轴索引, 步进量)
        s = JOINT_STEP
        self.key_map: dict[str, tuple[int, float]] = {
            "a": (0, +s / 2.0),  "d": (0, -s / 2.0),
            "w": (1, +s / 1.25), "s": (1, -s / 1.25),
            "u": (2, -s / 1.25), "j": (2, +s / 1.25),
            "r": (3, +s),        "f": (3, -s),
            "t": (4, +s),        "g": (4, -s),
            "e": (5, +s),        "q": (5, -s),
        }

    # ------------------------------------------------------------------ #
    #  反馈
    # ------------------------------------------------------------------ #

    def _feedback_cb(self, msg: JointState) -> None:
        """同步来自 piper_single_ctrl 的实际关节位置到本地缓存。"""
        if len(msg.position) >= 6:
            self.joint_positions = list(msg.position[:6])

    # ------------------------------------------------------------------ #
    #  发布帮助函数
    # ------------------------------------------------------------------ #

    def _publish_joint(self) -> None:
        msg = JointState()
        msg.name     = [f"joint{i + 1}" for i in range(6)]
        msg.position = list(self.joint_positions)
        self.pub_joint.publish(msg)

    def _publish_gripper(self) -> None:
        self.pub_gripper.publish(Float64(data=float(self.gripper_pos)))

    def _send_control(self, cmd: str) -> None:
        self.pub_control.publish(String(data=cmd))

    # ------------------------------------------------------------------ #
    #  主循环
    # ------------------------------------------------------------------ #

    def run(self) -> None:
        executor = SingleThreadedExecutor()
        executor.add_node(self)

        # 等待 ROS 图稳定后发使能指令
        time.sleep(0.5)
        self._send_control("enable")

        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        sys.stdin.flush()

        self._print_help()

        try:
            while self._running and rclpy.ok():
                executor.spin_once(timeout_sec=0.0)
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not rlist:
                    continue

                key = sys.stdin.read(1)
                self._handle_key(key)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print()  # 换行，让终端输出整洁

    def _handle_key(self, key: str) -> None:
        # ----- 退出（安全退出）-----
        if key == " ":
            print("\n[INFO] 安全退出中 ...")
            if self.recording:
                self.pub_record.publish(String(data="stop"))
                self.recording = False
                print("[REC] 已停止采集")
            self._send_control("exit")   # 触发 Process A 回 HOME + 失能
            self.pub_record.publish(String(data="exit"))  # 通知 Process C 退出
            time.sleep(6.0)              # 等待 Process A 完成回零
            self._running = False
            return

        # ----- 复位 -----
        if key in ("z", "Z"):
            print("[CMD] 回 HOME")
            self._send_control("home")
            self.joint_positions = list(HOME_POS)
            return

        # ----- 使能 / 失能 -----
        if key in ("n", "N"):
            print("[CMD] 使能")
            self._send_control("enable")
            return
        if key in ("m", "M"):
            print("[CMD] 失能")
            self._send_control("disable")
            return

        # ----- 数据采集 -----
        if key in ("o", "O"):
            if not self.recording:
                self.pub_record.publish(String(data="start"))
                self.recording = True
                print("[REC] ● 开始采集")
            return
        if key in ("p", "P"):
            if self.recording:
                self.pub_record.publish(String(data="stop"))
                self.recording = False
                print("[REC] ■ 停止采集")
            return

        # ----- 夹爪 -----
        if key in ("h", "H"):
            self.gripper_pos = clamp(
                self.gripper_pos + GRIPPER_STEP, GRIPPER_MIN, GRIPPER_MAX
            )
            self._publish_gripper()
            print(f"[GRIP] 张开 → {self.gripper_pos:.0f} mm", end="\r", flush=True)
            return
        if key in ("k", "K"):
            self.gripper_pos = clamp(
                self.gripper_pos - GRIPPER_STEP, GRIPPER_MIN, GRIPPER_MAX
            )
            self._publish_gripper()
            print(f"[GRIP] 闭合 → {self.gripper_pos:.0f} mm", end="\r", flush=True)
            return

        # ----- 关节控制（14 键）-----
        lower = key.lower()
        if lower in self.key_map:
            idx, delta = self.key_map[lower]
            lo, hi = JOINT_LIMITS[idx]
            self.joint_positions[idx] = clamp(
                self.joint_positions[idx] + delta, lo, hi
            )
            self._publish_joint()
            parts = " | ".join(
                f"j{i + 1}:{v:+.3f}" for i, v in enumerate(self.joint_positions)
            )
            print(f"[CMD] {parts}", end="\r", flush=True)

    # ------------------------------------------------------------------ #
    #  帮助信息
    # ------------------------------------------------------------------ #

    @staticmethod
    def _print_help() -> None:
        print("=" * 60)
        print("Piper 键盘控制器")
        print("=" * 60)
        print("关节控制 (14 键, +/-):")
        print("  j1: a / d     j2: w / s     j3: u / j")
        print("  j4: r / f     j5: t / g     j6: e / q")
        print("夹爪:  h (张开) / k (闭合)")
        print("采集:  o (开始) / p (停止)")
        print("机械臂: z (回HOME) | n (使能) | m (失能)")
        print("退出:  空格  [安全退出: 先回HOME再失能]")
        print("=" * 60, flush=True)


# ------------------------------------------------------------------ #
#  入口
# ------------------------------------------------------------------ #

def main() -> None:
    rclpy.init()
    node = KeyboardController()
    try:
        node.run()
    except KeyboardInterrupt:
        print("\n[INFO] 用户中断")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
