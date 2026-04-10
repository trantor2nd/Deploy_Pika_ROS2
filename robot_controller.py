#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Process A: 机械臂控制器
============================================================
职责:
  - 控制 Piper 机械臂（通过 ROS2 piper_single_ctrl 节点）
  - 控制 Pika 夹爪（串口直连）
  - 监听 Process B 的控制指令，执行使能/控制/失能/回零
  - 向 Process C 发布夹爪实际状态

运行环境: py310 conda + ROS2 Humble

启动:
  source /opt/ros/humble/setup.bash
  source <piper_ws>/install/setup.bash
  python robot_controller.py
"""

import logging
import sys
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from pika.gripper import Gripper

# 将当前目录加入 Python 路径，确保能 import config
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import (
    GRIPPER_PORT, GRIPPER_MIN, GRIPPER_MAX,
    HOME_POS, JOINT_LIMITS,
    TOPIC_JOINT_CMD, TOPIC_GRIPPER_CMD, TOPIC_CONTROL_CMD,
    TOPIC_JOINT_STATES, TOPIC_GRIPPER_STATE,
    TOPIC_JOINT_CTRL, TOPIC_ENABLE_FLAG,
)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class RobotController(Node):
    """
    机械臂控制器节点。

    订阅来自 Process B 的控制指令，将关节指令转发给 piper_single_ctrl，
    直接控制 Pika 夹爪（串口），并发布夹爪实际状态。
    """

    def __init__(self) -> None:
        super().__init__("robot_controller")
        logging.getLogger("pika.serial_comm").setLevel(logging.ERROR)

        # ---------- 发布者 ----------
        self.pub_joint_ctrl   = self.create_publisher(JointState, TOPIC_JOINT_CTRL,    10)
        self.pub_gripper_state = self.create_publisher(Float64,    TOPIC_GRIPPER_STATE, 10)
        self.pub_enable        = self.create_publisher(Bool,       TOPIC_ENABLE_FLAG,   10)

        # ---------- 订阅者 ----------
        self.create_subscription(JointState, TOPIC_JOINT_CMD,    self._joint_cmd_cb,   10)
        self.create_subscription(Float64,    TOPIC_GRIPPER_CMD,  self._gripper_cmd_cb, 10)
        self.create_subscription(String,     TOPIC_CONTROL_CMD,  self._control_cmd_cb, 10)
        self.create_subscription(JointState, TOPIC_JOINT_STATES, self._joint_states_cb, 10)

        # ---------- 状态 ----------
        self.actual_joint_positions = list(HOME_POS)
        self.gripper_pos = 0.0
        self._running = True
        self._zero_glitch_count = 0

        # ---------- 夹爪 ----------
        self.gripper: Gripper | None = None
        self._init_gripper()

        # 夹爪状态定期轮询 (10 Hz)
        self._gripper_poll_timer = self.create_timer(0.1, self._poll_gripper)

        self.get_logger().info("RobotController 节点已启动 ✅")

    # ------------------------------------------------------------------ #
    #  夹爪初始化
    # ------------------------------------------------------------------ #

    def _init_gripper(self) -> None:
        try:
            self.get_logger().info(f"连接夹爪 {GRIPPER_PORT} ...")
            self.gripper = Gripper(GRIPPER_PORT)
            if not self.gripper.connect():
                self.get_logger().warning("夹爪连接失败，继续无夹爪模式")
                self.gripper = None
                return
            if not self.gripper.enable():
                self.get_logger().warning("夹爪上电失败，继续无夹爪模式")
                self.gripper = None
                return
            self.gripper_pos = clamp(
                self.gripper.get_gripper_distance(), GRIPPER_MIN, GRIPPER_MAX
            )
            self._publish_gripper_state()
            self.pub_enable.publish(Bool(data=True))
            self.get_logger().info("夹爪连接成功 ✅")
        except Exception as exc:
            self.get_logger().error(f"初始化夹爪异常: {exc}")
            self.gripper = None

    # ------------------------------------------------------------------ #
    #  夹爪状态
    # ------------------------------------------------------------------ #

    def _publish_gripper_state(self) -> None:
        self.pub_gripper_state.publish(Float64(data=float(self.gripper_pos)))

    def _poll_gripper(self) -> None:
        if not self.gripper:
            return
        try:
            current = clamp(
                self.gripper.get_gripper_distance(), GRIPPER_MIN, GRIPPER_MAX
            )
        except Exception:
            return

        # 防抖：靠近 0 时过滤毛刺
        if current < GRIPPER_MIN + 1.0 and self.gripper_pos > GRIPPER_MIN + 5.0:
            self._zero_glitch_count += 1
            if self._zero_glitch_count < 3:
                return
        else:
            self._zero_glitch_count = 0

        if abs(current - self.gripper_pos) > 0.5:
            self.gripper_pos = current
            self._publish_gripper_state()

    # ------------------------------------------------------------------ #
    #  回调：关节状态反馈
    # ------------------------------------------------------------------ #

    def _joint_states_cb(self, msg: JointState) -> None:
        if len(msg.position) >= 6:
            self.actual_joint_positions = list(msg.position[:6])

    # ------------------------------------------------------------------ #
    #  回调：关节控制指令（来自 Process B，透传给 piper_single_ctrl）
    # ------------------------------------------------------------------ #

    def _joint_cmd_cb(self, msg: JointState) -> None:
        # pika_ros version of piper_ctrl_single_node accesses position[6] unconditionally;
        # pad to 7 joints (arm×6 + gripper=0) to avoid IndexError.
        if len(msg.position) < 7:
            fwd = JointState()
            fwd.header   = msg.header
            fwd.name     = list(msg.name) + ["gripper"]
            fwd.position = list(msg.position) + [0.0]
            self.pub_joint_ctrl.publish(fwd)
        else:
            self.pub_joint_ctrl.publish(msg)

    # ------------------------------------------------------------------ #
    #  回调：夹爪控制指令
    # ------------------------------------------------------------------ #

    def _gripper_cmd_cb(self, msg: Float64) -> None:
        if not self.gripper:
            return
        dist = clamp(msg.data, GRIPPER_MIN, GRIPPER_MAX)
        self.gripper_pos = dist
        try:
            self.gripper.set_gripper_distance(dist)
            self._publish_gripper_state()
        except Exception as exc:
            self.get_logger().warning(f"设置夹爪失败: {exc}")

    # ------------------------------------------------------------------ #
    #  回调：综合控制指令（home / enable / disable / exit）
    # ------------------------------------------------------------------ #

    def _control_cmd_cb(self, msg: String) -> None:
        cmd = msg.data.lower().strip()
        if cmd == "enable":
            self._do_enable()
        elif cmd == "disable":
            self._do_disable()
        elif cmd == "home":
            self._do_home()
        elif cmd == "exit":
            self._do_exit()

    # ------------------------------------------------------------------ #
    #  控制动作
    # ------------------------------------------------------------------ #

    def _do_enable(self) -> None:
        self.pub_enable.publish(Bool(data=True))
        if self.gripper:
            try:
                self.gripper.enable()
            except Exception:
                pass
        self.get_logger().info("机械臂已使能")

    def _do_disable(self) -> None:
        self.pub_enable.publish(Bool(data=False))
        if self.gripper:
            try:
                self.gripper.disable()
            except Exception:
                pass
        self.get_logger().info("机械臂已失能")

    def _do_home(self, duration: float = 3.0, steps: int = 30) -> None:
        self.get_logger().info("回 HOME 位置 ...")
        start  = list(self.actual_joint_positions)
        target = [clamp(v, *JOINT_LIMITS[i]) for i, v in enumerate(HOME_POS)]
        for i in range(1, steps + 1):
            r = i / steps
            positions = [s + (t - s) * r for s, t in zip(start, target)]
            msg = JointState()
            msg.name     = [f"joint{j + 1}" for j in range(6)] + ["gripper"]
            msg.position = positions + [0.0]
            self.pub_joint_ctrl.publish(msg)
            time.sleep(duration / steps)
        self.get_logger().info("已到达 HOME 位置 ✅")

    def _do_exit(self) -> None:
        self.get_logger().info("收到退出指令，执行安全退出 ...")
        self._do_home()
        self._do_disable()
        self._running = False

    # ------------------------------------------------------------------ #
    #  清理
    # ------------------------------------------------------------------ #

    def shutdown(self) -> None:
        self._gripper_poll_timer.cancel()
        if self.gripper:
            try:
                self.gripper.disable()
                self.gripper.disconnect()
                self.get_logger().info("夹爪已断开")
            except Exception:
                pass


# ------------------------------------------------------------------ #
#  入口
# ------------------------------------------------------------------ #

def main() -> None:
    rclpy.init()
    node = RobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and node._running:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
