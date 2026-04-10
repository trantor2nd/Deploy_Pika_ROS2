#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Shared configuration constants for all processes.
No external imports – safe to use from any Python environment.
"""

# ===== 机械臂关节限制 =====
JOINT_LIMITS = [
    (-3.14,  3.14),
    ( 0.00,  2.00),
    (-2.00,  0.00),
    (-1.50,  1.80),
    (-1.30,  1.57),
    (-3.14,  3.14),
]

HOME_POS = [0.0, -0.035, 0.0, 0.0, 0.35, 0.0]
JOINT_STEP = 0.025          # 每次按键的关节步进量 (rad)

# ===== 夹爪 =====
GRIPPER_PORT  = "/dev/ttyUSB0"
GRIPPER_MIN   = 0.0          # 完全闭合 (mm)
GRIPPER_MAX   = 90.0         # 完全张开 (mm)
GRIPPER_STEP  = 10.0         # 每次按键步进量 (mm)

# ===== 相机 =====
FISHEYE_INDEX  = 6
FISHEYE_WIDTH  = 640
FISHEYE_HEIGHT = 480
FISHEYE_FPS    = 30

REALSENSE_SN     = "230322275684"
REALSENSE_WIDTH  = 640
REALSENSE_HEIGHT = 480
REALSENSE_FPS    = 30

# ===== 采集 =====
CAPTURE_HZ        = 10.0
DEFAULT_SAVE_ROOT = "/home/data/Dataset/piper_lerobot_dataset"
DEFAULT_TASK      = "default_task"

# ===== ROS2 话题名 =====
TOPIC_JOINT_CMD    = "/arm/joint_cmd"       # Process B -> A: JointState (目标关节位置)
TOPIC_GRIPPER_CMD  = "/arm/gripper_cmd"     # Process B -> A: Float64 (目标夹爪距离 mm)
TOPIC_CONTROL_CMD  = "/arm/control_cmd"     # Process B -> A: String (home/enable/disable/exit)

TOPIC_RECORD_CMD   = "/record_cmd"          # Process B -> C: String (start/stop/exit)

TOPIC_JOINT_STATES = "/joint_states_single" # piper_single_ctrl -> all: JointState (实际关节)
TOPIC_GRIPPER_STATE = "/gripper_state"      # Process A -> C: Float64 (实际夹爪距离)
TOPIC_JOINT_CTRL   = "/joint_ctrl_single"   # Process A -> piper: JointState (控制指令)
TOPIC_ENABLE_FLAG  = "/enable_flag"         # Process A -> piper: Bool (使能/失能)

# ===== LeRobot 数据集 =====
EPISODES_PER_CHUNK = 1000   # 每个 chunk 存放的最大 episode 数
