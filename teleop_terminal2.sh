#!/bin/bash
# FK/IK + 遥操作节点启动
# 由 make teleop 调用，环境变量由 Makefile 传入
set -e

PIKA_WS="${PIKA_WS:-/home/data/Project/pika_ros}"
CONDA_PYTHON="${CONDA_PYTHON:-/home/hsb/miniforge3/envs/py310/bin/python}"
CONDA_PKGS="${CONDA_PKGS:-/home/hsb/miniforge3/envs/py310/lib/python3.10/site-packages}"

export PYTHON_EXECUTABLE="${CONDA_PYTHON}"
export PYTHONPATH="${CONDA_PKGS}:${PYTHONPATH}"
source "${PIKA_WS}/install/setup.bash"
ros2 launch pika_remote_piper teleop_rand_single_piper.launch.py
