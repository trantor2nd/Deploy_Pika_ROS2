#!/bin/bash
# CAN 初始化 + 传感器/夹爪节点启动
# 由 make teleop 调用，环境变量由 Makefile 传入
set -e

PIPER_WS="${PIPER_WS:-/home/data/Project/pika_ros}"
CAN_PORT="${CAN_PORT:-can0}"

cd "${PIPER_WS}/src/PikaAnyArm/piper/piper_ros"
source ./can_activate.sh "${CAN_PORT}" 1000000
source "${PIPER_WS}/install/setup.bash"
cd "${PIPER_WS}/scripts" && bash start_sensor_gripper.bash
