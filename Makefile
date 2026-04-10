# ==============================================================================
# Pika ROS2 Piper 控制与数据采集系统
# ==============================================================================
#
# 快速上手:
#   make all                      # 在 tmux 中启动所有进程
#   make keyboard                 # 单独启动键盘控制器 (Process B)
#   make record                   # 单独启动数据采集器 (Process C)
#
# 指定路径与任务:
#   make record TASK="抓取红色方块" SAVE_ROOT=/data/mydata
#   make all    TASK="pick_block"  SAVE_ROOT=/data/mydata
#
# 停止所有:
#   make stop
#
# 查看数据:
#   make list
#   make list SAVE_ROOT=/data/mydata
# ==============================================================================

# ===== 可配置参数（命令行覆盖） =====
CONDA_ENV   ?= py310
PIPER_WS    ?= /home/data/Project/piper_ros
CAN_PORT    ?= can0
USB_PORT    ?= /dev/ttyUSB0
SAVE_ROOT   ?= /home/data/Dataset/piper_lerobot_dataset
TASK        ?= default_task
FPS         ?= 10

# ===== 内部路径（一般无需修改） =====
REPO_DIR     := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
CONDA_PYTHON := /home/hsb/miniforge3/envs/$(CONDA_ENV)/bin/python
CONDA_PKGS   := /home/hsb/miniforge3/envs/$(CONDA_ENV)/lib/python3.10/site-packages

# ROS2 环境初始化命令（在 bash -c 中使用）
ROS_SETUP    := source /opt/ros/humble/setup.bash && \
                source $(PIPER_WS)/install/setup.bash

# py310 conda 环境初始化命令
CONDA_SETUP  := source /home/hsb/miniforge3/etc/profile.d/conda.sh && \
                conda activate $(CONDA_ENV)

# py310 Python 与 ROS2 联用所需环境变量
PY310_ROS_ENV := PYTHON_EXECUTABLE=$(CONDA_PYTHON) \
                 PYTHONPATH=$(CONDA_PKGS):$$PYTHONPATH

.PHONY: all piper robot keyboard record stop list help setup-can check-tmux

# ==============================================================================
#  一键启动（tmux）
# ==============================================================================

all: check-tmux
	@echo "[INFO] 在 tmux session 'pika_ros' 中启动所有进程"
	@tmux new-session  -d -s pika_ros -n piper 2>/dev/null || true
	@# piper 控制节点（窗口 piper）
	@tmux send-keys -t pika_ros:piper \
		"cd $(REPO_DIR) && $(MAKE) setup-can && $(MAKE) piper" Enter
	@# 机械臂控制器（窗口 robot，等待 piper 就绪）
	@tmux new-window -t pika_ros -n robot
	@sleep 4
	@tmux send-keys -t pika_ros:robot \
		"cd $(REPO_DIR) && $(MAKE) robot" Enter
	@# 数据采集器（窗口 recorder）
	@tmux new-window -t pika_ros -n recorder
	@tmux send-keys -t pika_ros:recorder \
		"cd $(REPO_DIR) && $(MAKE) record SAVE_ROOT='$(SAVE_ROOT)' TASK='$(TASK)' FPS=$(FPS)" Enter
	@# 键盘控制器（窗口 keyboard，等待其余节点就绪）
	@tmux new-window -t pika_ros -n keyboard
	@sleep 3
	@tmux send-keys -t pika_ros:keyboard \
		"cd $(REPO_DIR) && $(MAKE) keyboard" Enter
	@echo ""
	@echo "  tmux 会话 'pika_ros' 已启动，包含 4 个窗口："
	@echo "    piper    – piper_single_ctrl 控制节点"
	@echo "    robot    – 机械臂 + 夹爪控制器 (Process A)"
	@echo "    recorder – 数据采集器           (Process C)"
	@echo "    keyboard – 键盘控制器           (Process B)"
	@echo ""
	@echo "  查看: tmux attach -t pika_ros"
	@echo "  停止: make stop"

# ==============================================================================
#  CAN 总线初始化
# ==============================================================================

setup-can:
	@echo "[INFO] 激活 CAN 总线: $(CAN_PORT)"
	@sudo chmod 666 $(USB_PORT) 2>/dev/null || true
	@if [ -f "$(PIPER_WS)/can_activate.sh" ]; then \
		bash $(PIPER_WS)/can_activate.sh $(CAN_PORT) 1000000; \
	else \
		echo "[WARN] 未找到 $(PIPER_WS)/can_activate.sh，跳过 CAN 激活"; \
	fi

# ==============================================================================
#  Process 0: piper_single_ctrl（官方 ROS2 节点，非本项目代码）
# ==============================================================================

piper: setup-can
	@echo "[INFO] 启动 piper_single_ctrl ..."
	@bash -c "$(ROS_SETUP) && \
	          $(PY310_ROS_ENV) \
	          ros2 run piper piper_single_ctrl \
	            --ros-args \
	            -p can_port:=$(CAN_PORT) \
	            -p auto_enable:=true \
	            -p gripper_exist:=true \
	            -p gripper_val_mutiple:=2 \
	            --log-level WARN"

# ==============================================================================
#  Process A: 机械臂控制器（py310 + ROS2）
# ==============================================================================

robot:
	@echo "[INFO] 启动机械臂控制器 (Process A) ..."
	@bash -c "$(CONDA_SETUP) && $(ROS_SETUP) && \
	          cd $(REPO_DIR) && \
	          $(PY310_ROS_ENV) $(CONDA_PYTHON) robot_controller.py"

# ==============================================================================
#  Process B: 键盘控制器（系统 Python3 + ROS2，独立于 conda 环境）
# ==============================================================================

keyboard:
	@echo "[INFO] 启动键盘控制器 (Process B) ..."
	@echo "[INFO] 键位: a/d w/s u/j r/f t/g e/q (关节) | h/k (夹爪)"
	@echo "[INFO]       o/p (采集) | z (回HOME) | 空格 (安全退出)"
	@bash -c "$(ROS_SETUP) && \
	          cd $(REPO_DIR) && \
	          python3 keyboard_controller.py"

# ==============================================================================
#  Process C: 数据采集器（py310 + ROS2）
#  路径与任务通过变量传入
# ==============================================================================

record:
	@echo "[INFO] 启动数据采集器 (Process C)"
	@echo "[INFO]   保存路径 : $(SAVE_ROOT)"
	@echo "[INFO]   任务指令 : $(TASK)"
	@echo "[INFO]   采集帧率 : $(FPS) Hz"
	@bash -c "$(CONDA_SETUP) && $(ROS_SETUP) && \
	          cd $(REPO_DIR) && \
	          $(PY310_ROS_ENV) $(CONDA_PYTHON) data_recorder.py \
	            --save-root '$(SAVE_ROOT)' \
	            --task      '$(TASK)' \
	            --fps       $(FPS)"

# ==============================================================================
#  停止所有节点
# ==============================================================================

stop:
	@echo "[INFO] 停止所有节点 ..."
	@bash -c "$(ROS_SETUP) && \
	  ros2 topic pub --once $(TOPIC_RECORD)  std_msgs/msg/String \"data: 'stop'\"  >/dev/null 2>&1 || true; \
	  ros2 topic pub --once $(TOPIC_CTRL)    std_msgs/msg/String \"data: 'exit'\"  >/dev/null 2>&1 || true; \
	  ros2 topic pub --once $(TOPIC_RECORD)  std_msgs/msg/String \"data: 'exit'\"  >/dev/null 2>&1 || true" \
	  2>/dev/null || true
	@pkill -SIGTERM -f "robot_controller.py"    2>/dev/null || true
	@pkill -SIGTERM -f "keyboard_controller.py" 2>/dev/null || true
	@pkill -SIGTERM -f "data_recorder.py"       2>/dev/null || true
	@sleep 2
	@pkill -9 -f "piper_single_ctrl"            2>/dev/null || true
	@pkill -9 -f "robot_controller.py"          2>/dev/null || true
	@pkill -9 -f "keyboard_controller.py"       2>/dev/null || true
	@pkill -9 -f "data_recorder.py"             2>/dev/null || true
	@tmux kill-session -t pika_ros              2>/dev/null || true
	@echo "[INFO] 所有节点已停止 ✅"

TOPIC_RECORD := /record_cmd
TOPIC_CTRL   := /arm/control_cmd

# ==============================================================================
#  查看已录制数据
# ==============================================================================

list:
	@echo "============================================"
	@echo "数据集路径: $(SAVE_ROOT)"
	@echo "============================================"
	@if [ -f "$(SAVE_ROOT)/meta/info.json" ]; then \
		echo "--- info.json ---"; \
		python3 -c "import json,sys; d=json.load(open('$(SAVE_ROOT)/meta/info.json')); \
		  print('  总 episodes:', d.get('total_episodes',0)); \
		  print('  总帧数    :', d.get('total_frames',0)); \
		  print('  总任务数  :', d.get('total_tasks',0)); \
		  print('  FPS       :', d.get('fps',0))"; \
	fi
	@if [ -f "$(SAVE_ROOT)/meta/tasks.jsonl" ]; then \
		echo "--- 任务列表 ---"; \
		cat "$(SAVE_ROOT)/meta/tasks.jsonl"; \
	fi
	@if [ -f "$(SAVE_ROOT)/meta/episodes.jsonl" ]; then \
		echo "--- episodes (最近 10 条) ---"; \
		tail -n 10 "$(SAVE_ROOT)/meta/episodes.jsonl"; \
	fi
	@echo ""
	@echo "Parquet 文件数: $$(find '$(SAVE_ROOT)/data' -name '*.parquet' 2>/dev/null | wc -l)"
	@echo "MP4 文件数    : $$(find '$(SAVE_ROOT)/videos' -name '*.mp4' 2>/dev/null | wc -l)"

# ==============================================================================
#  帮助
# ==============================================================================

help:
	@echo ""
	@echo "Pika ROS2 Piper 控制与数据采集系统"
	@echo ""
	@echo "目标:"
	@echo "  make all        在 tmux 中一键启动所有进程（需要 tmux）"
	@echo "  make piper      启动 piper_single_ctrl 控制节点"
	@echo "  make robot      启动机械臂控制器      (Process A, py310)"
	@echo "  make keyboard   启动键盘控制器         (Process B, 系统Python3)"
	@echo "  make record     启动数据采集器         (Process C, py310)"
	@echo "  make stop       停止所有节点"
	@echo "  make list       查看已录制的数据集信息"
	@echo ""
	@echo "可配置变量:"
	@echo "  SAVE_ROOT  数据集保存根目录  (当前: $(SAVE_ROOT))"
	@echo "  TASK       任务指令文本       (当前: $(TASK))"
	@echo "  FPS        采集帧率          (当前: $(FPS))"
	@echo "  CAN_PORT   CAN 总线接口      (当前: $(CAN_PORT))"
	@echo "  CONDA_ENV  Conda 环境名      (当前: $(CONDA_ENV))"
	@echo "  PIPER_WS   piper_ros 路径    (当前: $(PIPER_WS))"
	@echo ""
	@echo "示例:"
	@echo "  make record TASK='抓取红色方块' SAVE_ROOT=/data/dataset"
	@echo "  make all    TASK='pick_cube'    SAVE_ROOT=/data/dataset"
	@echo "  make list   SAVE_ROOT=/data/dataset"
	@echo ""

# ==============================================================================
#  内部工具
# ==============================================================================

check-tmux:
	@which tmux > /dev/null 2>&1 || \
		(echo "[ERROR] 需要 tmux: sudo apt install tmux" && exit 1)
