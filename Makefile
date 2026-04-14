# ==============================================================================
# Pika ROS2 Piper 控制与数据采集系统
# ==============================================================================
#
# 快速上手:
#   make control                   # 键盘模式：在 tmux 中启动所有进程
#   make keyboard                 # 单独启动键盘控制器 (Process B)
#   make record                   # 单独启动数据采集器 (Process C)
#
# 指定路径与任务:
#   make record TASK="抓取红色方块" SAVE_ROOT=/data/mydata
#   make control TASK="pick_block"  SAVE_ROOT=/data/mydata
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
PIKA_WS     ?= /home/data/Project/pika_ros
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
                source $(PIPER_WS)/install/setup.bash && \
                source $(PIKA_WS)/install/setup.bash

# py310 conda 环境初始化命令
CONDA_SETUP  := source /home/hsb/miniforge3/etc/profile.d/conda.sh && \
                conda activate $(CONDA_ENV)

# py310 Python 与 ROS2 联用所需环境变量
PY310_ROS_ENV := PYTHON_EXECUTABLE=$(CONDA_PYTHON) \
                 PYTHONPATH=$(CONDA_PKGS):$$PYTHONPATH

.PHONY: control piper robot keyboard record stop list migrate viz help setup-can check-tmux teleop record-teleop calibrate test-camera

# ==============================================================================
#  键盘模式一键启动（tmux）
# ==============================================================================

control: check-tmux
	@# ── 第一步：在当前终端完成 CAN 初始化（sudo 在此输入，不会藏进 tmux）──
	$(MAKE) setup-can
	@# ── 第二步：创建 2×2 分屏，各进程错峰自动启动 ──
	@tmux kill-session -t pika_ros 2>/dev/null || true
	@echo "[INFO] 创建 2×2 分屏布局 ..."
	@tmux new-session -d -s pika_ros
	@tmux split-window -h -t pika_ros:0.0
	@tmux split-window -v -t pika_ros:0.0
	@tmux split-window -v -t pika_ros:0.1
	@tmux select-layout -t pika_ros tiled
	@tmux set-option -t pika_ros mouse on
	@# pane 0.0=piper  0.1=robot  0.2=recorder  0.3=keyboard
	@# piper 立刻启动；robot/recorder 等 8s；keyboard 等 12s
	@tmux send-keys -t pika_ros:0.0 \
		"cd $(REPO_DIR) && $(MAKE) piper" Enter
	@tmux send-keys -t pika_ros:0.1 \
		"sleep 8  && cd $(REPO_DIR) && $(MAKE) robot" Enter
	@tmux send-keys -t pika_ros:0.2 \
		"sleep 8  && cd $(REPO_DIR) && $(MAKE) record SAVE_ROOT='$(SAVE_ROOT)' TASK='$(TASK)' FPS=$(FPS)" Enter
	@tmux send-keys -t pika_ros:0.3 \
		"sleep 12 && cd $(REPO_DIR) && $(MAKE) keyboard" Enter
	@echo ""
	@echo "  2×2 分屏已就绪:"
	@echo "  ┌─────────────────┬─────────────────┐"
	@echo "  │ 0.0  piper      │ 0.1  robot      │"
	@echo "  ├─────────────────┼─────────────────┤"
	@echo "  │ 0.2  recorder   │ 0.3  keyboard   │"
	@echo "  └─────────────────┴─────────────────┘"
	@echo "  停止: make stop   切换格子: 鼠标点击 或 Ctrl-b q (数字)"
	@echo ""
	@if [ -z "$$TMUX" ]; then \
		tmux attach-session -t pika_ros; \
	else \
		tmux switch-client -t pika_ros; \
	fi

# ==============================================================================
#  CAN 总线初始化
# ==============================================================================

setup-can:
	@echo "[INFO] 重置并激活 CAN 总线: $(CAN_PORT)"
	@sudo chmod 666 $(USB_PORT) 2>/dev/null || true
	@# 强制先 down，避免 can_activate.sh 检测到"已激活"而跳过重置
	@# （ERROR-PASSIVE 等错误状态需要 down/up 才能清除）
	@sudo ip link set $(CAN_PORT) down 2>/dev/null || true
	@if [ -f "$(PIPER_WS)/can_activate.sh" ]; then \
		bash $(PIPER_WS)/can_activate.sh $(CAN_PORT) 1000000; \
	else \
		echo "[WARN] 未找到 $(PIPER_WS)/can_activate.sh，手动激活 CAN"; \
		sudo ip link set $(CAN_PORT) type can bitrate 1000000 && \
		sudo ip link set $(CAN_PORT) up; \
	fi
	@echo "[INFO] CAN 状态:"; ip -details link show $(CAN_PORT) | grep "can state"

# ==============================================================================
#  Process 0: piper_single_ctrl（官方 ROS2 节点，非本项目代码）
# ==============================================================================

piper: setup-can
	@echo "[INFO] 等待 CAN 总线稳定 ..."
	@sleep 2
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
	@echo "[INFO]       o/p (采集) | l (丢弃) | z (回HOME) | 空格 (安全退出)"
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
#  遥操作数据采集（需先手动启动 teleop: terminal1.sh + terminal2.sh）
# ==============================================================================

teleop: check-tmux
	@tmux kill-session -t pika_teleop 2>/dev/null || true
	@echo "[INFO] 遥操作采集模式（自动启动全部节点）"
	@echo "[INFO]   保存路径 : $(SAVE_ROOT)"
	@echo "[INFO]   任务指令 : $(TASK)"
	@echo "[INFO]   采集帧率 : $(FPS) Hz"
	@tmux new-session -d -s pika_teleop
	@tmux split-window -h -t pika_teleop:0.0
	@tmux split-window -v -t pika_teleop:0.0
	@tmux split-window -v -t pika_teleop:0.1
	@tmux select-layout -t pika_teleop tiled
	@tmux set-option -t pika_teleop mouse on
	@# pane 0.0=CAN+sensor  0.1=FK/IK/teleop  0.2=recorder  0.3=keyboard
	@tmux send-keys -t pika_teleop:0.0 \
		"cd $(REPO_DIR) && PIKA_WS=$(PIKA_WS) CAN_PORT=$(CAN_PORT) bash teleop_terminal1.sh" Enter
	@tmux send-keys -t pika_teleop:0.1 \
		"sleep 10 && cd $(REPO_DIR) && PIKA_WS=$(PIKA_WS) CONDA_PYTHON=$(CONDA_PYTHON) CONDA_PKGS=$(CONDA_PKGS) bash teleop_terminal2.sh" Enter
	@tmux send-keys -t pika_teleop:0.2 \
		"sleep 15 && cd $(REPO_DIR) && $(MAKE) record-teleop SAVE_ROOT='$(SAVE_ROOT)' TASK='$(TASK)' FPS=$(FPS)" Enter
	@tmux send-keys -t pika_teleop:0.3 \
		"sleep 20 && cd $(REPO_DIR) && $(MAKE) keyboard" Enter
	@echo ""
	@echo "  遥操作采集已就绪 (4 窗格):"
	@echo "  ┌──────────────────┬──────────────────┐"
	@echo "  │ 0.0  CAN+sensor  │ 0.1  FK/IK/teleop│"
	@echo "  ├──────────────────┼──────────────────┤"
	@echo "  │ 0.2  recorder    │ 0.3  keyboard    │"
	@echo "  └──────────────────┴──────────────────┘"
	@echo "  停止: make stop   键盘窗口按 o/p/l 控制采集"
	@echo ""
	@if [ -z "$$TMUX" ]; then \
		tmux attach-session -t pika_teleop; \
	else \
		tmux switch-client -t pika_teleop; \
	fi

record-teleop:
	@echo "[INFO] 启动数据采集器 (遥操作模式)"
	@echo "[INFO]   保存路径 : $(SAVE_ROOT)"
	@echo "[INFO]   任务指令 : $(TASK)"
	@echo "[INFO]   采集帧率 : $(FPS) Hz"
	@bash -c "$(CONDA_SETUP) && $(ROS_SETUP) && \
	          cd $(REPO_DIR) && \
	          $(PY310_ROS_ENV) $(CONDA_PYTHON) data_recorder.py \
	            --save-root '$(SAVE_ROOT)' \
	            --task      '$(TASK)' \
	            --fps       $(FPS) \
	            --mode      teleop"

# ==============================================================================
#  测试相机
# ==============================================================================

test-camera:
	@echo "[INFO] 测试鱼眼 + RealSense 相机（按 q 退出, 按 s 保存截图）"
	@bash -c "$(CONDA_SETUP) && \
	          cd $(REPO_DIR) && \
	          $(CONDA_PYTHON) test_camera.py"

# ==============================================================================
#  标定（libsurvive VR 追踪设备）
# ==============================================================================

calibrate:
	@echo "[INFO] 启动 libsurvive 标定 ..."
	@bash $(REPO_DIR)/calibrate.sh

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
	@pkill -SIGTERM -f "teleop_rand_single_piper" 2>/dev/null || true
	@pkill -SIGTERM -f "start_sensor_gripper"   2>/dev/null || true
	@sleep 2
	@pkill -9 -f "piper_single_ctrl"            2>/dev/null || true
	@pkill -9 -f "robot_controller.py"          2>/dev/null || true
	@pkill -9 -f "keyboard_controller.py"       2>/dev/null || true
	@pkill -9 -f "data_recorder.py"             2>/dev/null || true
	@pkill -9 -f "teleop_rand_single_piper"     2>/dev/null || true
	@pkill -9 -f "start_sensor_gripper"         2>/dev/null || true
	@pkill -9 -f "pika_remote_piper"            2>/dev/null || true
	@tmux kill-session -t pika_ros              2>/dev/null || true
	@tmux kill-session -t pika_teleop           2>/dev/null || true
	@echo "[INFO] 所有节点已停止 ✅"

TOPIC_RECORD := /record_cmd
TOPIC_CTRL   := /arm/control_cmd

# ==============================================================================
#  迁移旧格式数据集 → lerobot v3 格式
# ==============================================================================

migrate:
	@echo "[INFO] 迁移数据集: $(SAVE_ROOT)"
	@bash -c "$(CONDA_SETUP) && \
	          cd $(REPO_DIR) && \
	          $(CONDA_PYTHON) migrate_dataset.py --root '$(SAVE_ROOT)'"

# 可视化验证（需要安装 lerobot + rerun）
# REPO_ID 用于标识，不需要是 HuggingFace 上的实际仓库
VIZ_REPO_ID ?= local/piper
VIZ_EPISODE ?= 0

viz:
	@echo "[INFO] 可视化 episode $(VIZ_EPISODE) from $(SAVE_ROOT)"
	@bash -c "$(CONDA_SETUP) && \
	          lerobot-dataset-viz \
	            --repo-id      '$(VIZ_REPO_ID)' \
	            --root         '$(SAVE_ROOT)' \
	            --episode-index $(VIZ_EPISODE) \
	            --tolerance-s  0.05"

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
	@if [ -f "$(SAVE_ROOT)/meta/tasks.parquet" ]; then \
		echo "--- 任务列表 ---"; \
		bash -c "$(CONDA_SETUP) && $(CONDA_PYTHON) -c \
		  \"import pandas as pd; print(pd.read_parquet('$(SAVE_ROOT)/meta/tasks.parquet').to_string())\""; \
	elif [ -f "$(SAVE_ROOT)/meta/tasks.jsonl" ]; then \
		echo "--- 任务列表 (旧格式) ---"; \
		cat "$(SAVE_ROOT)/meta/tasks.jsonl"; \
	fi
	@if [ -d "$(SAVE_ROOT)/meta/episodes" ]; then \
		echo "--- episodes parquet 目录 ---"; \
		ls "$(SAVE_ROOT)/meta/episodes/chunk-000/" 2>/dev/null || true; \
	elif [ -f "$(SAVE_ROOT)/meta/episodes.jsonl" ]; then \
		echo "--- episodes (最近 10 条, 旧格式) ---"; \
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
	@echo "  make control      键盘模式：tmux 一键启动所有进程"
	@echo "  make teleop       遥操作模式：自动启动全部节点"
	@echo "  make test-camera  测试鱼眼 + RealSense 相机"
	@echo "  make calibrate    运行 libsurvive 标定"
	@echo "  make piper        启动 piper_single_ctrl 控制节点"
	@echo "  make robot        启动机械臂控制器      (Process A, py310)"
	@echo "  make keyboard     启动键盘控制器         (Process B, 系统Python3)"
	@echo "  make record       启动数据采集器         (Process C, py310)"
	@echo "  make stop         停止所有节点"
	@echo "  make list         查看已录制的数据集信息"
	@echo "  make migrate      迁移旧格式数据集到 lerobot v3 格式"
	@echo "  make viz          可视化数据集（需要 rerun）"
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
	@echo "  make record  TASK='抓取红色方块' SAVE_ROOT=/data/dataset"
	@echo "  make control TASK='pick_cube'   SAVE_ROOT=/data/dataset"
	@echo "  make teleop  TASK='teleop_task' SAVE_ROOT=/data/dataset"
	@echo "  make list   SAVE_ROOT=/data/dataset"
	@echo ""
	@echo "遥操作采集流程:"
	@echo "  make teleop TASK='task_name' SAVE_ROOT=/data/dataset"
	@echo "  (自动启动 CAN/传感器/FK-IK/采集/键盘)"
	@echo ""

# ==============================================================================
#  内部工具
# ==============================================================================

check-tmux:
	@which tmux > /dev/null 2>&1 || \
		(echo "[ERROR] 需要 tmux: sudo apt install tmux" && exit 1)
