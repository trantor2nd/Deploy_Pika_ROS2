# Pika ROS2 Piper 控制与数据采集系统

基于 ROS2 Humble 的三进程机械臂键盘控制与 LeRobot v3 格式数据采集系统，适用于 **Piper 6-DOF 机械臂 + Pika 夹爪**组合。

---

## 目录

- [系统架构](#系统架构)
- [硬件要求](#硬件要求)
- [环境依赖](#环境依赖)
- [安装步骤](#安装步骤)
- [配置说明](#配置说明)
- [快速上手](#快速上手)
- [Makefile 命令参考](#makefile-命令参考)
- [键盘控制说明](#键盘控制说明)
- [数据集格式](#数据集格式)
- [ROS2 话题一览](#ros2-话题一览)
- [常见问题](#常见问题)

---

## 系统架构

系统由 **4 个进程**协同工作，通过 ROS2 话题通信：

```
┌────────────────────────────────────────────────────────────────┐
│  Process 0: piper_single_ctrl  (官方 ROS2 节点)                │
│  职责: CAN 总线通信，驱动 Piper 机械臂底层关节控制             │
└─────────────────────────┬──────────────────────────────────────┘
                          │ /joint_states_single  /joint_ctrl_single
                          ▼
┌────────────────────────────────────────────────────────────────┐
│  Process A: robot_controller.py  (py310 conda + ROS2)          │
│  职责: 转发关节指令、控制 Pika 夹爪（串口）、发布夹爪状态      │
└─────┬──────────────────────┬─────────────────────────────────┘
      │                      │ /arm/joint_cmd  /arm/control_cmd
      │ /gripper_state       │ /arm/gripper_cmd
      ▼                      ▼
┌─────────────────┐   ┌──────────────────────────────────────────┐
│  Process C      │◀──│  Process B: keyboard_controller.py       │
│  data_recorder  │   │  职责: 键盘输入 → 发布控制/采集信号      │
│  .py            │   │  运行环境: 系统 Python3 + ROS2           │
│  (py310 + ROS2) │   └──────────────────────────────────────────┘
│  职责: 订阅关节 │     /record_cmd
│  /夹爪状态，直写│
│  LeRobot v3 格式│
└─────────────────┘
```

| 进程 | 文件 | 运行环境 | 核心职责 |
|------|------|----------|----------|
| 0 (piper) | `piper_single_ctrl` | py310 + ROS2 | CAN 总线底层驱动 |
| A (robot) | `robot_controller.py` | py310 conda + ROS2 | 机械臂 + 夹爪控制 |
| B (keyboard) | `keyboard_controller.py` | 系统 Python3 + ROS2 | 键盘输入/信号发布 |
| C (recorder) | `data_recorder.py` | py310 conda + ROS2 | 数据采集与保存 |

---

## 硬件要求

| 设备 | 型号 / 规格 | 接口 |
|------|-------------|------|
| 机械臂 | AgileX Piper 6-DOF | CAN 总线 (`can0`) |
| 末端夹爪 | Pika Gripper | 串口 (`/dev/ttyUSB0`) |
| 鱼眼相机 | 通用 USB 鱼眼 640×480 | USB (`device_id=6`) |
| 深度相机 | Intel RealSense (SN: 230322275684) | USB |
| 主机 | Ubuntu 22.04 + CAN 接口卡 | — |

> 相机为可选设备，缺失时系统仍可运行，仅记录关节/夹爪状态数据。

---

## 环境依赖

### 基础系统要求

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **tmux**: 用于一键启动多进程（`sudo apt install tmux`）
- **Python**: 系统 Python 3.10+（Process B 使用）

### 1. 安装 ROS2 Humble

```bash
# 测试安装
source /opt/ros/humble/setup.bash
ros2 --version
```

### 2. 配置 CAN 总线

```bash
# 安装 CAN 工具
sudo apt install can-utils

# 加载 CAN 内核模块
sudo modprobe can
sudo modprobe can_raw
sudo modprobe gs_usb   # USB-CAN 适配器驱动

# 手动初始化（Makefile 会通过 can_activate.sh 自动执行）
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 验证
ip link show can0   # 状态应为 UP
candump can0        # 应有机械臂数据流
```

### 3. 创建 py310 Conda 环境

```bash
# 安装 miniforge（若尚未安装）
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
bash Miniforge3-Linux-x86_64.sh -b -p ~/miniforge3
~/miniforge3/bin/conda init bash && source ~/.bashrc

# 创建 py310 环境
conda create -n py310 python=3.10 -y
conda activate py310

# 安装核心 Python 依赖
pip install pyarrow opencv-python numpy

# 安装 ROS2 Python 绑定（供 conda 环境内调用）
pip install catkin_pkg empy lark
```

### 4. 安装 Pika SDK

Pika SDK 提供夹爪（`pika.gripper.Gripper`）和相机（`pika.camera.fisheye.FisheyeCamera`、`pika.camera.realsense.RealSenseCamera`）驱动。

```bash
conda activate py310

# 方式一：从 wheel 文件安装（询问设备提供方获取）
pip install /path/to/pika_sdk-*.whl

# 方式二：从源码安装
cd /path/to/pika_sdk
pip install -e .

# 验证安装
python -c "from pika.gripper import Gripper; print('Pika SDK OK')"
```

### 5. 安装 Intel RealSense 驱动（可选）

```bash
# 添加 Intel 软件源
sudo apt-key adv --keyserver keyserver.ubuntu.com \
     --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCD
sudo add-apt-repository \
     "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev

conda activate py310
pip install pyrealsense2

# 验证
realsense-viewer   # 图形界面查看相机
```

### 6. 编译 piper_ros 工作空间

```bash
cd /home/data/Project/piper_ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 验证
source install/setup.bash
ros2 pkg list | grep piper   # 应显示 piper
```

---

## 安装步骤

```bash
# 1. 进入项目目录
cd /home/data/Project/Deploy_Pika_ROS2

# 2. 验证文件完整性
ls -la
# 应显示: config.py  data_recorder.py  keyboard_controller.py
#         robot_controller.py  Makefile  README.md

# 3. 设置串口权限（每次重启后需重新执行，或写入 udev rules）
sudo chmod 666 /dev/ttyUSB0

# 4. 添加用户到 dialout 组（一次性，永久生效）
sudo usermod -aG dialout $USER
# 需要重新登录后生效

# 5. 验证 Makefile 配置（检查默认路径是否与实际一致）
make help
```

---

## 配置说明

所有参数集中在 `config.py`，修改该文件即可调整默认值，**无需修改其他脚本**。

```python
# ===== 关节软件限位（单位：rad）=====
JOINT_LIMITS = [
    (-3.14,  3.14),   # j1 底座旋转
    ( 0.00,  2.00),   # j2 肩部俯仰
    (-2.00,  0.00),   # j3 大臂俯仰
    (-1.50,  1.80),   # j4 肘部旋转
    (-1.30,  1.57),   # j5 腕部俯仰
    (-3.14,  3.14),   # j6 末端旋转
]
HOME_POS   = [0.0, -0.035, 0.0, 0.0, 0.35, 0.0]  # 回零目标位置（rad）
JOINT_STEP = 0.025   # 每次按键步进量（rad）

# ===== 夹爪 =====
GRIPPER_PORT = "/dev/ttyUSB0"
GRIPPER_MIN  = 0.0    # 完全闭合（mm）
GRIPPER_MAX  = 90.0   # 完全张开（mm）
GRIPPER_STEP = 10.0   # 每次按键步进量（mm）

# ===== 鱼眼相机（USB）=====
FISHEYE_INDEX  = 6    # USB 设备 ID（ls /dev/video* 查看）
FISHEYE_WIDTH  = 640
FISHEYE_HEIGHT = 480
FISHEYE_FPS    = 30

# ===== RealSense 相机 =====
REALSENSE_SN     = "230322275684"  # 序列号（留空则自动匹配第一个）
REALSENSE_WIDTH  = 640
REALSENSE_HEIGHT = 480
REALSENSE_FPS    = 30

# ===== 数据采集 =====
CAPTURE_HZ        = 10.0
DEFAULT_SAVE_ROOT = "/home/data/Dataset/piper_lerobot_dataset"
DEFAULT_TASK      = "default_task"
EPISODES_PER_CHUNK = 1000  # 每个 chunk 存放的最大 episode 数
```

**Makefile 可在命令行覆盖的变量：**

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `CONDA_ENV` | `py310` | Conda 环境名 |
| `PIPER_WS` | `/home/data/Project/piper_ros` | piper_ros 工作空间路径 |
| `CAN_PORT` | `can0` | CAN 总线接口名 |
| `USB_PORT` | `/dev/ttyUSB0` | 夹爪串口设备路径 |
| `SAVE_ROOT` | `/home/data/Dataset/piper_lerobot_dataset` | 数据集保存目录 |
| `TASK` | `default_task` | 任务指令文本（写入数据集元信息） |
| `FPS` | `10` | 数据采集帧率（Hz） |

---

## 快速上手

### 一键启动（推荐方式）

```bash
# 默认配置启动
make all

# 指定任务名称和保存路径
make all TASK="抓取红色方块" SAVE_ROOT=/data/my_dataset

# 指定采集帧率
make all TASK="pick_cube" SAVE_ROOT=/data/dataset FPS=20
```

启动后会创建名为 `pika_ros` 的 tmux 会话，包含 4 个窗口：

| 窗口名 | 进程 | 说明 |
|--------|------|------|
| `piper` | Process 0 | piper_single_ctrl 底层 CAN 控制节点 |
| `robot` | Process A | 机械臂 + 夹爪控制器（py310） |
| `recorder` | Process C | 数据采集器（py310） |
| `keyboard` | Process B | 键盘控制器（系统 Python3） |

**连接到 tmux 会话：**

```bash
tmux attach -t pika_ros

# tmux 常用快捷键
# Ctrl+B, 数字   切换到指定窗口（0=piper, 1=robot, 2=recorder, 3=keyboard）
# Ctrl+B, D      退出 tmux（不关闭进程）
# Ctrl+B, [      进入滚动模式（q 退出）
```

> **重要**：键盘控制需要在 **keyboard** 窗口（编号 3）中操作，确保 terminal 焦点在该窗口。

### 停止所有进程

```bash
# 方式一（推荐）：在 keyboard 窗口按 空格，安全退出（机械臂先回零再失能）
# 方式二：强制停止所有进程
make stop
```

### 数据采集流程

1. `make all` 启动所有进程
2. `tmux attach -t pika_ros`，切换到 keyboard 窗口（`Ctrl+B 3`）
3. 使用关节键（`a/d/w/s/u/j/r/f/t/g/e/q`）和夹爪键（`h/k`）控制机械臂到初始位置
4. 按 `o` 开始录制当前 episode
5. 操作机械臂完成任务演示
6. 按 `p` 停止录制并保存（数据自动写入磁盘）
7. 重复步骤 3-6 录制更多 episode
8. 按 `空格` 安全退出

### 查看已录制数据

```bash
make list

# 查看指定路径
make list SAVE_ROOT=/data/my_dataset
```

输出示例：
```
============================================
数据集路径: /home/data/Dataset/piper_lerobot_dataset
============================================
--- info.json ---
  总 episodes: 42
  总帧数    : 1260
  总任务数  : 2
  FPS       : 10.0
--- 任务列表 ---
{"task_index": 0, "task": "pick_cube"}
{"task_index": 1, "task": "place_cube"}
```

---

## Makefile 命令参考

```bash
make help        # 显示完整帮助信息

make all         # tmux 一键启动所有进程
make piper       # 单独启动 piper_single_ctrl 底层节点（含 CAN 初始化）
make robot       # 单独启动 Process A（机械臂控制器，py310）
make keyboard    # 单独启动 Process B（键盘控制器，系统 Python3）
make record      # 单独启动 Process C（数据采集器，py310）
make setup-can   # 仅初始化 CAN 总线
make stop        # 停止所有节点并关闭 tmux 会话
make list        # 查看数据集统计信息
```

**带参数示例：**

```bash
# 启动时指定任务和路径
make all TASK="放置蓝色积木" SAVE_ROOT=/mnt/ssd/dataset FPS=20

# 单独启动采集器（适合已有 robot 在运行时）
make record TASK="grasp_apple" SAVE_ROOT=~/data/apple FPS=15

# 查看非默认路径的数据集
make list SAVE_ROOT=~/data/apple

# 使用自定义 CAN 接口和 conda 环境
make all CONDA_ENV=myenv PIPER_WS=/opt/piper_ws CAN_PORT=can1
```

---

## 键盘控制说明

> 需要在 `keyboard` 窗口（前台运行）操作，terminal 焦点必须在该窗口。

### 关节控制（12 键，增/减各一键）

| 关节 | 含义 | 增大键 | 减小键 | 单次步进 |
|------|------|--------|--------|----------|
| j1 | 底座旋转 | `a` | `d` | 0.0125 rad |
| j2 | 肩部俯仰 | `w` | `s` | 0.020 rad |
| j3 | 大臂俯仰 | `u` | `j` | 0.020 rad |
| j4 | 肘部旋转 | `r` | `f` | 0.025 rad |
| j5 | 腕部俯仰 | `t` | `g` | 0.025 rad |
| j6 | 末端旋转 | `e` | `q` | 0.025 rad |

### 夹爪控制（2 键）

| 操作 | 按键 | 步进 | 范围 |
|------|------|------|------|
| 张开 | `h` | +10 mm | 0 ~ 90 mm |
| 闭合 | `k` | -10 mm | 0 ~ 90 mm |

### 系统控制

| 操作 | 按键 | 说明 |
|------|------|------|
| 回 HOME | `z` | 插值平滑运动回零位（耗时约 3 秒） |
| 使能 | `n` | 机械臂和夹爪上电使能 |
| 失能 | `m` | 断电失能（机械臂将受重力影响下落，**谨慎使用**） |
| 开始采集 | `o` | 通知 Process C 创建新 episode 并开始录制 |
| 停止采集 | `p` | 通知 Process C 结束录制并保存到磁盘 |
| **安全退出** | `空格` | 依次执行：停止采集 → 回 HOME → 失能 → 退出所有进程 |

> **安全退出说明**：按下空格后请等待约 6 秒，系统会等待机械臂完成回零动作后再失能，请勿提前断电。

---

## 数据集格式

数据采用 **LeRobot v3 格式**直接写入磁盘，录制过程中实时编码 MP4，**无需任何后处理转换步骤**。

### 目录结构

```
<SAVE_ROOT>/
├── meta/
│   ├── info.json           # 数据集元信息（总 episode 数、帧数、FPS、features 定义等）
│   ├── episodes.jsonl      # 每条 episode 的元信息（逐行 JSON，可追加）
│   └── tasks.jsonl         # 任务索引表（task_text → task_index）
├── data/
│   └── chunk-000/          # 每 1000 条 episode 一个 chunk
│       ├── episode_000000.parquet
│       ├── episode_000001.parquet
│       └── ...
└── videos/
    └── chunk-000/
        ├── observation.images.fisheye_rgb/
        │   ├── episode_000000.mp4
        │   └── ...
        └── observation.images.realsense_rgb/
            ├── episode_000000.mp4
            └── ...
```

### Parquet 文件字段定义

每个 `episode_XXXXXX.parquet` 文件包含该 episode 的所有帧数据：

| 字段名 | 类型 | 维度 | 说明 |
|--------|------|------|------|
| `observation.state` | `float32` | `[7]` | 关节角 j1~j6（rad）+ 夹爪距离（mm） |
| `action` | `float32` | `[7]` | 与 state 相同（遥操作场景） |
| `timestamp` | `float32` | scalar | 相对 episode 起始时间（秒） |
| `frame_index` | `int64` | scalar | episode 内帧序号（从 0 开始） |
| `episode_index` | `int64` | scalar | 全局 episode 编号 |
| `index` | `int64` | scalar | 全局帧序号 |
| `task_index` | `int64` | scalar | 对应 tasks.jsonl 中的任务索引 |

### 视频文件规格

| 属性 | 值 |
|------|-----|
| 编码格式 | mp4v (MPEG-4 Part 2) |
| 分辨率 | 640 × 480 |
| 帧率 | 与 `--fps` 一致（默认 10 Hz） |
| 色彩空间 | BGR（OpenCV 默认） |

### 追加录制说明

系统支持向已有数据集追加录制，不会覆盖已有数据。每次启动 Process C 时，会自动读取 `meta/episodes.jsonl` 获取当前最大 episode 编号，从下一个编号开始写入。

---

## ROS2 话题一览

| 话题名 | 消息类型 | 数据流向 | 说明 |
|--------|----------|----------|------|
| `/arm/joint_cmd` | `sensor_msgs/JointState` | B → A | 目标关节角度（rad） |
| `/arm/gripper_cmd` | `std_msgs/Float64` | B → A | 目标夹爪距离（mm） |
| `/arm/control_cmd` | `std_msgs/String` | B → A | 指令：`home` / `enable` / `disable` / `exit` |
| `/record_cmd` | `std_msgs/String` | B → C | 指令：`start` / `stop` / `exit` |
| `/joint_states_single` | `sensor_msgs/JointState` | 0 → all | 实际关节角度反馈（rad） |
| `/gripper_state` | `std_msgs/Float64` | A → C | 实际夹爪距离（mm） |
| `/joint_ctrl_single` | `sensor_msgs/JointState` | A → 0 | 关节控制指令透传 |
| `/enable_flag` | `std_msgs/Bool` | A → 0 | 使能 / 失能信号 |

**手动发布话题（调试用）：**

```bash
source /opt/ros/humble/setup.bash
source /home/data/Project/piper_ros/install/setup.bash

# 手动使能
ros2 topic pub --once /arm/control_cmd std_msgs/msg/String "data: 'enable'"

# 手动回零
ros2 topic pub --once /arm/control_cmd std_msgs/msg/String "data: 'home'"

# 手动开始/停止采集
ros2 topic pub --once /record_cmd std_msgs/msg/String "data: 'start'"
ros2 topic pub --once /record_cmd std_msgs/msg/String "data: 'stop'"

# 查看实际关节角度
ros2 topic echo /joint_states_single

# 查看所有活跃话题
ros2 topic list
```

---

## 常见问题

### 机械臂无响应

1. 检查 CAN 总线状态：`ip link show can0`（状态应为 `UP`，`LOWER_UP`）
2. 查看 piper 窗口日志：`tmux attach -t pika_ros`，切换窗口 `Ctrl+B 0`
3. 监听 CAN 数据流：`candump can0`（应有连续数据）
4. 尝试手动重新激活 CAN：`make setup-can`

### 夹爪无法连接

1. 确认设备存在：`ls /dev/ttyUSB*`
2. 检查权限：`sudo chmod 666 /dev/ttyUSB0`，或将用户加入 dialout 组
3. 确认 `config.py` 中 `GRIPPER_PORT` 与实际设备路径一致
4. 夹爪连接失败时系统进入"无夹爪模式"，关节控制仍然正常

### 相机无法打开

1. 鱼眼相机：`ls /dev/video*` 查看可用设备 ID，修改 `config.py` 中 `FISHEYE_INDEX`
2. RealSense：运行 `realsense-viewer` 确认序列号，修改 `REALSENSE_SN`（也可设为空字符串 `""` 自动匹配）
3. 相机缺失时系统仍可录制，只是无视频文件输出

### 键盘按键无效

- 确认 terminal 焦点在 `keyboard` 窗口（tmux 中用 `Ctrl+B 3` 切换）
- Process B 必须在前台运行，不能在后台或通过 `nohup` 启动

### Python 模块找不到

```bash
# 检查 pika SDK
conda activate py310
python -c "from pika.gripper import Gripper"

# 检查 pyarrow
python -c "import pyarrow; print(pyarrow.__version__)"

# 缺少时安装
pip install pyarrow opencv-python
```

### 数据集文件损坏（录制中断）

若录制中途异常中断，最后一个 episode 的 MP4 可能不完整，但 Parquet 和元数据通常已安全写入。处理方法：

```bash
# 查看最后一个 episode 的帧数
tail -n 1 $SAVE_ROOT/meta/episodes.jsonl

# 删除损坏的最后一个 episode（手动清理）
# 然后重新编辑 episodes.jsonl 删除最后一行即可
```

### make stop 后进程仍存在

```bash
# 手动强制终止
pkill -9 -f robot_controller.py
pkill -9 -f keyboard_controller.py
pkill -9 -f data_recorder.py
pkill -9 -f piper_single_ctrl
tmux kill-session -t pika_ros
```

---

## 项目文件说明

```
Deploy_Pika_ROS2/
├── config.py              # 共享配置常量（关节限位、话题名、设备路径、默认参数）
├── robot_controller.py    # Process A：机械臂 + Pika 夹爪控制器
├── keyboard_controller.py # Process B：键盘交互控制器
├── data_recorder.py       # Process C：LeRobot v3 格式数据采集器
├── Makefile               # 快捷启动脚本
└── README.md              # 本文档
```
