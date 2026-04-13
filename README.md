# Pika ROS2 Piper 数据采集系统

ROS2 Humble 环境下的 Piper 6-DOF 机械臂 + Pika 夹爪控制与数据采集系统。支持键盘直控和遥操作两种模式，实时录制 LeRobot v3 格式数据集（MP4 视频 + Parquet 数据）。

## 快速开始

### 键盘模式（直接关节控制）

```bash
make control TASK="pick_cube" SAVE_ROOT=/path/to/dataset
```

在 tmux 2x2 分屏中自动启动 piper 驱动、机械臂控制器、数据采集器、键盘控制器。

### 遥操作模式

```bash
make teleop TASK="teleop_task" SAVE_ROOT=/path/to/dataset
```

在 tmux 2x2 分屏中自动启动 CAN/传感器、FK/IK/遥操作、数据采集器、键盘控制器。

> 键盘模式和遥操作模式不可同时运行。

### 测试相机

```bash
make test-camera
```

实时预览鱼眼 + RealSense 画面，按 `q` 退出，按 `s` 保存截图。

### 标定

```bash
make calibrate
```

运行 libsurvive VR 追踪设备标定。

### 停止所有

```bash
make stop
```

## 键盘控制

| 按键 | 功能 |
|------|------|
| a / d | 关节 1 +/- |
| w / s | 关节 2 +/- |
| u / j | 关节 3 +/- |
| r / f | 关节 4 +/- |
| t / g | 关节 5 +/- |
| e / q | 关节 6 +/- |
| h / k | 夹爪 张开/闭合 |
| **o** | **开始录制** |
| **p** | **停止录制** |
| **l** | **丢弃 episode** |
| z | 回 HOME 位置 |
| n / m | 使能/失能 |
| 空格 | 安全退出 |

### 丢弃功能（L 键）

- 录制中按 L：停止当前录制并丢弃该 episode（删除视频文件）
- 录制结束后按 L：丢弃上一条已完成的 episode（删除 parquet + 视频文件，重写 metadata）

## 数据管理

```bash
# 查看数据集统计
make list
make list SAVE_ROOT=/other/path

# 迁移旧 JSONL 格式到 v3
make migrate SAVE_ROOT=/path/to/dataset

# 可视化某个 episode（需要 lerobot + rerun）
make viz SAVE_ROOT=/path/to/dataset VIZ_EPISODE=0
```

## 可配置变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| TASK | default_task | 任务描述文本 |
| SAVE_ROOT | /home/data/Dataset/piper_lerobot_dataset | 数据集保存路径 |
| FPS | 10 | 采集帧率 (Hz) |
| CAN_PORT | can0 | CAN 总线接口 |
| CONDA_ENV | py310 | Conda 环境名 |
| PIPER_WS | /home/data/Project/piper_ros | piper_ros 工作空间 |

示例：`make control TASK="抓取红色方块" SAVE_ROOT=/data/exp1 FPS=20`

## 遥操作模式详情

`make teleop` 创建 4 窗格 tmux 会话，自动依次启动：

| 窗格 | 内容 | 延迟 |
|------|------|------|
| 0 | CAN 初始化 + 传感器/夹爪节点 | 立即 |
| 1 | FK/IK + 遥操作启动 | 10s |
| 2 | 数据采集器（teleop 模式） | 15s |
| 3 | 键盘控制器（o/p/l 控制采集） | 20s |

## 所有 Makefile 目标

| 目标 | 说明 |
|------|------|
| `make control` | 键盘模式：tmux 4 窗格启动全部进程 |
| `make teleop` | 遥操作模式：tmux 4 窗格自动启动全部 |
| `make test-camera` | 测试鱼眼 + RealSense 相机 |
| `make calibrate` | 运行 libsurvive 标定 |
| `make piper` | 单独启动 piper CAN 驱动 |
| `make robot` | 单独启动机械臂控制器 |
| `make keyboard` | 单独启动键盘控制器 |
| `make record` | 单独启动数据采集器（键盘模式） |
| `make record-teleop` | 单独启动数据采集器（遥操作模式） |
| `make stop` | 停止所有进程 |
| `make list` | 查看数据集信息 |
| `make migrate` | 迁移旧格式数据到 v3 |
| `make viz` | 可视化 episode |
| `make help` | 显示帮助信息 |
