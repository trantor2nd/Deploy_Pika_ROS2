# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Humble-based system for keyboard-controlled Piper 6-DOF robot arm with Pika gripper, plus LeRobot v3 format data collection. Three main processes communicate via ROS2 topics, with a fourth external node (`piper_single_ctrl`) handling CAN bus.

## Architecture

Four processes, all communicating via ROS2 topics:

- **Process 0** (`piper_single_ctrl`): External ROS2 node from `/home/data/Project/piper_ros/`. Drives the Piper arm over CAN bus. Not our code.
- **Process A** (`robot_controller.py`): Bridges keyboard commands to the arm (forwards joint commands to piper_single_ctrl) and directly controls the Pika gripper via serial. Runs in py310 conda + ROS2.
- **Process B** (`keyboard_controller.py`): Reads keyboard input in raw terminal mode, publishes joint/gripper/control/record commands. Runs on system Python3 + ROS2 (no conda needed).
- **Process C** (`data_recorder.py`): Subscribes to joint states and commands, captures camera frames, writes LeRobot v3 format (MP4 video + Parquet) in real time. Runs in py310 conda + ROS2.

All shared constants (topic names, joint limits, device paths, defaults) live in `config.py`. Process scripts import from it.

## Key Data Flow

### Keyboard mode (`make all`)
```
Process B (keyboard) --joint_cmd/gripper_cmd--> Process A (robot) --joint_ctrl--> Process 0 (piper CAN)
Process B --record_cmd--> Process C (recorder)
Process 0 --joint_states_single--> Process A, Process C
Process A --gripper_state--> Process C
Process B --joint_cmd/gripper_cmd--> Process C (as action data)
```

### Teleop mode (`make teleop`)

Teleop scripts (from `/home/data/Project/Deploy/teleoperate/`) must be started separately first. They launch `piper_single_ctrl` (with remapping), FK/IK nodes, and sensor nodes. Process C subscribes to teleop topics instead:
```
Teleop IK --> /joint_states_gripper --> Process C (as action data)
piper_single_ctrl --> /joint_states_single --> Process C (as observation.state)
Teleop sensors --> ROS2 Image topics --> Process C (camera frames)
Process B (keyboard, for o/p only) --record_cmd--> Process C
```

Do NOT run `make all` and teleop simultaneously — they conflict on piper_single_ctrl and camera devices.

`observation.state` = actual positions from hardware feedback. `action` = command positions (keyboard or teleop).

## Commands

```bash
# Keyboard mode: start everything in tmux (2x2 pane layout)
make all TASK="task_name" SAVE_ROOT=/path/to/dataset

# Teleop mode: start recorder + keyboard only (teleop scripts must be running first)
make teleop TASK="task_name" SAVE_ROOT=/path/to/dataset

# Start individual processes
make piper      # Process 0 (includes CAN init)
make robot      # Process A (py310 conda)
make keyboard   # Process B (system python3)
make record TASK="task_name" SAVE_ROOT=/path FPS=10  # Process C keyboard mode
make record-teleop TASK="task_name" SAVE_ROOT=/path   # Process C teleop mode

# Stop all
make stop

# View dataset stats
make list SAVE_ROOT=/path/to/dataset

# Migrate old JSONL-format datasets to v3 Parquet format
make migrate SAVE_ROOT=/path/to/dataset

# Visualize (requires lerobot + rerun)
make viz SAVE_ROOT=/path VIZ_EPISODE=0
```

## Environment Details

- **py310 conda** (`/home/hsb/miniforge3/envs/py310/`): Used by Process A and C. Has `pika` SDK (gripper + cameras), `pyarrow`, `opencv-python`.
- **System Python3**: Used by Process B (keyboard). Only needs ROS2 packages.
- **ROS2 setup**: `source /opt/ros/humble/setup.bash && source /home/data/Project/piper_ros/install/setup.bash`
- **Hardware**: CAN bus (`can0`), gripper serial (`/dev/ttyUSB0`), FisheyeCamera (device_id=6), RealSense (SN: 230322275684)

## Rules

1. **Never modify files under `/home/data/Project/Deploy/`** (reference code, read-only).
2. All project code must stay within `/home/data/Project/Deploy_Pika_ROS2/`.
3. The Makefile must support `TASK` and `SAVE_ROOT` overrides for quick task/path changes.

## LeRobot v3 Format Notes

Data is written directly during recording (no post-processing conversion needed):
- `meta/info.json`, `meta/tasks.parquet`, `meta/episodes/chunk-000/file-000.parquet`
- `data/chunk-{n:03d}/episode_{ep:06d}.parquet` (state + action + timestamps)
- `videos/observation.images.{cam}/chunk-{n:03d}/episode_{ep:06d}.mp4`

Critical: video path structure is `videos/{cam_key}/chunk-{n}/` (cam before chunk). Timestamps use `frame_count / fps` (frame-aligned) to stay within lerobot's video tolerance.
