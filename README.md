# Ros2_Mocap_Motion

[![CN](https://img.shields.io/badge/语言-中文-red.svg)](README_CN.md)
[![EN](https://img.shields.io/badge/Language-English-blue.svg)](README.md)

A ROS2 + Gemini VLM wheeled robot visual path planning system. The robot perceives the environment through a depth camera, uses the Gemini multimodal large model for obstacle-avoidance path planning, and tracks paths with Pure Pursuit + PID control.

---

## Table of Contents

- [System Architecture](#system-architecture)
- [Dependencies](#dependencies)
- [Build Workspace](#build-workspace)
- [Configure Gemini API](#configure-gemini-api)
- [Launch](#launch)
  - [Mode A: Manual Path Planning (Step 1-2)](#mode-a-manual-path-planning-step-1-2)
  - [Mode B: VLM Autonomous Obstacle Avoidance (Step 3-6)](#mode-b-vlm-autonomous-obstacle-avoidance-step-3-6)
- [Manual LLM Trigger](#manual-llm-trigger)
- [ROS2 Nodes](#ros2-nodes)
- [Debug Commands](#debug-commands)

---

## System Architecture

```
Depth Camera RGB ──→ image_to_llm_node ──→ Gemini API
                                               │
                                         /llm_pixels (normalized pixel coordinates JSON)
                                               │
Depth Camera Depth ──→ image_conversion ──→ Pinhole reprojection + odom transform
                                               │
                                            /path (nav_msgs/Path)
                                               │
                                          track_path ──→ /cmd_vel (velocity commands)
                                               │
                                  Position feedback (Odom / Mocap / GPS)
```

---

## Dependencies

Use `install_dependencies.sh` to install all dependencies in one command (ROS2 Humble, Python packages, rosdep, etc.):

```bash
source install_dependencies.sh
```

Key dependencies:
- ROS2 Humble (Ubuntu 22.04)
- Python3, numpy, Pillow, opencv-python
- google-genai, python-dotenv, PyYAML
- cv_bridge, std_srvs

---

## Build Workspace

```bash
# 1. Source ROS2 environment
source /opt/ros/humble/setup.bash

# 2. Build (first time or after code changes)
cd ~/humble_ws
colcon build --symlink-install

# 3. Source workspace
source install/setup.bash
```

> **Tip:** You need to run `source /opt/ros/humble/setup.bash` and `source ~/humble_ws/install/setup.bash` in every new terminal, or add them to `~/.bashrc`.

---

## Configure Gemini API

`image_to_llm_node` reads API keys from `src/image_to_llm/llm_config.env`. This file is in `.gitignore` and will not be committed.

**Create config file:**

```bash
nano src/image_to_llm/llm_config.env
```

**Example content:**

```env
GEMINI_API_KEY=your_api_key_here
GEMINI_MODEL=gemini-2.5-flash

# System proxy (if needed to access Gemini API)
HTTP_PROXY=http://127.0.0.1:7890
HTTPS_PROXY=http://127.0.0.1:7890
```

> **Note:** Prompts have been migrated to Skill YAML files (`src/image_to_llm/image_to_llm/skills/default.yaml`) and are no longer configured in env. If you don't need a proxy, you can omit the `HTTP_PROXY` / `HTTPS_PROXY` lines.

### Skill System

System prompts are managed via Skill YAML files with hot-swap support:

```bash
# Use default Skill
ros2 launch image_to_llm llm_node_launch.py

# Specify a Skill (corresponds to YAML filename in skills/)
ros2 launch image_to_llm llm_node_launch.py skill_name:=default
```

Skill files are located in `src/image_to_llm/image_to_llm/skills/`. Adding a new Skill only requires adding a YAML file — zero code changes.

---

## Launch

### Prerequisites

Ensure the following are met:
1. Workspace is built and sourced
2. Robot hardware is running (providing camera topics, odom topic, etc.)
3. If using VLM mode, `llm_config.env` is properly configured

### Mode A: Manual Path Planning (Step 1-2)

This mode does not require a camera or Gemini — paths are generated from manually entered coordinates.

**Terminal 1 — Launch path generation node:**
```bash
ros2 run my_robot_planner generate_path
```
After launch, enter target coordinates and shape in the terminal, e.g.:
```
1.0 3.0 semicircle
```
Supported shapes: `straight`, `semicircle`, `s_shape`, `polynomial`

**Terminal 2 — Launch path tracking node:**
```bash
ros2 run my_robot_planner track_path
```

> `track_path` automatically subscribes to `/path` and begins tracking, publishing velocity commands to `/cmd_vel` and `/agent0/cmd_vel` at 10Hz.

---

### Mode B: VLM Autonomous Obstacle Avoidance (Step 3-6)

Full end-to-end closed loop: `track_path` automatically triggers Gemini → receives pixel paths → depth reprojection to 3D → tracks path → arrival/obstacle → triggers again.

#### Launch (Recommended, two terminals)

**Terminal 1 — Gemini API node** (LLM interaction logs displayed independently):
```bash
ros2 launch image_to_llm llm_node_launch.py
```

**Terminal 2 — Coordinate conversion + path tracking** (downstream processing pipeline):
```bash
ros2 launch my_robot_planner robot_vlm_launch.py
```

> Logs from the two terminals do not interfere with each other, making it easy to debug LLM communication and path tracking separately. Nodes have built-in readiness waiting — they cooperate correctly regardless of launch order.

### Node Readiness Logic

Each node has built-in intelligent waiting, ensuring correct collaboration regardless of launch order:

| Node | Wait Condition | Ready Signal |
|------|---------------|-------------|
| `image_to_llm_node` | First RGB frame received | ✅ Service ready log |
| `image_conversion` | camera_info + depth image + odom all received | ✅ Log indicates ready; caches pixel messages until ready |
| `track_path` | `/path` received + odom position | ✅ Begins PID tracking |

### Automatic Trigger Conditions

`track_path` automatically requests new paths from Gemini when:
1. **Path completion trigger:** Arrival at endpoint (distance < 0.15m) triggers next segment request
2. **Obstacle trigger:** Depth map detects obstacle < 0.8m ahead, sustained for 3 seconds → triggers Gemini replanning

---

## Manual LLM Trigger

To manually trigger a Gemini path planning request (e.g., for debugging):

```bash
ros2 service call /trigger_llm_plan std_srvs/srv/Trigger
```

You can also manually publish test pixel points (skipping the Gemini API to directly test image_conversion + track_path):

```bash
ros2 topic pub --once /llm_pixels std_msgs/msg/String "{data: '[{\"point\": [500, 800]}, {\"point\": [500, 700]}, {\"point\": [500, 600]}, {\"point\": [480, 500]}, {\"point\": [460, 400]}]'}"
```

---

## ROS2 Nodes

| Node | Package | Launch Command | Function |
|------|---------|---------------|----------|
| `generate_path` | `my_robot_planner` | `ros2 run my_robot_planner generate_path` | Generate path from coordinates and shape, publish to `/path` (manual mode) |
| `track_path` | `my_robot_planner` | `ros2 run my_robot_planner track_path` | Pure Pursuit + PID path tracking + depth-based obstacle detection |
| `image_to_llm_node` | `image_to_llm` | `ros2 run image_to_llm image_to_llm_node` | Send RGB images to Gemini, receive pixel path points |
| `image_conversion` | `image_to_llm` | `ros2 run image_to_llm image_conversion` | Pixel coordinates → 3D odom coordinates (depth reprojection), publish `/path` |

---

## Debug Commands

```bash
# List all active topics
ros2 topic list

# Echo path points
ros2 topic echo /path

# Echo LLM pixel coordinates
ros2 topic echo /llm_pixels

# Echo velocity commands
ros2 topic echo /cmd_vel

# List nodes
ros2 node list

# List services (confirm /trigger_llm_plan is available)
ros2 service list

# Manually trigger LLM planning
ros2 service call /trigger_llm_plan std_srvs/srv/Trigger
```

---

## Key ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Depth camera RGB image |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth image |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | Depth camera intrinsics |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RGB camera intrinsics |
| `/llm_pixels` | `std_msgs/String` | Normalized pixel coordinates JSON from Gemini |
| `/path` | `nav_msgs/Path` | 3D path points (odom frame) |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/agent0/cmd_vel` | `geometry_msgs/Twist` | Webots robot velocity commands |
| `/odom` | `nav_msgs/Odometry` | Odometry position + orientation |
| `/vrpn_mocap/rm_0_Test/pose` | `geometry_msgs/PoseStamped` | Mocap position (optional) |

> **Note:** Camera topic names depend on the depth camera model. The table above shows the current Intel D435 topic names — they may change when switching to an Orbbec Gemini 336. Topic parameters are configured in launch files; modify the launch file to adapt to different cameras.

---

## Directory Structure

```
humble_ws/
├── ARCHITECTURE.md          # System architecture design doc
├── PROCESS.md               # Development steps & progress
├── MEMORY.md                # Change log
├── README.md                # This file (English)
├── README_CN.md             # Chinese version
├── install_dependencies.sh  # One-click dependency install script
├── data/
│   ├── image/               # Runtime captured images
│   └── log/                 # Coordinate conversion debug logs
├── future/                  # Future architecture design docs
│   ├── function_calling_design.md
│   ├── skill_progressive_cognition.md
│   └── agentic_workflow_vision.md
└── src/
    ├── my_robot_msgs/       # Custom service messages (GeneratePath.srv)
    ├── my_robot_planner/    # Path generation + path tracking
    │   ├── launch/
    │   │   └── robot_vlm_launch.py  # Launch conversion + track_path
    │   └── my_robot_planner/
    │       ├── generate_path.py
    │       └── track_path.py
    └── image_to_llm/        # Gemini API interaction + pixel→3D conversion
        ├── launch/
        │   └── llm_node_launch.py   # Launch image_to_llm_node
        ├── llm_config.env   # API config (not in git)
        └── image_to_llm/
            ├── image_to_llm_node.py
            ├── image_conversion.py
            └── skills/      # Skill YAML hot-swap framework
                ├── __init__.py   # Skill loader
                └── default.yaml  # Default path planning Skill
```
