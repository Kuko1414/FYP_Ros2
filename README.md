# Ros2_Mocap_Motion

基于 ROS2 + Gemini VLM 的轮式机器人视觉路径规划系统。机器人通过深度相机感知环境，利用 Gemini 多模态大模型进行避障路径规划，并使用 Pure Pursuit + PID 控制算法跟踪路径。

---

## 目录

- [系统架构](#系统架构)
- [依赖安装](#依赖安装)
- [编译工作区](#编译工作区)
- [配置 Gemini API](#配置-gemini-api)
- [启动流程](#启动流程)
  - [模式 A：手动路径规划（Step 1-2）](#模式-a手动路径规划step-1-2)
  - [模式 B：VLM 自动避障闭环（Step 3-6）](#模式-bvlm-自动避障闭环step-3-6)
- [手动触发 LLM 规划](#手动触发-llm-规划)
- [ROS2 节点说明](#ros2-节点说明)
- [常用调试命令](#常用调试命令)

---

## 系统架构

```
深度相机 RGB ──→ image_to_llm_node ──→ Gemini API
                                           │
                                     /llm_pixels (归一化像素坐标 JSON)
                                           │
深度相机 Depth ──→ image_conversion ──→ 针孔模型反投影 + odom 变换
                                           │
                                        /path (nav_msgs/Path)
                                           │
                                      track_path ──→ /cmd_vel (速度指令)
                                           │
                              位置反馈 (Odom / Mocap / GPS)
```

---

## 依赖安装

使用 `install_dependencies.sh` 一键安装所有依赖（ROS2 Humble、Python 包、rosdep 等）：

```bash
source install_dependencies.sh
```

主要依赖：
- ROS2 Humble (Ubuntu 22.04)
- Python3, numpy, Pillow, opencv-python
- google-genai, python-dotenv, PyYAML
- cv_bridge, std_srvs

---

## 编译工作区

```bash
# 1. source ROS2 环境
source /opt/ros/humble/setup.bash

# 2. 编译（首次编译或代码修改后）
cd ~/humble_ws
colcon build --symlink-install

# 3. source 工作区
source install/setup.bash
```

> **提示**：每次打开新终端都需要执行 `source /opt/ros/humble/setup.bash` 和 `source ~/humble_ws/install/setup.bash`，或将它们添加到 `~/.bashrc`。

---

## 配置 Gemini API

`image_to_llm_node` 从配置文件 `src/image_to_llm/llm_config.env` 读取 API 密钥。此文件已在 `.gitignore` 中，不会被提交。

**创建配置文件**：

```bash
nano src/image_to_llm/llm_config.env
```

**文件内容示例**：

```env
GEMINI_API_KEY=你的API密钥
GEMINI_MODEL=gemini-2.5-flash

# 系统代理（如需翻墙访问 Gemini API）
HTTP_PROXY=http://127.0.0.1:7890
HTTPS_PROXY=http://127.0.0.1:7890
```

> **注意**：Prompt 已迁移到 Skill YAML 文件管理（`src/image_to_llm/image_to_llm/skills/default.yaml`），不再在 env 中配置。如果不需要代理，可以留空或删除 `HTTP_PROXY` / `HTTPS_PROXY` 行。

### Skill 系统

系统提示词通过 Skill YAML 文件管理，支持热切换：

```bash
# 使用默认 Skill
ros2 launch image_to_llm llm_node_launch.py

# 指定 Skill（对应 skills/ 目录下的 YAML 文件名）
ros2 launch image_to_llm llm_node_launch.py skill_name:=default
```

Skill 文件位于 `src/image_to_llm/image_to_llm/skills/`，新增 Skill 只需添加一个 YAML 文件，零代码改动。

---

## 启动流程

### 前提条件

确保以下条件满足：
1. 工作区已编译且已 source
2. 机器人硬件已启动（提供相机 topic、odom topic 等）
3. 如使用 VLM 模式，`llm_config.env` 已正确配置

### 模式 A：手动路径规划（Step 1-2）

此模式不需要相机和 Gemini，仅使用手动输入坐标生成路径并跟踪。

**终端 1 — 启动路径生成节点**：
```bash
ros2 run my_robot_planner generate_path
```
启动后在终端中输入目标坐标和形状，例如：
```
1.0 3.0 semicircle
```
支持的形状：`straight`、`semicircle`、`s_shape`、`polynomial`

**终端 2 — 启动路径跟踪节点**：
```bash
ros2 run my_robot_planner track_path
```

> `track_path` 会自动订阅 `/path` 并开始跟踪，同时以 10Hz 发布速度指令到 `/cmd_vel` 和 `/agent0/cmd_vel`。

---

### 模式 B：VLM 自动避障闭环（Step 3-6）

此模式为完整的端到端闭环：`track_path` 自动触发 Gemini → 获取像素路径 → 深度反投影转 3D → 跟踪路径 → 到达/遇障碍 → 再次触发。

#### Launch 启动（推荐，两个终端）

**终端 1 — Gemini API 节点**（LLM 交互日志独立显示）：
```bash
ros2 launch image_to_llm llm_node_launch.py
```

**终端 2 — 坐标转换 + 路径跟踪**（下游处理链路）：
```bash
ros2 launch my_robot_planner robot_vlm_launch.py
```

> 两个终端的日志互不干扰，方便分别调试 LLM 通信和路径跟踪。节点内置就绪等待机制，无论哪个终端先启动都能正确协作。

### 节点就绪逻辑

各节点内置了智能等待机制，无论启动顺序如何都能正确协作：

| 节点 | 等待条件 | 就绪标志 |
|------|---------|---------|
| `image_to_llm_node` | 收到首帧 RGB 图像 | ✅ 日志提示服务已就绪 |
| `image_conversion` | camera_info + 深度图 + odom 均已收到 | ✅ 日志提示准备接收像素数据；未就绪时缓存像素消息 |
| `track_path` | 收到 `/path` 路径 + odom 位置 | ✅ 开始 PID 跟踪 |

### 自动触发时机

`track_path` 会在以下情况自动向 Gemini 请求新路径：
1. **路径完成触发**：到达终点（距离 < 0.15m）后自动请求下一段
2. **障碍物触发**：深度图检测到正前方 < 0.8m 有障碍物，持续 3 秒后触发 Gemini 重规划

---

## 手动触发 LLM 规划

如果需要手动触发一次 Gemini 路径规划（例如调试时）：

```bash
ros2 service call /trigger_llm_plan std_srvs/srv/Trigger
```

也可以手动发布测试像素点（跳过 Gemini API，直接测试 image_conversion + track_path）：

```bash
ros2 topic pub --once /llm_pixels std_msgs/msg/String "{data: '[{\"point\": [500, 800]}, {\"point\": [500, 700]}, {\"point\": [500, 600]}, {\"point\": [480, 500]}, {\"point\": [460, 400]}]'}"
```

---

## ROS2 节点说明

| 节点名 | 包名 | 启动命令 | 功能 |
|--------|------|---------|------|
| `generate_path` | `my_robot_planner` | `ros2 run my_robot_planner generate_path` | 根据坐标和形状生成路径，发布到 `/path`（手动模式） |
| `track_path` | `my_robot_planner` | `ros2 run my_robot_planner track_path` | Pure Pursuit + PID 路径跟踪 + 深度图障碍物检测 |
| `image_to_llm_node` | `image_to_llm` | `ros2 run image_to_llm image_to_llm_node` | 将 RGB 图像发送给 Gemini，获取像素路径点 |
| `image_conversion` | `image_to_llm` | `ros2 run image_to_llm image_conversion` | 像素坐标 → 3D odom 坐标（深度反投影），发布 `/path` |

---

## 常用调试命令

```bash
# 查看所有活跃的 topic
ros2 topic list

# 监听路径点
ros2 topic echo /path

# 监听 LLM 返回的像素坐标
ros2 topic echo /llm_pixels

# 监听速度指令
ros2 topic echo /cmd_vel

# 查看节点列表
ros2 node list

# 查看服务列表（确认 /trigger_llm_plan 是否可用）
ros2 service list

# 手动触发 LLM 规划
ros2 service call /trigger_llm_plan std_srvs/srv/Trigger
```

---

## 关键 ROS2 Topic

| Topic | 类型 | 说明 |
|-------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 深度相机 RGB 图像 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 深度相机内参 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RGB 相机内参 |
| `/llm_pixels` | `std_msgs/String` | Gemini 返回的归一化像素坐标 JSON |
| `/path` | `nav_msgs/Path` | 3D 路径点（odom 坐标系） |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令 |
| `/agent0/cmd_vel` | `geometry_msgs/Twist` | Webots 机器人速度指令 |
| `/odom` | `nav_msgs/Odometry` | 里程计位置+朝向 |
| `/vrpn_mocap/rm_0_Test/pose` | `geometry_msgs/PoseStamped` | Mocap 位置（可选） |

> **注意**：相机话题名称取决于使用的深度相机型号。上表为当前 Intel D435 的话题名，更换为奥比中光 Gemini 336 后可能需要更新。话题参数在 launch 文件中配置，修改 launch 文件即可适配不同相机。

---

## 目录结构

```
humble_ws/
├── ARCHITECTURE.md          # 系统架构设计文档
├── PROCESS.md               # 开发步骤与进度
├── MEMORY.md                # 变更记录
├── README.md                # 本文件
├── install_dependencies.sh  # 一键依赖安装脚本
├── data/
│   ├── image/               # 运行时捕获的图像
│   └── log/                 # 坐标转换 debug 日志
├── future/                  # 未来架构设计文档
│   ├── function_calling_design.md
│   ├── skill_progressive_cognition.md
│   └── agentic_workflow_vision.md
└── src/
    ├── my_robot_msgs/       # 自定义服务消息 (GeneratePath.srv)
    ├── my_robot_planner/    # 路径生成 + 路径跟踪
    │   ├── launch/
    │   │   └── robot_vlm_launch.py  # 启动 conversion + track_path
    │   └── my_robot_planner/
    │       ├── generate_path.py
    │       └── track_path.py
    └── image_to_llm/        # Gemini API 交互 + 像素→3D 转换
        ├── launch/
        │   └── llm_node_launch.py   # 启动 image_to_llm_node
        ├── llm_config.env   # API 配置（不在 git 中）
        └── image_to_llm/
            ├── image_to_llm_node.py
            ├── image_conversion.py
            └── skills/      # Skill YAML 热插拔框架
                ├── __init__.py   # Skill 加载器
                └── default.yaml  # 默认路径规划 Skill
```
