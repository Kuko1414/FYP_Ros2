For AI coding agents. This is a record of each change made to the workspace content. You can refer to what changes we have previously made. If you want to make a change, please first refer to ARCHITECTURE.md 

## Mar.14, 2026
- Added the overview of the workspace, the ROS2 topics, the ROS2 nodes, and the workspace structure in ARCHITECTURE.md. But there is no functional code in this stage, so the MEMORY.md is still empty.
- Implemented Step 1 of PROCESS.md. Created `my_robot_planner` package in `src`. Added `Generate_Path` node to publish paths based on user input (shape, target coords) using starting pose. Added `Track_Path` node to subscribe to the generated paths and publish `cmd_vel` to control the robot movement. Both nodes align with ARCHITECTURE.md topic specifications.

## Mar.17, 2026
- Fixed `Track_Path` node: added IMU subscription (`/agent0/imu`) for yaw extraction, fixed `msg.pose` → `msg.poses` bug, replaced rudimentary steering with heading-based proportional control using angle error. Mocap mode extracts yaw from orientation directly; Webots mode uses IMU.
- Added IMU topic remapping in Webots `robot_launch.py`: `/imu` → `/agent0/imu` and `/agent1/imu` to separate two robots' IMU data.
- Updated `ARCHITECTURE.md` to document the `/agent0/imu` topic and orientation acquisition strategy.
- Refactored `Generate_Path`: removed persistent position subscription. Now subscribes on-demand when path generation is triggered (manual input or service call), then unsubscribes. Added ROS2 Service server (`/generate_path`, `my_robot_msgs/srv/GeneratePath`) for programmatic replan requests. Input thread now only requires target X Y and shape.
- Refactored `Track_Path`: added Service client for `/generate_path`. Added deviation monitoring via `find_closest_path_point()`. When deviation > 0.5m, stops robot and sends async replan request. Stores final target for replan use.
- Created `my_robot_msgs` package with custom `GeneratePath.srv` (request: target_x, target_y, shape; response: path, success).
- Updated `ARCHITECTURE.md` node descriptions and `my_robot_planner/package.xml` dependencies.

## Mar.24, 2026
- 在 README.md 的依赖部分添加了 `source install_dependencies.sh` 代码块。此更改是为了直观提示用户运行环境安装脚本，确保依赖项正确安装，从而符合系统整体设计中对快速部署和环境配置易用性的要求。
- 修改了 `test_sendImage.py`，要求 Gemini 输出无格式 JSON 路径点，通过 `json`、`re` 与 `PIL.ImageDraw` 解析并在原图绘制红色路径，保存至 `/home/kuko/Pictures/Gemini_Path.jpg`。原因是 LLM 无法直接返回文件。此举验证了多模态视觉规划逻辑，为后续将其整合至 ROS2 系统的环境感知与规划引擎提供了原型基础。
- 更新了 `test_sendImage.py` prompt，向 Gemini 注入图像实际宽高 (`img.size`)，并强调使用左上角为 `(0,0)`、范围在 `[0, width]` 及 `[0, height]` 内的坐标系。原因在于提升轨迹坐标提取与红线绘制的准确度，此举验证了基于大模型视觉感知的精准空间映射映射能力，契合系统端到端智能路径规划的设计要求。
- 修改了 `test_sendImage.py`，更改提示词要求模型输出 `[0, 1000]` 的归一化坐标，并在代码中根据原图宽高进行缩放还原。原因：直接请求绝对像素坐标会产生严重的平移偏移。设计对齐：此举大幅提升了大模型输出空间坐标的准确性，确保规划路径与图像精确对齐，符合系统高精度视觉引导的要求。

