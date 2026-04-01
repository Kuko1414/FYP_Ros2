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


## Mar.28, 2026
- 将 Gemini 多模态视觉模型接入 ROS2 节点流，完成了 Step 4 的核心图像转换开发。为了保证系统解耦与高并发稳定性，将其抽取为了两个独立环节：
  1. 新建 `image_to_llm` 包与 `image_to_llm_node` 节点，专门负责订阅彩色摄像头的 `sensor_msgs/Image`，通过内部封装将实时帧发送给 Gemini 2.5 Flash 多模态 API 并取回 2D 像素坐标系下避障路点的 JSON 然后发布于 `/llm_pixels`。
  2. 在 `my_robot_planner` 中新增了 `image_conversion` 节点。它监听 LLM 返回的像素坐标 JSON，结合深度相机 `depth_image` 提取的 `mono16` 毫米深度，利用相机内参模型 ($K$ 矩阵 $f_x, f_y, c_x, c_y$) 精准还原生成带方向的 $3D$ 真实坐标数据，并包装在 `nav_msgs/Path`。
- 修改了 `ARCHITECTURE.md` 与实际开发代码相匹配，增加了基于触发服务 `std_srvs/srv/Trigger` 的执行逻辑设计，以避免由于轮询刷新率过高导致的 API 并发上限与财务损失风险。

## Apr.1, 2026
- 将 `image_conversion` 节点从 `my_robot_planner` 包迁移至 `image_to_llm` 包，使 LLM 图像处理链路（发送+转换）在同一包内，提升模块内聚性。同步更新了两个包的 `setup.py` 入口点和 `image_to_llm/package.xml` 依赖（新增 `nav_msgs`、`geometry_msgs`）。
- 修复 `image_to_llm_node`：使用 `MultiThreadedExecutor` + `MutuallyExclusiveCallbackGroup` 解决 Gemini API 同步调用阻塞 `rgb_callback` 的问题；将 `json.loads()` 验证移至 `publish()` 之前，防止发布无效 JSON。
- 优化 `image_conversion`：将单点深度采样改为 5×5 窗口中值采样（`np.median`），提升抗噪声和深度空洞的鲁棒性。删除冗余导入。
- 更新 `ARCHITECTURE.md`：新增 `/imu/rpy/filtered`、`/odom`、`/tf`、`/tf_static`、`/depth_cam/rgb0/camera_info`、`/depth_cam/depth0/points` 等高价值 topic 文档，为 Step 5/6 开发提供参考。
- 在 `PROCESS.md` 中新增 Section 3（Extension Steps）和 Section 4（Deployment Notes），规划了核心管线之后的扩展路线，面向 Jetson Orin Nano 部署：
  - **Step 7**（高优先级）：实时可通行区域语义分割。新建 `semantic_perception` 包，用 BiSeNetV2/MobileNetV3-Seg（TensorRT FP16, ~10ms）对 RGB 逐像素分类，发布 `/semantic_mask`、`/traversable_path`（本地 fallback 路径）、`/semantic_overlay`。将架构从"停车等 API"改为"并行预判+本地 fallback"，消除 Gemini 延迟导致的停车等待。
  - **Step 8**（中优先级）：深度补全 CNN。在 `image_conversion` 节点内集成小型 U-Net（TensorRT FP16, ~5-10ms），输入 RGB+原始深度，输出修复后稠密深度图，提升 3D 坐标精度，消除深度空洞和边缘飞点。
  - **Step 9**（低优先级，依赖 Step 6+7）：持久语义地图。新增 `semantic_map_node`，将逐帧语义分割结果通过 TF2 累积为全局 `OccupancyGrid`，实现跨视野障碍记忆与全局路径规划。
  - **Step 10**：全系统集成优化，包括统一 launch 文件、GPU 显存管理、延迟 profiling、故障处理和 QoS 调优。

## Apr.1, 2026 (续)
- 在 `track_path.py` 中实现了 LLM 自动触发闭环（Step 5 核心），完成三种触发时机：
  1. **启动触发**：节点启动 3 秒后自动调用 `/trigger_llm_plan` 服务请求首条路径（一次性定时器 `startup_timer`）。
  2. **路径完成触发**：`control_loop` 检测到到达终点（<0.15m）后自动调用 `trigger_llm_replan("path_completed")`。
  3. **障碍物触发**：新增 `depth_obstacle_callback` 订阅深度图 `/depth_cam/depth0/image_raw`，检查中央 1/3 ROI 最近深度 < 0.8m 时停车并触发 LLM。含 10 秒冷却期防重复触发。
- 新增 `Trigger` 服务客户端连接 `image_to_llm_node` 的 `/trigger_llm_plan`，使用 `MutuallyExclusiveCallbackGroup` + `MultiThreadedExecutor` 确保异步调用不阻塞 10Hz 控制循环。
- 更新 `my_robot_planner/package.xml` 新增 `tf2_ros` 依赖。至此端到端闭环打通：`track_path 触发 → Gemini API → /llm_pixels → image_conversion → /path → track_path 跟踪 → 再次触发`。尚未在实机验证。
