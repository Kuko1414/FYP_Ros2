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

## Apr.8, 2026
- 为 `image_conversion` 添加智能就绪等待：camera_info + 深度图未就绪时缓存 `/llm_pixels` 消息（`_pending_pixel_msg`），两者就绪后自动处理缓存，消除启动顺序依赖。
- 重构 `track_path` 启动触发逻辑：将固定 3 秒定时器改为每 2 秒轮询 `/trigger_llm_plan` 服务可用性，确认可用后才触发首次 LLM 请求，彻底消除启动时序竞态。
- 新建 `my_robot_planner/launch/robot_vlm_launch.py` 一键启动 3 个核心节点（image_to_llm_node + image_conversion + track_path），更新 `setup.py` 注册 launch 文件。
- 同步更新 `README.md`：新增一键 Launch 启动说明、节点就绪逻辑表格、目录结构中 launch 文件。

- 同步更新 `README.md`：新增一键 Launch 启动说明、节点就绪逻辑表格、目录结构中 launch 文件。
=======
- 同步更新 `README.md`：新增一键 Launch 启动说明、节点就绪逻辑表格、目录结构中 launch 文件。

## Apr.8, 2026 (续)
- 重写 `image_conversion` 节点，改用 **PointCloud2 点云优先**的三级 fallback 架构：Level 1 从 `/depth_cam/depth0/points` 按像素索引直接获取 3D 坐标（无需针孔模型反投影）；Level 2 fallback 到深度图+内参；Level 3 fallback 到地面平面假设。原因：实测深度图因地面材质全零，但点云有 12476 个有效 3D 点（100% 有效率）。
- 修复 TF2 逐点变换失败导致路径点被丢弃的问题：异常捕获后降级为 `depth_camera_link` 坐标系发布，而非静默丢弃。将逐点变换时间戳改为 `rclpy.time.Time()`（最新可用变换），解决 `ExtrapolationException`。
- 点云订阅使用 `BEST_EFFORT` QoS 匹配发布端。更新 `robot_vlm_launch.py` 新增 `pointcloud_topic`、`pc_search_radius` 参数。小车跟踪效果仍待进一步调试（坐标系变换后路径点在 odom 平面的分布可能需要优化）。

## Apr.10, 2026
- 重构 `track_path.py`：因路径点改由 Gemini 经 `image_conversion` 提供，移除所有 `generate_path` 服务客户端、LLM 自动触发逻辑、深度图障碍物检测。简化为单线程 `rclpy.spin()` 的纯 PID 路径点追踪器。同步更新 `package.xml`（移除 `cv_bridge`、`std_srvs`、`tf2_ros`）。
- 诊断深度相机问题：深度图 89.9% 零值（地面 98.8% 零值），原因是相机安装太低（~11cm）导致入射角过大+地面镜面反射。深度值本身正确（`mono16` mm 单位）。将诊断结论写入 `PROCESS.md` 警告章节。
- 将 `track_path.py` 障碍物检测从点云改为**雷达 LaserScan**（`/scan_raw`）。雷达 91.3% 有效率、0.06m~25m 量程、360° 覆盖，远优于深度相机（1.3m 盲区）。点云保留给 `image_conversion` 做像素→3D 坐标转换。更新 `robot_vlm_launch.py` 参数和 `ARCHITECTURE.md` 新增雷达话题文档。

## Apr.13, 2026
- 更换深度相机为奥比中光 Gemini 2L（双目），全面移除雷达依赖，改用深度图做距离计算和障碍物检测。
- 重写 `image_conversion.py`：移除雷达+固定步长模式，改用**针孔相机模型反投影**（`Z=depth[v,u]`, `X=(u-cx)*Z/fx`, `Y=(v-cy)*Z/fy`）+ odom 坐标转换。新增深度图订阅（BEST_EFFORT QoS）、窗口中值滤波采样、地面平面 fallback。所有话题参数化。
- 精简 `track_path.py`（722→230 行）：移除 debug 日志、原地转向、cross-track error、Mocap/GPS 多位置源。障碍物检测改为深度图中央 ROI 最近深度判定。路径完成/障碍物持续 3s 时直接调用 `/trigger_llm_plan` 触发 Gemini 重规划。
- 更新 `robot_vlm_launch.py` 参数（`scan_topic`→`depth_image_topic`，新增 ROI、深度范围等参数）。`my_robot_planner/package.xml` 新增 `cv_bridge` 依赖。

## Apr.13, 2026 (续)
- 架构方向变更：决定放弃本地 CNN 路线（语义分割、深度补全），转向 Gemini Function Calling Agent 架构。原因：Gemini 已具备视觉理解+语义标注能力，无需额外训练本地模型；深度相机硬件仍不稳定。
- 新建 `future/function_calling_design.md`：完整的 Function Calling 架构设计文档，包含 `RobotTools` 工具类代码（7 个工具函数）、`tool_schemas.py` JSON Schema 定义、`agent_node.py` 多轮循环伪代码、耦合性分析、实施检查清单。
- 重写 `PROCESS.md` Section 3：新 Step 7（Skill 系统提示词注入）、Step 8（Function Calling Agent 架构）、Step 9（Gemini 语义标签）、Step 10（Agent 集成测试）。原 Step 7-10 本地 CNN 计划用折叠标签归档保留。更新 Section 4 部署说明和目录结构。
- 更新 `PROCESS.md` 硬件章节：精简 Orbbec Aurora 为"已弃用"摘要，新增 Gemini 2L 当前状态。
- 新建 `future/skill_progressive_cognition.md`：渐进式场景认知 Skill 架构设计。三阶段模型（Scout 粗粒度区域探索 → Inspector 细粒度物体标注 → Navigator 精准导航），层级式语义标签数据结构（区域→物体→物品 + 拓扑连通图），Skill YAML 热插拔框架（加载器 + 3 个 Skill 文件），新增 9 个工具函数设计，完整使用场景示例。
- 更新 `PROCESS.md` Step 9：从简单的语义标签改为渐进式场景认知与 Skill 切换架构，Step 10 同步更新。目录结构新增 `skill_progressive_cognition.md` 引用。
- **实现 Skill 框架（Step 7 落地）：** 新建 `src/image_to_llm/skills/` 目录，含 `__init__.py`（Skill 加载器，`load_skill()`/`list_skills()`）和 `default.yaml`（从 `llm_config.env` 的 PROMPT 迁移而来的默认 Skill）。修改 `image_to_llm_node.py`：新增 `skill_name` 参数，从 YAML 加载 prompt 替代 env 中的 PROMPT；模型优先级为 env > YAML > 默认值。更新 `llm_node_launch.py` 支持 `skill_name:=xxx` 参数。更新 `setup.py` 注册 skills 包和 YAML data_files。精简 `llm_config.env` 仅保留 API_KEY 和代理。

## Apr.14, 2026
- **更换深度相机后话题适配：** 临时使用 Intel D435（后续将换为奥比中光 Gemini 336），ROS2 驱动话题命名与旧 Aurora 不同，更新了所有 launch 文件中的话题参数：RGB `→ /camera/color/image_raw`，深度图 `→ /camera/depth/image_raw`，camera_info `→ /camera/depth/camera_info`。涉及 `llm_node_launch.py` 和 `robot_vlm_launch.py`。同步更新 `ARCHITECTURE.md` 话题文档。注意：换 Gemini 336 后话题名可能再次变化，届时需重新适配。
- **修复 skills 模块 import 失败：** `skills/` 原来放在包根目录（与 `image_to_llm/` 并列），被 setuptools 安装为独立顶层包，导致 `from image_to_llm.skills import ...` 找不到。将 `skills/` 移至 `image_to_llm/image_to_llm/skills/` 使其成为子包，并在 `setup.py` 添加 `package_data` 确保 `.yaml` 文件随包安装。
- **端到端坐标转换验证通过：** 手动发布测试像素点到 `/llm_pixels`，`image_conversion` 成功用深度反投影生成 odom 路径点。D435 深度图有效率 68.8%，远优于旧 Aurora（~10%）。验证日志：`data/log/conversion_20260414_221336.log`。

## Apr.15, 2026
- **PID 抗振荡调参（`track_path.py`）：** 实测发现小车沿路径剧烈振荡（yaw ±54°），根因是角速度 PID 的 kd=0.6 过大导致 D 项过度反应。调整：angular kp 1.0→0.6, ki 0.02→0.01, kd 0.6→0.1；linear kp 0.4→0.3, ki 0.02→0.01, kd 0.1→0.05；lookahead_dist 0.6→0.3m。新增角度误差低通滤波（alpha=0.3）和余弦衰减减速策略。实测振荡完全消除，yaw 振幅降至 ±3°。
- **修复坐标转换左右镜像 bug（`image_conversion.py`）：** `_camera_to_odom` 公式中 cam_x 符号错误，相机"右侧"被映射到 odom 的"左侧"。修正：`-cam_x*sin(yaw)` → `+cam_x*sin(yaw)`，`+cam_x*cos(yaw)` → `-cam_x*cos(yaw)`。实测确认小车现在正确地往图像右侧方向行驶。
- **深度图就绪检查加强（`image_conversion.py`）：** `pixel_callback` 原来只检查 camera_info + yaw 稳定，未检查深度图是否就绪，导致路径全部使用不准确的地面假设 fallback。现在三者全部就绪才处理像素消息，否则缓存等待自动处理。

## Apr.15, 2026
- **修复路径方向错误（根因：IMU/odom yaw 启动不稳定）：** 通过 `data/test_odom_yaw.py` 测试脚本验证发现，odom 启动初期 yaw 会从 0° 剧烈跳动后收敛到真实值（~149°），而 `image_conversion` 在 yaw 未稳定时就执行了坐标转换（yaw=-5.8°），导致路径方向完全错误。在 `image_conversion.py` 中新增 **yaw 稳定性检查**：用 `deque` 记录最近 3 秒 yaw 历史，当变化幅度 < 2° 时才标记就绪。不硬编码任何角度值，适用于任何朝向/场景。修复后路径方向与实际移动方向差值从 ~150° 降至 ~1°。
- **新增位姿记录器 `data/pose_logger.py`：** 每 0.5s 采样 odom (x, y, yaw)，Ctrl+C 退出时自动保存到 `data/log/pose_YYYYMMDD_HHMMSS.log`，含汇总（起终点、总位移、移动方向角）和详细轨迹表格，用于 PID 调参和路径 debug。
- **PID 调参（进行中）：** 原参数 angular(kp=2.0, ki=0.1, kd=0.3) 导致严重 S 型振荡（yaw 振幅 ~60°）。第一轮调为 (1.2, 0.02, 0.5)，振幅降至前半段 ~11°、后半段 ~42°。第二轮调为 (1.0, 0.02, 0.6)，angular max 1.2，linear max 0.4，lookahead 0.3→0.6，待测试验证。

## Apr.19, 2026
- **解决 Jetson 宕机与轨迹边沿跳变**：调低 Orin 功耗与分辨率解决相机启动死机；优化 `default.yaml` 提示词禁止大模型跨越障碍物生成路点。
- **引入 TF2 重构由于机械误差导致的坐标漂移偏移**：在 `robot_vlm_launch.py` 注入实测高度（Z=155mm）的静态 TF 树。重构 `image_conversion.py`，彻底废除手动 `odom` 订阅和三角函数算姿态，转用 `tf2_ros.TransformListener`。此修改通过依赖系统级 TF 树计算深度流像素的级联空间矩阵，完美解决了路点一直错位到“车轮下”的追踪异常。
- **完善端到端纯视觉导航闭环（到达终点门）**：修复了 `image_conversion.py` 中 TF 缓冲区对未来时间戳的外推报错（强制 stamp=0）。在 `robot_vlm_launch.py` 中将 `obstacle_distance` 安全距离缩小至 0.2m 防止提前误刹车。去除了 `track_path.py` 中所有硬编码的绝对坐标与原地旋转逻辑。在 `default.yaml` 的提示词中，向大模型引入了相机极低安装高度带来的“透视原理”约束，强制大模型将局部路径点纵深规划到画面中上方（Y在500~550附近），成功将单次步进距离从0.3m提升至1.5m~2m。通过纯粹的“拍摄->生成平滑短曲线->循迹->再拍摄”闭环，实现了自然平滑的转向与长距离避障，最终成功引导小车到达走廊门前。

## Apr.22, 2026
- **修复 Agent 模式 RGB 图像无法获取（相机驱动阻塞）：** 根因：`robot_tools.py` 和 `track_path.py` 使用 `RELIABLE` QoS 订阅相机话题，多个 RELIABLE 订阅者导致 Orbbec Gemini 2L USB 相机驱动的 DDS 发送队列饱和，相机完全停止发布所有图像。解决：将两个文件的相机订阅 QoS 改为 `BEST_EFFORT`（RELIABLE publisher → BEST_EFFORT subscriber 兼容）。同时为 `get_front_image` 增加 3 轮×5 秒重试等待，`_rgb_cb` 不再静默吞异常。
- **修复 Gemini 规划路径方向反转（坐标系公式错误）：** 根因：`agent_default.yaml` 中 yaw 定义写成"顺时针为正"，但实测 `/odom` 的 yaw 是标准 ROS2 的逆时针为正，导致前方/左方/右方坐标转换公式全部反向。解决：修正 yaw 定义为"逆时针为正（90°=正北, -90°=正南）"，更正三个方向转换公式，同步更新 `robot_tools.py` docstring。
