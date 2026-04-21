For AI coding agents. This is the task we need to accomplish in this workspace and the design concept for each step. Read before making changes. 

## 1. Overview

This is a workspace for a wheeled robot design.

The desired functionality of this workspace is to design a path planning program that uses ROS2 and VLM to access API data to obtain path points and thereby control the movement of the wheeled robot. The robot should be able to plan a path from a starting point to a destination point, and then follow that path. The robot should also be able to detect obstacles in its path and re-plan accordingly.

## 2. Process

To achieve this powerful function, the design concept is as follows, ONLY WHEN IT IS PERMITTED TO PROCEED TO THE NEXT STEP CAN THE SUBSEQUENT FUNCTIONS BE WRITTEN.

  **Step 1.** (done) Firstly, design a preliminary function: create a Python node named "Generate_Path", which, upon receiving the starting point and the ending point, can generate several path points. The path shapes include but are not limited to: straight line, semi-circle, polynomial curve, S-shaped, etc. (Optimization: The terminal input of target coordinates must be done via a separate thread, ROS2 Service, or Action to avoid blocking the ROS2 node's spin loop). Then design another Python node named "Track_Path", which stores the ideal trajectory and plans the speed based on it. The final effect is: "Generate_Path" receives coordinates (e.g., 1.0 3.0 Semicircle), publishes the trajectory, and "Track_Path" makes the car navigate along it.
)
  **Step 2.** (done) AFTER ACHIEVING THE ABOVE FUNCTIONS, calculate the deviation between the ideal path points and the actual robot position at 10Hz. (Optimization: Instead of hard-segmented control rules that cause oscillation, use a continuous tracking algorithm like Pure Pursuit, Stanley, or continuous PID for all deviations under 0.5m to ensure smooth corrections). When the deviation is greater than 0.5m (indicating a severe track loss or obstruction), the vehicle forcefully stops and triggers a re-plan. The final effect is a smooth, continuous navigation approach to the destination with low average error.

  **Step 3.** AFTER ACHIEVING THE ABOVE FUNCTIONS, we added a depth camera and simulation camera. (Optimization: Distinguish between "detection distance" and "safe stopping distance". For instance, detect at 1.0m, and initiate braking so it safely stops at least 0.3m away). When an obstacle is detected in the path contour, a copy of the front image is saved to a dedicated `data/images` directory. Based on the walkable area feedback from the depth camera, it plans a route to go around the obstacle. 

  **Step 4.** AFTER ACHIEVING THE ABOVE FUNCTIONS, an MCP structure needs to be added to assist the robot in accessing Gemini through the API. Upon stopping for an obstacle, the front image is sent to Gemini to get path points in pixel coordinates. (Optimization: Network latency to Gemini must be handled by forcing the vehicle to remain stopped/waiting during API calls). The returned pixel points are sent to the "Image Conversion" Python node. (Optimization: This node must fuse the 2D pixel coordinates with the 'depth_image' depth values and 'camera_info' intrinsic matrix to convert them into real-world 3D coordinates in meters), and out to 'Track_Path' for tracking.
  **Step 5.** AFTER ACHIEVING THE ABOVE FUNCTIONS, it requires a full integration and testing suite. The depth camera detection and stopping logic (from Step 3) needs to be formally attached as the TRIGGER to the LLM path planning Service (from Step 4). When `Track_Path` pauses the vehicle due to an obstacle, it should asynchronously call `/trigger_llm_plan` to solicit a new trajectory. 
  
  **Step 6.** AFTER ACHIEVING THE ABOVE FUNCTIONS, implement Coordinate Transformation (TF2) to ensure the 3D Point coordinates outputted by `image_conversion` on `depth_camera_link` are dynamically translated into the `map` or `odom` reference frame so the chassis can robustly track them across both spatial domains without misalignments.

---

## ⚠️ Known Hardware Notes: Depth Camera

### Orbbec Aurora（已弃用，Apr.10 诊断）

The Orbbec Aurora (structured light) was diagnosed on Apr.10, 2026 with ~90% zero-value pixels when mounted at the robot's low height (~11cm). Root cause: near-parallel incidence angles on smooth tile floor. This camera has been **replaced**.

### Intel D435（临时使用）→ Orbbec Gemini 336（计划更换）

On Apr.14, 2026, temporarily using **Intel RealSense D435** for testing. Will switch to **Orbbec Gemini 336** soon. Key changes from Aurora era:
- `image_conversion.py` was rewritten to use **pinhole camera model reprojection** (depth image + camera_info intrinsics), removing the previous PointCloud-based approach.
- `track_path.py` obstacle detection switched from LiDAR/PointCloud to **depth image central ROI** minimum depth.
- All depth-related topics remain the same (`/depth_cam/depth0/image_raw`, `/depth_cam/depth0/camera_info`).
- **For future AI agents:** The depth camera hardware is still being stabilized. Do NOT assume depth data is always reliable. The `image_conversion` node has a ground plane fallback for when depth is unavailable.

---

## 3. Extension Steps (Post-Core Pipeline)

> **⚠️ 架构方向变更（Apr.13, 2026）：** 原 Step 7-10 计划在本地部署 CNN 模型（语义分割、深度补全）来增强感知能力。经过重新评估，决定**不再走本地 CNN 路线**，转而利用 Gemini 的 **Function Calling（工具调用）** 机制实现更强大的 Agent 架构。原因如下：
> 1. Gemini 本身已具备强大的视觉理解能力，可以同时完成路径规划和语义标注，无需额外训练本地模型
> 2. 本地 CNN 需要大量标注数据和训练工作，且模型泛化能力有限
> 3. Function Calling 架构更灵活，新增能力只需添加工具函数，不需要重新训练模型
> 4. 深度相机硬件仍在调试中，过早依赖深度数据的本地模型风险较高
>
> 原 Step 7-10 的详细设计已归档，保留在本节末尾的"已归档计划"中供参考。

### Step 7: Gemini Skill System — 系统提示词与任务上下文注入 (Priority: HIGH)

**Goal:** 通过精心设计的系统提示词（System Prompt）赋予 Gemini 特定的"技能"，使其从"看图出点"升级为"带着任务理解和语义知识看图"。

**Problem Being Solved:**
  - 当前 `image_to_llm_node` 发给 Gemini 的 prompt 只是简单的"规划路径点"，缺乏 任务上下文。
  - Gemini 因相机安装高度带来的透视问题，容易规划过短的距离。
  - 需要在单纯依靠纯追踪（Pure Pursuit）而不人工硬编码偏航（Yaw）转向逻辑的前提下，靠视觉连续引导向目标（如门口）前进。

**Implementation:**
  - 修改 `default.yaml` 技能文件：
    - 针对相机的低仰角透视，指示其在图像中上方（Y在500~550）落点，从而实现一次生成 1.5 到 2 米的路径。
    - 指导大模型平滑转弯并对齐长远目标（如“右方走廊的门”），利用大模型本身生成的平滑贝塞尔路线，依靠底层的 Pure Pursuit 实现自然偏航对齐。
- **Status:** **Completed**
    - 输出格式要求（路径点 + 语义标签的 JSON 格式）
  - 示例输出格式：
    ```json
    {
      "path": [{"point": [300, 400], "label": "corridor"}, {"point": [500, 700], "label": "workspace_1"}],
      "observations": [{"region": "left", "description": "桌子和椅子", "label": "desk_area"}]
    }
    ```
  - **改动范围：** 仅修改 `llm_config.env` 中的 PROMPT 内容，不改动任何节点代码。

---

### Step 8: Gemini Function Calling — Agent 工具调用架构 (Priority: HIGH)

**Goal:** 将 Gemini 从"单次问答计算器"升级为"多轮交互式 Agent"，使其能主动查询机器人状态、获取传感器数据、发布控制指令。

**Problem Being Solved:**
  - 当前 Gemini 只能被动接收一张图片然后输出坐标，无法主动获取信息
  - 无法根据中间结果调整策略（如"这条路太窄了，换一条"）
  - 路径规划完全依赖像素坐标 → 深度图反投影，受深度相机数据质量影响大

**Architecture:**
  ```
  当前（单次问答）：
    track_path 触发 → 拍照 → 发给 Gemini → 返回像素点 → image_conversion → /path

  Agent 模式（多轮交互）：
    用户/track_path 触发 → Agent 节点启动任务循环
    → Gemini: "我需要知道机器人位置" → 调用 get_robot_pose() → 返回结果
    → Gemini: "让我看看前方" → 调用 get_front_image() → 返回图像
    → Gemini: "前方有障碍，我规划一条绕行路径" → 调用 publish_path(points) → 发布到 /path
    → Gemini: "这片区域是走廊" → 调用 label_region("corridor", ...) → 存储语义标签
    → 循环直到任务完成
  ```

**New Files (in `image_to_llm` package):**
  - `robot_tools.py` — 工具执行层（`RobotTools` 类），内部订阅 ROS2 Topics 缓存数据，对外提供简洁的查询/操作函数
  - `tool_schemas.py` — 工具的 JSON Schema 定义，告诉 Gemini 有哪些工具可用
  - `agent_node.py` — Agent 模式主节点，实现多轮 Function Calling 循环

**Key Design Principles:**
  1. **工具层只通过 ROS2 标准接口通信**，绝不 import 其他节点的 Python 模块 → 零耦合
  2. **实时控制与高层决策分离**：PID 跟踪和障碍物急停仍由 `track_path` 本地处理，Gemini 只负责高层决策
  3. **工具函数保持原子化**：每个函数只做一件事（5-20 行），让 Gemini 来编排多步调用
  4. **与传统模式并存**：`agent_node` 和 `image_to_llm_node` 是并列关系，可随时切换

**Available Tools (initial set):**

| 工具名 | 功能 | 数据来源 |
|--------|------|----------|
| `get_robot_pose()` | 获取机器人位置和朝向 | `/odom` Topic |
| `get_front_image()` | 获取前方 RGB 图像 | `/depth_cam/rgb0/image_raw` Topic |
| `get_obstacle_distance()` | 获取前方最近障碍物距离 | `/depth_cam/depth0/image_raw` Topic |
| `publish_path(points)` | 发布导航路径点 | 发布到 `/path` Topic |
| `label_region(name, bounds)` | 标注语义区域 | 内存存储 |
| `get_semantic_labels()` | 查询所有语义标签 | 内存存储 |
| `get_current_region()` | 查询当前所在区域 | `/odom` + 语义标签 |

**Detailed design:** See `future/function_calling_design.md` for complete code examples, architecture diagrams, coupling analysis, and implementation checklist.

---

### Step 9: Progressive Cognition — 渐进式场景认知与 Skill 切换 (Priority: MEDIUM, depends on Step 8)

**Goal:** 通过三阶段渐进式 Skill 架构，让机器人像人一样逐步认识新环境：先粗略探索整体布局，再根据任务需要深入了解细节，最终成为熟悉环境的导航专家。

**Three-Phase Skill Model:**

| 阶段 | Skill 名称 | 标签粒度 | 行为 |
|------|-----------|---------|------|
| 1 | **Scout（侦察兵）** | 区域级（"工作区1"、"走廊"） | 快速扫描，建立粗粒度区域地图 + 拓扑连通关系 |
| 2 | **Inspector（检查员）** | 物体级 + 物品级（"桌子A 上有杯子"） | 在指定区域内详细标注物体，附属到区域 |
| 3 | **Navigator（导航专家）** | 不再标注，利用已有知识 | 查询语义地图，规划最优路径精准执行 |

**Hierarchical Semantic Labels:**
  - 语义标签从扁平结构升级为**层级结构**：区域 → 物体 → 物品
  - 每个区域有 `detail_level`（`coarse` / `fine`），Inspector 阶段自动升级
  - 区域间有 `topology`（连通关系图），Navigator 用于全局路径规划
  - 支持 `save_map()` / `load_map()` 持久化到 JSON 文件，跨会话记忆

**Skill Switching:**
  - 每个 Skill 定义为独立的 YAML 文件（`skills/scout.yaml` 等），包含 system_prompt + required_tools + max_turns
  - 通过 launch 参数切换：`ros2 launch image_to_llm agent_launch.py skill_name:=scout`
  - 新增 Skill 只需添加一个 YAML 文件，零代码改动

**New Tools (beyond Step 8 base set):**
  - `connect_regions()` — 记录区域连通关系
  - `add_object_to_region()` — 在区域内添加物体
  - `add_item_to_object()` — 在物体上添加附属物品
  - `find_object()` — 全局搜索物体/物品
  - `get_route()` — 基于拓扑图的最短路径（BFS）
  - `get_full_map()` — 获取完整语义地图
  - `save_map()` / `load_map()` — 持久化

**Detailed design:** See `future/skill_progressive_cognition.md` for complete three-phase workflow, hierarchical data structure, Skill YAML examples, and full usage scenario.

---

### Step 10: Full Agent Integration and Testing (Priority: after Steps 7-9)

**Goal:** 将 Skill System + Function Calling + 渐进式场景认知整合为完整的 Agent 工作流，并进行端到端测试。

**Key Tasks:**
  1. **Agent launch file:** 创建 `agent_launch.py`，启动 `agent_node` + `track_path` + 必要的传感器节点
  2. **Skill 框架搭建：** 创建 `skills/` 目录，实现 Skill 加载器和三个 YAML Skill 文件
  3. **多场景测试：** 测试三阶段 Skill 切换流程（Scout → Inspector → Navigator）
  4. **API 费用监控：** 多轮 Function Calling 会增加 Token 消耗，需要设置 `max_turns` 上限和费用预警
  5. **Fallback 机制：** 当 Gemini API 超时或不可用时，自动降级到传统模式（`image_to_llm_node`）
  6. **语义地图持久化：** 实现 `save_map()` / `load_map()`，支持跨会话环境记忆

---

### 已归档计划（原 Step 7-10，本地 CNN 路线）

> 以下计划已于 Apr.13, 2026 归档。如果未来需要在 Jetson Orin Nano 上部署本地感知模型（例如为了降低 API 延迟或离线运行），可以参考这些设计。

<details>
<summary>点击展开：原 Step 7 — 实时可通行区域语义分割（BiSeNetV2/MobileNetV3-Seg）</summary>

**Goal:** Give the robot continuous, frame-by-frame understanding of *where it can drive*, eliminating the current "blind until stopped" behavior.

**Model:** BiSeNetV2 or MobileNetV3-Seg (~1-3M params), TensorRT FP16, ~8-15ms/frame on Jetson Orin Nano.

**New Node:** `semantic_seg_node` in `semantic_perception` package. Publishes `/semantic_mask`, `/traversable_path`, `/semantic_overlay`.

**Integration:** `track_path` subscribes to `/traversable_path` as fallback; `image_to_llm_node` optionally sends semantic overlay to Gemini.

</details>

<details>
<summary>点击展开：原 Step 8 — 深度补全/去噪 CNN（U-Net）</summary>

**Goal:** Replace raw depth image with CNN-refined dense depth map in `image_conversion.py`.

**Model:** Small U-Net (~1-2M params), input RGB+depth, output refined depth. TensorRT FP16, ~5-10ms/frame.

**Integration:** Inside `image_conversion` node's `depth_callback`, minimal code change.

</details>

<details>
<summary>点击展开：原 Step 9 — 持久语义地图（OccupancyGrid）</summary>

**Goal:** Accumulate per-frame semantic segmentation into global 2D semantic grid map.

**Prerequisites:** Step 6 (TF2) + Step 7 (segmentation).

**New Node:** `semantic_map_node` in `semantic_perception` package. Publishes `/semantic_map` (OccupancyGrid).

</details>

<details>
<summary>点击展开：原 Step 10 — 全系统集成优化</summary>

**Goal:** Integrate all CNN components, optimize GPU memory, latency profiling, failure handling, QoS tuning on Jetson Orin Nano.

</details>

---

## 4. Deployment Notes

**Target Hardware:** Jetson Orin Nano (8GB)
  - GPU: 1024-core Ampere, 32 Tensor Cores, 40 TOPS INT8
  - For Agent mode (Step 8): No local CNN required, only needs stable network connection to Gemini API
  - If local CNN models are needed in the future (archived Steps 7-10): PyTorch → ONNX → TensorRT (FP16), use `trtexec` on Jetson

**Agent Mode Requirements:**
  - Stable internet connection for Gemini API access
  - `google-genai` Python SDK with Function Calling support
  - API key configured in `llm_config.env`
  - Expected latency per Agent turn: 1-5 seconds (network dependent)
  - Recommended `max_turns` limit: 10 (to control API costs)

**Recommended Directory Structure Extension:**
  ```
  src/
      image_to_llm/
          image_to_llm/
              image_to_llm_node.py   # 传统模式（保留）
              image_conversion.py    # 像素→odom 转换（保留）
              agent_node.py          # NEW: Agent 模式主节点
              robot_tools.py         # NEW: Gemini 工具函数集
              tool_schemas.py        # NEW: 工具 JSON Schema
          launch/
              llm_node_launch.py     # 传统模式 launch
              agent_launch.py        # NEW: Agent 模式 launch
  future/
      agentic_workflow_vision.md     # Agent 架构愿景
      function_calling_design.md     # Function Calling 详细设计（含完整代码示例）
      skill_progressive_cognition.md # 渐进式场景认知 Skill 架构设计
  data/
      images/                        # runtime captured images
      log/                           # conversion debug logs
      semantic_labels.json           # NEW: 持久化语义标签（未来）
  ```
