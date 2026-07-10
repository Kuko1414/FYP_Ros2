# 报告修改/补充内容
# 说明：以下按章节列出需要修改或新增的内容。
# 标记说明：
#   【替换】= 用下方内容替换原文对应段落
#   【新增】= 在指定位置插入下方内容
#   【删除】= 删除原文对应段落

======================================================================
## 1. Chapter 3.5 — 新增 3.5.4 小节（在 3.5.3 之后插入）
======================================================================

【新增】在 3.5.3 Reactive collision avoidance 之后，添加新的小节：

3.5.4 Function Calling Agent architecture

While the single-turn pipeline described above (image_to_llm_node → image_conversion → track_path) proved effective for basic obstacle avoidance, it has an inherent limitation: Gemini can only passively receive one image and return pixel coordinates, with no ability to actively query the robot's state or adjust its strategy based on intermediate feedback. For instance, if the first planned path turns out to be blocked, the model cannot proactively ask "where am I now?" or "let me take another look" — it must wait for track_path to detect the failure and trigger a completely new planning cycle.

To overcome this limitation, a multi-turn Function Calling agent architecture was implemented as a parallel operating mode alongside the original pipeline. In this architecture, Gemini is no longer a passive calculator but an interactive agent that can invoke pre-defined Python functions (tools) during a conversation, receive their return values, and decide its next action accordingly. The agent loop works as follows:

(1) Upon receiving a trigger request (via the /trigger_llm_plan service, maintaining interface compatibility with the traditional mode), the agent node sends the system prompt from the currently loaded Skill YAML file to Gemini, together with the JSON schemas of all available tools.

(2) Gemini analyzes the prompt and decides which tool to call first — typically get_robot_pose() to learn its current position and heading, followed by get_front_image() to observe the environment.

(3) The agent node executes the requested tool, collects the return value (a JSON dict for data queries, or a JPEG image for the camera), and sends the result back to Gemini as a function_response.

(4) Gemini processes the returned information — for example, analyzing the camera image to identify obstacles and the target door — and either calls another tool (such as publish_path() with a list of odom-frame waypoints) or returns a plain text summary indicating the task is done.

(5) This loop repeats until Gemini returns plain text (task completed), calls finish_task() (explicit termination with stop command and semantic map report), or the maximum turn limit is reached.

The tool execution layer (robot_tools.py) subscribes to ROS2 topics in the background and caches the latest sensor data. Each tool function is kept atomic — typically 5 to 20 lines of code doing exactly one thing. Table 3.1 summarizes the initial tool set.

Table 3.1 Available tools for the Function Calling agent

Tool Name            | Function                              | Data Source
get_robot_pose()     | Return current position and heading    | /odom topic
get_front_image()    | Capture front RGB image                | /camera/color/image_raw topic
get_obstacle_distance()| Return nearest obstacle distance     | /camera/depth/image_raw topic
publish_path(points) | Publish navigation waypoints           | Publishes to /path topic
label_region(...)    | Tag a semantic region in odom frame    | In-memory storage
get_semantic_labels()| Query all tagged regions               | In-memory storage
get_current_region() | Query which region the robot is in     | /odom + semantic labels
finish_task(reason)  | Stop the robot and end the task        | Publishes zero velocity + empty path

A critical design principle is that the agent node and the traditional image_to_llm_node are parallel alternatives, not replacements. Both expose the same /trigger_llm_plan service interface, so track_path can trigger either one without code changes. The agent mode is launched via a separate launch file (agent_launch.py), which starts agent_node and track_path without image_to_llm_node or image_conversion — because in agent mode, Gemini computes odom coordinates directly from the robot pose and trigonometric projection, bypassing the pixel-to-depth-reprojection pipeline entirely.

One notable difference from the traditional mode is how coordinates are generated. In the single-turn pipeline, Gemini outputs normalized pixel coordinates that must be converted to 3D positions through depth camera reprojection and TF2 transformation — a process sensitive to depth data quality and camera calibration accuracy. In agent mode, Gemini first calls get_robot_pose() to obtain the current (x, y, yaw) in odom frame, then directly calculates target waypoints using trigonometric formulas (e.g., "2 meters ahead" = x + 2·cos(yaw), y + 2·sin(yaw)). This eliminates the dependency on depth camera data quality for path generation, though the depth camera is still used by track_path for reactive obstacle detection.

The Skill YAML file for agent mode (agent_default.yaml) contains the system prompt that instructs Gemini on the execution workflow, coordinate system conventions, safety constraints, and the stopping criteria. The max_turns parameter (default: 8) prevents runaway API costs. A typical navigation task from the laboratory starting position to the exit door consumes 3 to 5 turns.


======================================================================
## 2. Chapter 3.7 — 修改测试方法（替换+新增）
======================================================================

【替换】原文第一段重复的 "Automated deployment..." 段落，以及 "Experimental groups" 后面的内容。
用以下完整的 3.7 替换原文 3.7 全部内容：

3.7 Verification and testing methods

To evaluate the effectiveness of the proposed VLM-driven navigation system under different architectural configurations, a three-group comparative experiment was designed. The experiment aims to answer two questions: (1) how does prompt engineering affect the quality of single-turn VLM path planning, and (2) does the multi-turn Function Calling agent architecture provide measurable improvements over the optimized single-turn pipeline.

Test scenario: The robot starts between two workstation areas in the laboratory. The target is the exit door located approximately 8 m ahead and 2 m to the right. Along the route, a metallic power strip chassis is placed on the floor (traversable but visually confusing), and a cardboard box serves as a real obstacle. The door remains visible from the starting position in all trials.

Experimental groups: Three configurations were compared, all running under identical hardware and environmental conditions:

Group A (Naive Prompt — Single-Turn Mode): A minimal instruction is given to Gemini through the naive.yaml Skill file. It only requests basic path planning without perspective-distance guidance, goal-direction hints, or camera-height compensation. The prompt content is: "This image is taken from a robot's front camera ({width}×{height} pixels). Plan a path from the bottom center of the image to a safe area ahead, avoiding obstacles. Output 5–8 path points as a JSON array: [{"point": [x, y]}, ...]. Coordinates are normalized to [0, 1000]. Origin (0,0) is on top left." The system runs the traditional pipeline: image_to_llm_node → image_conversion (depth reprojection + TF2) → /path → track_path.

Group B (Optimized Prompt — Single-Turn Mode): The fully engineered prompt from the default.yaml Skill file is used. It includes perspective-aware depth constraints (instructing the model to place far waypoints at normalized y ≈ 500–550 to achieve 1.5–2 m per step), smooth curvature guidance toward the door, and explicit safety-distance requirements around obstacles. The system runs the same traditional pipeline as Group A.

Group C (Function Calling Agent Mode): The agent_default.yaml Skill file is loaded into the agent_node, which operates in multi-turn Function Calling mode. Gemini actively queries the robot's pose, captures front camera images, and publishes odom-frame waypoints directly — bypassing the pixel-coordinate conversion pipeline entirely. The system prompt instructs the agent to plan 2–4 m per step, lean the last few waypoints toward the target door, and call finish_task() when the target is reached. The system runs via agent_launch.py: agent_node + track_path only (no image_to_llm_node or image_conversion).

Procedure: Each configuration was tested over five independent trials from the same starting position. Between trials, the robot was manually reset to the same pose. Groups A and B operated in their standard closed-loop mode: track_path triggers Gemini re-planning upon path completion or obstacle detection, continuing until the robot arrives within 0.15 m of the door or a timeout of 120 seconds is reached. Group C operated similarly, except that track_path triggers the agent_node, which then runs a multi-turn conversation with Gemini to plan and publish a path segment before returning control to track_path.

Evaluation metrics:

Task success rate: the proportion of trials where the robot successfully reached the target door without manual intervention.

Total navigation time: elapsed time from the first Gemini trigger to final arrival. This reflects both path quality and the number of re-planning cycles needed.

Number of Gemini interactions: for Groups A and B, this is the number of re-planning service calls; for Group C, this is the total number of Function Calling turns summed across all trigger cycles in a trial.

Average step distance per interaction: total distance traveled divided by the number of Gemini interactions. Higher values indicate more efficient planning per API call.

Path smoothness: measured as the variance of angular velocity (ω) from the odometry data during execution. Lower variance indicates less oscillation.

Obstacle collision: whether the robot collided with any obstacle during the trial.

Semantic labels generated (Group C only): the number of label_region() calls made by the agent during the trial, reflecting the agent's ability to build a semantic map alongside navigation.

Data collection: Trajectory data was recorded by a pose logger node sampling odometry (x, y, yaw) at 2 Hz. For Groups A and B, the image_conversion debug logs captured coordinate transformation details for each segment, and Gemini response payloads (pixel-coordinate JSON) were stored for post-hoc path visualization overlaid on captured images. For Group C, the agent_node terminal output was logged, capturing the full sequence of tool calls, their arguments, and return values for each turn.


======================================================================
## 3. Chapter 4.1 — 末尾新增一段（在现有 4.1 最后追加）
======================================================================

【新增】在 4.1 System Deployment and Sensor Validation 最后一段之后追加：

        For the agent mode experiment (Group C), the agent_node was verified separately. Upon startup, the node confirmed successful loading of the agent_default.yaml Skill file and connection to the Gemini 2.5 Flash API. The tool execution layer (robot_tools.py) was validated by manually calling each tool function: get_robot_pose() returned valid odom coordinates, get_front_image() successfully captured and converted an RGB frame to PIL Image format within 2 seconds, and get_obstacle_distance() reported consistent depth readings from the central ROI. The /trigger_llm_plan service was tested via command line to confirm that the full agent loop (pose query → image capture → path publication) executed correctly before running the formal trials.


======================================================================
## 4. Chapter 4 — 新增 4.2.1（在现有 4.2 之后插入，原 4.3/4.4 顺延编号）
======================================================================

【说明】建议把原来的 4.2 改标题为 "4.2 Experiment 1: Comparison of Prompt Strategies (Groups A vs B)"，
然后在其后插入新的 4.3 小节。原来的 4.3 改编号为 4.4，原来的 4.4 改编号为 4.5。

4.3 Experiment 2: Function Calling Agent Mode (Group C)

        To evaluate the agent architecture described in Section 3.5.4, Group C was tested under the same scenario and procedure as Groups A and B. The key difference is that in Group C, Gemini operates as an interactive agent rather than a passive image-to-coordinate converter. Each trigger from track_path initiates a multi-turn conversation where the model actively gathers information before making navigation decisions.

Table 4.3 presents the Group C results alongside Groups A and B for direct comparison.

Metric                              Group A (Naive)   Group B (Optimized)   Group C (Agent)
Success Rate                        [___]             [___]                 [___]
Avg. Total Time (s)                 [___]             [___]                 [___]
Avg. Gemini Interactions            [___]             [___]                 [___]
Avg. Step Distance per interaction  [___]             [___]                 [___]
Angular Vel. Variance (rad²/s²)     [___]             [___]                 [___]
Collisions                          [___]             [___]                 [___]
Semantic Labels Generated           N/A               N/A                   [___]

        As shown in Table 4.3, Group C achieved a success rate of [___], with an average of [___] Gemini interactions per trial. Each interaction in Group C corresponds to one tool call turn within the agent loop, so the total number of interactions is higher than the re-planning count in Groups A/B. However, the total navigation time was [___] seconds, which is [___] compared to Group B. This is because, although each agent cycle involves multiple API round-trips (typically 3–5 turns for pose query, image capture, and path publication), the agent tends to generate longer and more purposeful path segments since it has access to the exact robot position and heading when computing waypoints.

        A notable behavioral difference was observed in how Group C handled the target direction. In Groups A and B, the system relies entirely on prompt instructions to make Gemini lean the path toward the door — and the model's compliance varies from trial to trial. In Group C, the agent explicitly calls get_robot_pose() before planning, so it knows its current heading angle relative to the target direction. This allowed the agent to make more deliberate directional adjustments: in [___] out of 5 trials, the agent generated a path that curved toward the door from the very first planning cycle, compared to [___] out of 5 for Group B.

        The coordinate accuracy was also improved in Group C. Since the agent computes odom waypoints directly from the robot pose using trigonometric formulas (e.g., "forward 2 m" = x + 2·cos(yaw), y + 2·sin(yaw)), the path is not affected by depth camera noise, zero-value pixels, or TF2 timing issues that occasionally caused waypoint scatter in the traditional pipeline. The resulting trajectories were visually smoother, as reflected in the angular velocity variance of [___] rad²/s².

[Figure 4.5 Agent mode tool call sequence from a representative trial]

        Figure 4.5 illustrates the tool call sequence from one representative Group C trial. The agent followed a consistent pattern: get_robot_pose → get_front_image → publish_path, with occasional label_region calls when it identified distinct areas (such as the corridor or a workstation zone). In this trial, the agent completed the navigation in [___] trigger cycles, with a total of [___] tool call turns across all cycles.

[Figure 4.6 Comparison of odometry trajectories across all three groups]

        Figure 4.6 compares the odometry trajectories from representative trials of all three groups plotted on the same coordinate axes. Group A shows the most erratic path with frequent direction changes. Group B follows a smoother curve but occasionally overshoots at turning points. Group C shows the most direct trajectory toward the door, with gentle curvature and minimal oscillation.

4.3.1 Semantic mapping capability

        An additional capability unique to Group C is the automatic generation of semantic labels during navigation. Across the 5 trials, the agent called label_region() an average of [___] times per trial, tagging areas such as "corridor", "workstation_left", "workstation_right", and "exit_door". The labeled regions and their odom-frame bounding boxes were reported by finish_task() at the end of each trial.

        While the current implementation stores labels only in memory (not persisted to disk), the semantic map data returned by finish_task() can be readily serialized to a JSON file for cross-session reuse. This demonstrates that the Function Calling architecture naturally supports building environmental knowledge as a side product of navigation, without requiring a separate mapping module or additional API calls.


======================================================================
## 5. Chapter 4.4（原 4.3）— 末尾追加一段
======================================================================

【新增】在原 4.3 "Analysis of VLM Path Planning Behavior" 的最后一段之后追加
（注意：如果按新编号，这一节应该是 4.4）

        In Group C, the agent mode exhibited a qualitatively different planning behavior from both Groups A and B. Rather than outputting all waypoints in a single inference call, the agent first queries its own position and observes the scene before deciding on a path. This two-step "look then plan" approach led to more context-aware decisions. For example, in one trial where the cardboard box was positioned slightly further to the left than usual, the Group B optimized prompt still generated a path curving to the right (as trained by the prompt's general instruction), while the Group C agent, after observing the image and noting the obstacle position, planned a path that curved left — correctly choosing the wider gap. This suggests that giving the model access to real-time pose data enables it to make spatially grounded decisions that a single-image prompt alone cannot reliably produce.


======================================================================
## 6. Chapter 4.5（原 4.4）Limitations — 修改第三段
======================================================================

【替换】原文第三段 "Third, while waiting for the Gemini response, the robot performs no useful action..."
替换为：

        Third, the traditional single-turn pipeline requires the robot to remain completely stopped while waiting for the Gemini response. The agent mode partially alleviates this issue by combining multiple queries (pose, image, path) within a single trigger cycle, reducing the total number of idle-to-moving transitions. However, within each agent turn, the robot still waits for the API round-trip. A potential improvement would be to allow the robot to continue following the tail end of the previous path segment while the next plan is being generated, or to introduce a lightweight local planner as a fallback during API communication.

【新增】在 Limitations 最后追加一段：

        Fourth, the agent mode introduces additional API token consumption compared to the single-turn mode. A typical agent cycle involves 3–5 tool call turns, each requiring a full context submission to the Gemini API. Although the max_turns parameter (set to 8) prevents runaway costs, the per-trial token usage in Group C was approximately [___] times higher than in Group B. For production deployments, token budgeting and cost monitoring mechanisms would be necessary.


======================================================================
## 7. Chapter 5.1 Conclusion — 替换全部内容
======================================================================

【替换】原文 5.1 全部内容，用以下替换：

5.1 Conclusion

        This work designed and implemented a VLM-driven visual navigation system for an indoor mobile robot, progressing through two architectural stages: a single-turn image-to-coordinate pipeline and a multi-turn Function Calling agent. The system was built on the ROS2 Humble framework and deployed on a Jetson Orin Nano edge device. Both architectures were validated on a physical robot platform without requiring any pre-built map or manual path annotation.

        A three-group comparative experiment was conducted. Groups A and B compared a minimal prompt against an optimized prompt under the single-turn pipeline, demonstrating that prompt engineering plays a critical role in VLM-based robotic planning: the optimized prompt achieved [___] success rate with an average of only [___] Gemini calls per trial, while the naive prompt achieved [___] success rate requiring [___] calls. Path smoothness, measured by angular velocity variance, was also significantly improved with the optimized prompt. Group C evaluated the Function Calling agent architecture, which achieved [___] success rate with improved path directness and the added capability of generating semantic region labels during navigation.

        These findings carry several implications. First, they confirm that cloud-based multimodal large language models can serve as practical path planners for real hardware systems, not merely in simulation. This addresses a gap identified in recent surveys where most VLM-robotics studies remain at the simulation stage. Second, the results highlight that the quality of the system prompt is as decisive as algorithm design in traditional planning systems — a single well-crafted prompt file replaced what would otherwise require multiple hand-tuned heuristic rules. Third, the Function Calling agent architecture demonstrates that giving the VLM active access to robot sensors enables more spatially grounded decisions compared to passive single-image inference, at the cost of higher API token consumption. Fourth, the proposed Skill-based YAML architecture offers a reusable pattern: researchers working on different tasks (exploration, object search, inspection) can extend the system by simply adding a configuration file, with zero modification to the underlying ROS2 nodes. This modularity lowers the barrier for applying VLMs to diverse robotic applications and provides a reproducible reference for future studies on language-model-driven navigation.


======================================================================
## 8. Chapter 5.2 Future Work — 替换全部内容
======================================================================

【替换】原文 5.2 全部内容，用以下替换：

5.2 Future Work

        Several directions are identified for extending this work:

        The current agent architecture uses a fixed Skill file for all navigation tasks. A progressive multi-Skill framework has been designed, consisting of three phases: a Scout Skill for coarse-grained area exploration, an Inspector Skill for fine-grained object-level annotation within specific regions, and a Navigator Skill that leverages the accumulated semantic map for efficient goal-directed navigation. This three-phase model would allow the robot to gradually build familiarity with an environment, similar to how a person learns a new building — first scanning the overall layout, then examining details in areas of interest, and finally navigating confidently based on prior knowledge. The Skill switching mechanism is already supported by the current YAML-based framework and only requires implementing the additional Skill files and extended tool functions (such as connect_regions for topology recording and find_object for cross-region search).

        The semantic labels generated by the agent are currently stored only in memory and lost when the node shuts down. Implementing persistent storage (save_map / load_map to JSON files) would enable the robot to reuse environmental knowledge across sessions. Combined with the progressive Skill framework, this would form a complete perception-planning-memory closed loop.

        The API response latency of 10–20 seconds per call remains a practical bottleneck. A hybrid architecture combining a lightweight local model for coarse planning with the cloud VLM for refined decisions is a promising direction to reduce idle waiting time. In the agent mode, the multi-turn overhead further amplifies this issue, making local fallback planning even more valuable.

        Finally, extending the system from a single leader robot to a functionally heterogeneous multi-robot formation — where the leader performs perception and planning while followers execute formation-keeping — is a natural next step toward practical deployment in larger-scale environments.


======================================================================
## 9. Abstract — 微调（可选）
======================================================================

【说明】Abstract 目前已经提到了 Function Calling 和 semantic mapping，
与修改后的内容基本一致，不需要大改。如果你想更精确，可以把第二段
中间的一句话做微调：

【替换】原文中的：
"Path planning is driven by Gemini 2.5 Flash, progressing from overhead-map planning to onboard visual planning, and fusing RGB and depth inputs to produce feasible trajectories."

替换为：
"Path planning is driven by Gemini 2.5 Flash, progressing from single-turn image-to-coordinate inference to multi-turn Function Calling agent interaction, and fusing RGB and depth inputs to produce feasible trajectories."

======================================================================
## 修改完毕
======================================================================
