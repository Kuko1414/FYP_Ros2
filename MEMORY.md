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
