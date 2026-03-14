For AI coding agents. This is a record of each change made to the workspace content. You can refer to what changes we have previously made. If you want to make a change, please first refer to ARCHITECTURE.md 

## Mar.14, 2026
- Added the overview of the workspace, the ROS2 topics, the ROS2 nodes, and the workspace structure in ARCHITECTURE.md. But there is no functional code in this stage, so the MEMORY.md is still empty.
- Implemented Step 1 of PROCESS.md. Created `my_robot_planner` package in `src`. Added `Generate_Path` node to publish paths based on user input (shape, target coords) using starting pose. Added `Track_Path` node to subscribe to the generated paths and publish `cmd_vel` to control the robot movement. Both nodes align with ARCHITECTURE.md topic specifications.