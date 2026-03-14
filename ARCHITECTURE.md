For AI coding agents. This is the task we need to accomplish in this workspace and the design concept for each step. Read before making changes. 

## 1. Overview

All the goals that this workspace aims to achieve and each of the work steps are written in 'PROCESS.md'. You can refer to this file to obtain the current work goals, and also refer to 'MEMORY.md' to get the current work progress. Only when I set the goal for the next stage of work can I start the next stage in 'PROCESS.md'.

When generating the required code, here are all the ROS2-related topics that you need to use or will use in the future. If there are any topics that you think might be needed but I haven't provided, please ask me according to the "Ask if unsure" principle mentioned in the copliot-instruction.md. Below is the structure of the workspace and the required topics.

## 2. ROS2 Topics
The topic for obtaining the current position of the robot:
    - 'PoseStamped, /vrpn_mocap/rm_0_Test/pose' (include:position and orientation)
    - 'PoseStamped, /agent0/gps'(include: header and point x,y,z)

    Because these two sense would not be used at the same time, These two topics should be subscribed at the same time, /vrpn_mocap/rm_0_Test/pose is the topic from the Mocap which is gained in the real world, and /agent0/gps is the topic from the Webots which is gained in the virtual world. 

The topic for sending the speed to the robot:
    - 'Twist, /agent0/cmd_vel' (include: linear x,y,z and angular x,y,z)
    - 'Twist, /cmd_vel' (include: linear x,y,z and angular x,y,z)
    
The topic for sending the path points to the robot:
    - 'Path, /path' (include: header and poses)

## 3. ROS2 Nodes
(here is the brief description for the function of each node, more details will be added later or in the 'PROCESS.md', some nodes are not implemented or declared here, blind them for now)

- 'Generate_Path': This node is responsible for generating path points based on the starting point, ending point, and the desired path shape. Subscribe the 'agent0/gps' and 'vrpn_mocap/rm_0_Test/pose' topic ONCE to obtain the starting point. It will publish the generated path points to the '/path' topic.

- 'Track_Path': This node is responsible for receiving the path points from the '/path' topic and controlling the robot's movement based on those points. It will subscribe to the '/path' topic and store it as desired path. Subscribe the 'agent0/gps' and 'vrpn_mocap/rm_0_Test/pose' topic to obtain the current position of the robot, and then calculate the deviation from the ideal path points. Based on the deviation, it will publish speed commands to the '/agent0/cmd_vel' or '/cmd_vel' topic to control the robot's movement.

- 'Image_Conversion': This node is responsible for converting the pixel-coordinates path points received from Gemini into actual path points in meters. It will subscribe to the topic where the pixel-coordinates path points are published and then publish the converted path points to the '/path' topic for tracking by the 'Track_Path' node.

- 'rgb_image': This node is responsible for capturing RGB images from the depth camera, but it haven't been implemented yet. So blind it for now.

- 'depth_image': This node is responsible for capturing depth images from the depth camera, but it haven't been implemented yet. So blind it for now.

## 4. Workspace Structure
The workspace is structured as follows:
src/
    (that is the source file for workspace, you can write your code here, but make sure to follow the design concept in 'PROCESS.md' and the rules in 'copliot-instruction.md')

src_learn/
    (Ignore it, that is only for me to learn, not for you to change or write)

Past_code/
    (Ignore it, that is the past code for workspace, not for you to change or write)

build/
    (that is the building file for workspace, not for you to change or write, but you can refer it while there is a bug our build fail)

install/
    (that is the installing file for workspace, not for you to change or write, but you can refer it while there is a bug our build fail)

log/
    (that is the log file for workspace, not for you to change or write, but you can refer it while there is a bug our build fail)

quick_start_lattice.sh && test_lattice_planner.sh
    (Ignore it, that is only for me to learn, not for you to change or write)

