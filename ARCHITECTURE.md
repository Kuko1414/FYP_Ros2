For AI coding agents. This is the task we need to accomplish in this workspace and the design concept for each step. Read before making changes. 

## 1. Overview

All the goals that this workspace aims to achieve and each of the work steps are written in 'PROCESS_update.md'. You can refer to this file to obtain the current work goals. Only when I set the goal for the next stage of work can I start the next stage.

When generating the required code, here are all the ROS2-related topics that you need to use or will use in the future. 

## 2. ROS2 Topics
The topic for obtaining the current position of the robot:
    - 'PoseStamped, /vrpn_mocap/rm_0_Test/pose' (include:position and orientation)
    - 'PointStamped, /agent0/gps'(include: header and point x,y,z)

The topic for sending the speed to the robot:
    - 'Twist, /agent0/cmd_vel' (include: linear x,y,z and angular x,y,z)
    - 'Twist, /cmd_vel' (include: linear x,y,z and angular x,y,z)
    
The topic for obtaining the robot's orientation (yaw):
    - 'Vector3Stamped, /imu/rpy/filtered' (include: header with frame_id 'imu_link', vector.x=Roll, vector.y=Pitch, vector.z=Yaw, all in radians. This is the filtered RPY output, vector.z can be used directly as the robot's heading angle)

The topic for sending the path points to the robot:
    - 'Path, /path' (include: header and poses)

The topic for depth camera and vision (Added for Step 3 & Step 4):
    - 'Image, /depth_cam/rgb0/image_raw' (RGB image from depth camera)
    - 'CameraInfo, /depth_cam/depth0/camera_info' (camera_info for pixel to physical conversion)
    - 'Image, /depth_cam/depth0/image_raw' (the depth image data for obstacle detection and path planning)
    - 'CameraInfo, /depth_cam/rgb0/camera_info' (RGB camera intrinsic parameters. image_conversion node should subscribe to this to dynamically obtain fx, fy, cx, cy instead of hardcoding)
    - 'PointCloud2, /depth_cam/depth0/points' (3D point cloud from depth camera. Can be used for obstacle detection and spatial reasoning, more powerful than per-pixel depth lookup)
    - 'Image, /depth_cam/ir0/image_raw' (infrared image from depth camera)

The topic for odometry and coordinate transforms (needed for Step 6 TF2):
    - 'Odometry, /odom' (include: pose with position+orientation, twist with linear+angular velocity. Core data source for odom frame, can also serve as position feedback for Track_Path)
    - 'TFMessage, /tf' (dynamic coordinate transform tree, essential for Step 6 depth_camera_link → odom/map transform)
    - 'TFMessage, /tf_static' (static coordinate transforms, e.g. camera mount position relative to base_link)

The topic for getting pose of the robot from imu:
    - 'Imu, /agent0/imu' (its imu for webots robot, include: orientation, angular velocity, linear acceleration)
    - 'Imu, /imu' (real robot raw IMU data)

The topic for LiDAR (2D laser scanner):
    - 'LaserScan, /scan_raw' (2D laser scan from LiDAR. frame_id: lidar_frame. 360° coverage, 504 rays, 0.716° resolution, range: 0.06m~25m. Used by track_path for obstacle detection in the forward FOV)

## 3. ROS2 Nodes

- 'Generate_Path': （Not use）This node is responsible for generating path points based on the starting point, ending point, and the desired path shape. It will publish the generated path points to the '/path' topic. (Optimization: To prevent blocking the ROS2 executor, user inputs for coordinates and shapes should be handled via a non-blocking separate thread, or ideally designed as a ROS2 Service/Action).

- 'Track_Path': This node receives the path points from the '/path' topic. It stores it as the desired path, calculates the deviation from the ideal path points continuously, and applies a feedback control algorithm (e.g., Pure Pursuit or continuous PID) to publish speed commands. If deviation is dangerously large (>0.5m), it stops the vehicle and requests a replan.

- 'Image_Conversion': This node converts pixel-coordinates from the LLM (Gemini) into actual path points in meters. (Optimization: It MUST subscribe to the pixel coordinates AND the depth camera topics ('depth_image' and 'camera_info') to perform accurate 2D to 3D inverse projection mapping), then it publishes the converted path points to the '/path' topic.

- 'image_to_llm': This node is send and response for the Gemini API. It sends the RGB image captured from the front camera to Gemini and receives the path points in normalized coordinates. It then publishes these pixel coordinates to a topic ('/llm_pixels') for the 'Image_Conversion' node to process.

- 'depth_image': This node is responsible for capturing depth images from the depth camera. (blind for now)

## 4. Workspace Structure
The workspace is structured as follows:
src/
    (source files for workspace, strictly no runtime generated data like images should be saved here to avoid polluting source code)

data/
    (Suggested folder: to store runtime data such as captured images for Gemini during obstacle avoidance, avoiding source directory pollution)

src_learn/
    (Ignore it, purely for learning)

Past_code/
    (Ignore it)

build/ install/ log/
    (standard ROS2 directories)
