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
