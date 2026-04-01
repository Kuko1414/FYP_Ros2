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

## 3. Extension Steps (Post-Core Pipeline)

The following steps extend the system's capabilities beyond the core VLM-based path planning pipeline. They introduce on-device perception models to reduce Gemini API dependency, improve coordinate accuracy, and enable persistent spatial understanding. The target deployment platform is **Jetson Orin Nano** (40 TOPS INT8, 8GB shared LPDDR5), so all local models must be lightweight and TensorRT-optimized.

### Step 7: Real-Time Traversable Area Segmentation (Priority: HIGH)

**Goal:** Give the robot continuous, frame-by-frame understanding of *where it can drive*, eliminating the current "blind until stopped" behavior. This is the single most impactful extension because it enables local fallback path planning, reducing the robot's dependency on Gemini API calls and eliminating the 1-5 second "thinking freeze" when encountering obstacles.

**Problem Being Solved:**
  - Currently the robot has no real-time spatial awareness during driving. It only "looks" when `Track_Path` detects a deviation > 0.5m and triggers the Gemini API.
  - The Gemini API call takes 1-5 seconds (network latency), during which the robot is completely stopped.
  - Gemini receives a raw RGB image and must infer 3D spatial layout from 2D — this is inherently imprecise.

**Model Selection:**
  - Architecture: **BiSeNetV2** or **MobileNetV3-Seg** (~1-3M parameters)
  - Input: RGB image from `/depth_cam/rgb0/image_raw` (640×400×3)
  - Output: Per-pixel class label map (640×400×1), with classes:
    - `0 = traversable ground` (floor, flat road, grass)
    - `1 = obstacle` (furniture, walls, people, boxes)
    - `2 = boundary/edge` (steps, curbs, drop-offs)
    - `3 = unknown/uncertain`
  - Inference: TensorRT FP16 on Jetson Orin Nano, expected **8-15ms per frame**
  - Training data sources:
    - Public datasets: Cityscapes, ADE20K, SUN RGB-D (transfer learning base)
    - Self-collected: Record RGB rosbags from the robot's camera in target environments, use SAM (Segment Anything Model) for semi-automatic annotation — only human confirmation/correction needed
    - Simulation: Webots/Gazebo can directly output semantic label images at zero annotation cost

**New ROS2 Node: `semantic_seg_node`**
  - Package: Create new package `semantic_perception` in `src/`
  - Subscriptions:
    - `/depth_cam/rgb0/image_raw` (sensor_msgs/Image) — RGB input for segmentation
  - Publications:
    - `/semantic_mask` (sensor_msgs/Image, mono8) — per-pixel class labels for downstream nodes
    - `/traversable_path` (nav_msgs/Path) — locally planned fallback path based on largest traversable region
    - `/semantic_overlay` (sensor_msgs/Image, bgr8) — color-coded visualization overlay for debugging and optional Gemini input enhancement
  - Parameters:
    - `model_path` (string): path to TensorRT engine file
    - `confidence_threshold` (float, default 0.7): minimum confidence to classify a pixel
    - `traversable_width_threshold` (float, default 0.3): minimum traversable corridor width in meters before triggering pre-emptive Gemini call

**Integration with Existing Nodes:**

  1. **`track_path` (modified):** Subscribe to `/traversable_path` as a fallback. When deviation > 0.5m AND Gemini has not yet responded, switch to the local traversable path instead of stopping completely. When Gemini returns a better path, seamlessly switch to it.

  2. **`image_to_llm_node` (modified):** Optionally subscribe to `/semantic_overlay` and send the annotated image (with traversable areas highlighted in green, obstacles in red) to Gemini instead of raw RGB. This dramatically improves Gemini's spatial understanding. The prompt should be updated to reference the color coding: *"Green regions are confirmed traversable ground. Red regions are obstacles. Plan a path within the green regions."*

  3. **Pre-emptive Triggering:** Instead of waiting for `Track_Path` to detect a 0.5m deviation, `semantic_seg_node` continuously monitors the traversable corridor width ahead. When it narrows below a threshold (e.g., < 0.3m passable width at 1.0m ahead), it proactively calls `/trigger_llm_plan` while the robot is still moving (just decelerating). This overlaps the API latency with driving time, so the perceived "thinking time" approaches zero.

**Architecture Change — From Serial to Parallel:**
  ```
  CURRENT (Serial):
    Driving → Hit obstacle → STOP → Call Gemini (3s) → Get path → Resume
    Total pause: 3-5 seconds

  AFTER Step 7 (Parallel):
    Driving → Seg detects narrowing corridor → Decelerate + Call Gemini async
         ↓ (simultaneously)
    Follow local traversable_path as fallback
         ↓ (when Gemini responds)
    Seamlessly switch to Gemini's optimized path
    Total pause: ~0 seconds (or minimal deceleration)
  ```

---

### Step 8: Depth Completion / Denoising CNN (Priority: MEDIUM)

**Goal:** Improve the accuracy and reliability of the pixel-to-3D coordinate conversion in `image_conversion.py` by replacing the raw (noisy, hole-ridden) depth image with a CNN-refined dense depth map.

**Problem Being Solved:**
  - The current `image_conversion` node uses raw depth values with a 5×5 median filter. This is a basic mitigation but cannot handle:
    - Large depth holes (reflective surfaces, transparent objects, out-of-range areas) — if the entire 5×5 patch is zero, the point is discarded
    - Flying pixels at object edges — depth values "jump" between foreground and background, creating phantom 3D points in mid-air
    - Depth noise jitter — ±2-3cm random fluctuation on static scenes causes the output `/path` to wobble, making `Track_Path` oscillate
    - Accuracy degradation at distance — structured light / ToF cameras lose precision beyond 2m (±5-10cm error)
  - These issues directly degrade the quality of the 3D path points sent to `Track_Path`, causing tracking instability.

**Model Selection:**
  - Architecture: Small **U-Net** (~1-2M parameters), encoder-decoder with skip connections
  - Input: 4-channel tensor — RGB (3ch) + raw depth (1ch), resolution 640×400
  - Output: 1-channel refined depth map, same resolution, dense (no holes), denoised, with clean edges
  - Why RGB is needed: RGB provides texture and edge cues that allow the CNN to infer "this region looks like a continuous flat surface (e.g., floor), so the depth should transition smoothly" — enabling intelligent hole-filling that pure depth filtering cannot achieve
  - Inference: TensorRT FP16 on Jetson Orin Nano, expected **5-10ms per frame**
  - Training data sources:
    - Public datasets: NYU Depth V2 (indoor), KITTI (outdoor), ScanNet
    - Self-collected: Record RGB+Depth rosbags, generate ground truth via multi-frame fusion or high-accuracy mode
    - Simulation: Webots/Gazebo renders perfect depth maps as GT; artificially add noise/holes as input — zero-cost paired training data

**Integration Point — Minimal Code Change:**
  - This model is integrated **inside the existing `image_conversion` node** (in `image_to_llm` package), not as a separate node.
  - Only the `depth_callback` method needs modification:
    ```python
    def depth_callback(self, msg):
        raw_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        # NEW: CNN refines the depth map before any downstream use
        self.latest_depth = self.depth_completion_model.infer(self.latest_rgb, raw_depth)
        # All existing pixel_callback code (pinhole model reprojection) remains unchanged
    ```
  - The `depth_completion_model` is loaded once at node startup from a TensorRT engine file specified by a new parameter `depth_model_path`.

**Expected Improvement:**

  | Metric | Current (5×5 median) | After Step 8 (CNN) |
  |--------|---------------------|-------------------|
  | Depth hole rate | ~5-15% | <1% |
  | Edge flying pixels | Present | Mostly eliminated |
  | Depth noise | ±2-3cm | ±0.5-1cm |
  | 3D coordinate accuracy | Moderate | Significantly improved |
  | Path point stability | Jittery | Smooth and stable |
  | Conversion failure rate | Occasional (all-zero patches) | Near zero |

---

### Step 9: Persistent Semantic Map (Priority: LOW — depends on Step 6 TF2 + Step 7 Segmentation)

**Goal:** Accumulate per-frame semantic segmentation results into a global 2D occupancy/semantic grid map, giving the robot persistent memory of explored areas and known obstacle locations.

**Prerequisites:** Step 6 (TF2 coordinate transforms) and Step 7 (semantic segmentation) must both be completed first. TF2 is essential because each frame's semantic labels are in `depth_camera_link` frame and must be projected into the global `map`/`odom` frame to accumulate correctly.

**How It Works:**
  1. For each frame, take the semantic mask from Step 7 (`/semantic_mask`) and the refined depth from Step 8 (or raw depth if Step 8 is not yet done).
  2. For each pixel classified as `traversable` or `obstacle`, use the depth value + camera intrinsics to compute the 3D point in `depth_camera_link` frame.
  3. Use TF2 (`depth_camera_link → map`) to transform the 3D point into the global `map` frame.
  4. Project the 3D point onto a 2D grid (bird's-eye view) and update the corresponding cell's semantic label with a Bayesian update (to handle noise and conflicting observations over time).

**New ROS2 Node: `semantic_map_node`**
  - Package: `semantic_perception` (same package as Step 7)
  - Subscriptions:
    - `/semantic_mask` (sensor_msgs/Image) — per-pixel class labels from Step 7
    - `/depth_cam/depth0/image_raw` (sensor_msgs/Image) — depth for 3D projection
    - `/depth_cam/rgb0/camera_info` (sensor_msgs/CameraInfo) — intrinsics
    - `/tf`, `/tf_static` — coordinate transforms from Step 6
  - Publications:
    - `/semantic_map` (nav_msgs/OccupancyGrid) — global semantic grid map, compatible with RViz2 visualization
    - `/semantic_map_image` (sensor_msgs/Image) — color-coded bird's-eye view for debugging
  - Parameters:
    - `map_resolution` (float, default 0.05): meters per grid cell
    - `map_size` (float, default 10.0): map side length in meters
    - `update_rate` (float, default 2.0): Hz, how often to publish the accumulated map

**Use Cases:**
  - **Global path planning:** Instead of only reacting to what's directly in front, the robot can plan paths around previously-seen obstacles even when they're no longer in the camera's field of view.
  - **Gemini context enrichment:** The semantic map can be rendered as a top-down image and included in the Gemini prompt: *"Here is a bird's-eye semantic map of the explored area. The robot is at ☆. Plan a global path to the destination."*
  - **Exploration:** The map reveals unexplored regions, enabling autonomous exploration behaviors.

---

### Step 10: Full System Integration and Optimization (Priority: after Steps 7-9)

**Goal:** Integrate all extension components into a cohesive, robust system and optimize for real-time performance on Jetson Orin Nano.

**Key Tasks:**
  1. **Launch file consolidation:** Create a unified launch file that starts all nodes (`track_path`, `generate_path`, `image_to_llm_node`, `image_conversion`, `semantic_seg_node`, `semantic_map_node`) with correct parameter configurations and topic remappings.
  2. **GPU memory management:** Ensure TensorRT engines for semantic segmentation (Step 7) and depth completion (Step 8) share GPU memory efficiently. On Jetson's unified memory architecture, monitor total usage stays under ~5GB to leave headroom for ROS2 and system processes.
  3. **Latency profiling:** Measure end-to-end latency from obstacle detection to path execution. Target: <100ms for local fallback path, <3s for Gemini-enhanced path.
  4. **Failure mode handling:** Define behavior when models produce low-confidence outputs, when Gemini API times out, or when depth camera data is unavailable.
  5. **ROS2 QoS tuning:** Ensure all high-frequency topics (semantic mask, depth, cmd_vel) use appropriate QoS profiles (BEST_EFFORT for sensor data, RELIABLE for path commands).

---

## 4. Deployment Notes

**Target Hardware:** Jetson Orin Nano (8GB)
  - GPU: 1024-core Ampere, 32 Tensor Cores, 40 TOPS INT8
  - All CNN models must be exported as: PyTorch → ONNX → TensorRT (FP16)
  - Use `trtexec` or `torch2trt` for conversion on the Jetson device itself
  - Expected total CNN inference budget: ~20-30ms/frame (both models combined), well within 10Hz control loop

**Model Training Pipeline:**
  - Train on a desktop GPU (e.g., RTX 3060+) using PyTorch
  - Export to ONNX with fixed input dimensions (640×400)
  - Convert to TensorRT engine on the Jetson Orin Nano (engines are hardware-specific)
  - Store engine files in a `models/` directory at workspace root (add to `.gitignore`)

**Recommended Directory Structure Extension:**
  ```
  src/
      semantic_perception/        # NEW package for Steps 7, 9
          semantic_perception/
              semantic_seg_node.py
              semantic_map_node.py
              model_utils.py       # TensorRT loading, pre/post-processing
          models/                  # .onnx source models (tracked in git)
          resource/
          ...
  models/                          # TensorRT .engine files (NOT in git, device-specific)
      bisenetv2_fp16.engine
      depth_unet_fp16.engine
  data/
      images/                      # runtime captured images
      training/                    # training datasets (NOT in git)
  ```
