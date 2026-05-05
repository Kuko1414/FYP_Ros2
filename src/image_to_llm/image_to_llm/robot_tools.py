"""
RobotTools：Gemini Function Calling 的工具执行层。

核心逻辑：内部订阅 ROS2 Topics 缓存最新数据，对外提供简洁的查询/操作函数。
所有函数只通过 ROS2 标准接口与外部节点通信，不 import 任何其他节点的代码。

【重要】每个 public 方法对应 Gemini 可调用的一个 Tool。
新增工具时，需要同步在 tool_schemas.py 中添加对应的 FunctionDeclaration。
"""

import math
import threading
import time
import numpy as np
import cv2
from PIL import Image as PILImage

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from cv_bridge import CvBridge


class RobotTools:
    """Gemini 可调用的工具函数集合。

    所有工具函数返回 dict（JSON 可序列化），
    唯一例外是 get_front_image() 返回 PIL.Image。
    """

    def __init__(self, ros_node):
        """
        Args:
            ros_node: 宿主 ROS2 节点实例（agent_node），用于创建订阅和发布。
        """
        self.node = ros_node
        self.bridge = CvBridge()

        # ---- 缓存的最新数据 ----
        self._latest_odom = None       # Odometry 消息
        self._latest_yaw = None        # float, 弧度
        self._latest_rgb = None        # numpy BGR
        self._latest_depth = None      # numpy float32, 单位米

        # ---- 线程事件：用于等待图像就绪 ----
        self._rgb_event = threading.Event()

        # ---- 语义标签存储（内存） ----
        self._semantic_labels = {}

        # ---- QoS ----
        # 注意：Orbbec Gemini 2L 的相机驱动发布 RELIABLE QoS，
        # 但多个 RELIABLE 订阅者会导致 USB 带宽饱和/DDS 发送队列阻塞，
        # 使相机驱动完全停止发布。改用 BEST_EFFORT 订阅可以避免此问题
        # （RELIABLE publisher → BEST_EFFORT subscriber 是兼容的）。
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # ---- 后台订阅（只缓存，不做业务逻辑）----
        # 话题名通过参数从 agent_node 传入
        odom_topic = self.node.declare_parameter(
            'odom_topic', '/odom').value
        rgb_topic = self.node.declare_parameter(
            'rgb_topic', '/camera/color/image_raw').value
        depth_topic = self.node.declare_parameter(
            'depth_image_topic', '/camera/depth/image_raw').value

        self.node.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.node.create_subscription(Image, rgb_topic, self._rgb_cb, sensor_qos)
        self.node.create_subscription(Image, depth_topic, self._depth_cb, sensor_qos)

        # ---- 发布器 ----
        self.goal_pub = self.node.create_publisher(PoseStamped, '/goal_point', 10)
        self.cmd_pub1 = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_pub2 = self.node.create_publisher(Twist, '/agent0/cmd_vel', 10)

        # ---- 任务状态 ----
        self._task_finished = False

        self.node.get_logger().info("[RobotTools] 工具层已初始化，订阅话题就绪")

    # ================================================================
    #  内部回调（只缓存数据，不做任何业务逻辑）
    # ================================================================

    def _odom_cb(self, msg):
        self._latest_odom = msg
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._latest_yaw = math.atan2(siny, cosy)

    def _rgb_cb(self, msg):
        try:
            self._latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._rgb_event.set()  # 通知等待线程：图像已就绪
        except Exception as e:
            self.node.get_logger().warn(
                f"[RobotTools] RGB 回调 cv_bridge 转换失败: {e}")

    def _depth_cb(self, msg):
        try:
            if msg.encoding in ('16UC1', 'mono16'):
                raw = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                self._latest_depth = raw.astype(np.float32) / 1000.0
            elif msg.encoding == '32FC1':
                self._latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception:
            pass

    # ================================================================
    #  Gemini 可调用的工具函数
    # ================================================================

    def get_robot_pose(self) -> dict:
        """获取机器人当前在 odom 坐标系下的位置和朝向。

        Returns:
            {"x": float, "y": float, "yaw_deg": float}
            yaw_deg: 0=正东(x+), 逆时针为正（90=正北, -90=正南）
        """
        if self._latest_odom is None:
            return {"error": "odom 数据尚未就绪，请稍后重试"}
        p = self._latest_odom.pose.pose.position
        return {
            "x": round(p.x, 3),
            "y": round(p.y, 3),
            "yaw_deg": round(math.degrees(self._latest_yaw), 1)
        }

    def _stop_robot(self):
        """发布零速度指令，立即停止机器人运动。"""
        stop_cmd = Twist()
        self.cmd_pub1.publish(stop_cmd)
        self.cmd_pub2.publish(stop_cmd)

    def get_front_image(self) -> PILImage.Image:
        """获取前方摄像头的最新 RGB 图像。

        调用时会先发布零速度指令让机器人停车，确保拍照时图像稳定。

        Returns:
            PIL.Image 对象（可直接传给 Gemini multimodal API）
            或 None（如果图像未就绪）

        【特殊处理】agent_node 中会将此返回值作为图像 Part 传入 Gemini，
        而非普通的 function_response 文本。
        """
        # 先停车，确保拍照时机器人静止，避免运动模糊
        self._stop_robot()
        time.sleep(0.3)  # 短暂等待，让机器人完全停稳

        # 如果图像还没到，使用 threading.Event 分轮等待，
        # 总共最多等 3 轮 × 5 秒 = 15 秒，覆盖 DDS 发现延迟。
        # 【注意】不能用 rclpy.spin_once()！因为 get_front_image 是在
        # service 回调（plan_callback）内被调用的，而 service 回调本身运行
        # 在 MultiThreadedExecutor 的线程中。spin_once 会尝试再次调度同一
        # 节点的回调，但 executor 已经在 spin 了，导致死锁——_rgb_cb 永远
        # 不会被执行，图像永远收不到。
        # 改用 threading.Event：_rgb_cb 收到第一帧图像后 set() 事件，
        # 这里只需 wait() 即可，executor 的其他线程会正常触发回调。
        max_retries = 3
        wait_per_retry = 5.0  # 每轮等待秒数
        for attempt in range(1, max_retries + 1):
            if self._latest_rgb is not None:
                break
            self.node.get_logger().info(
                f"[RobotTools] RGB 图像未就绪，等待中... "
                f"(第 {attempt}/{max_retries} 轮，最多 {wait_per_retry}s)")
            self._rgb_event.clear()
            self._rgb_event.wait(timeout=wait_per_retry)

        if self._latest_rgb is None:
            self.node.get_logger().warn(
                f"[RobotTools] RGB 图像在 {max_retries * wait_per_retry:.0f} 秒内"
                f"仍未收到，请检查相机驱动是否正常运行")
            return None
        self.node.get_logger().info(
            "[RobotTools] RGB 图像已获取")
        rgb = cv2.cvtColor(self._latest_rgb, cv2.COLOR_BGR2RGB)
        return PILImage.fromarray(rgb)

    def get_obstacle_distance(self) -> dict:
        """获取前方中央区域最近障碍物的距离。

        使用深度图中央 ROI（水平中间 1/3，垂直 30%-80%）的最小有效深度值。

        Returns:
            {"min_distance_m": float, "has_obstacle": bool}
        """
        if self._latest_depth is None:
            return {"error": "深度图数据尚未就绪"}

        h, w = self._latest_depth.shape
        row_top = int(h * 0.3)
        row_bottom = int(h * 0.8)
        col_left = int(w * 0.33)
        col_right = int(w * 0.67)

        roi = self._latest_depth[row_top:row_bottom, col_left:col_right]
        valid = roi[(roi > 0.1) & (roi < 5.0)]

        if len(valid) == 0:
            return {
                "min_distance_m": None,
                "has_obstacle": False,
                "note": "ROI 内无有效深度数据"
            }

        min_dist = float(np.min(valid))
        return {
            "min_distance_m": round(min_dist, 3),
            "has_obstacle": min_dist < 0.8
        }

    def publish_goal_relative(self, direction_deg: float, distance_m: float) -> dict:
        """根据相对方向和距离发布目标点（推荐使用此函数代替 publish_goal）。

        Gemini 只需判断"目标在左边还是右边、大约多远"，坐标计算由代码完成。
        比 publish_goal(x, y) 更不容易出错。

        Args:
            direction_deg: 相对于机器人当前朝向的方向（度）。
                           0 = 正前方，正值 = 左偏，负值 = 右偏。
                           例如：-30 表示右前方 30°，+20 表示左前方 20°。
            distance_m: 目标距离（米），建议 2-3m。

        Returns:
            {"success": bool, "goal_x": float, "goal_y": float,
             "direction_deg": float, "distance_m": float}
        """
        if self._latest_odom is None or self._latest_yaw is None:
            return {"error": "odom 数据尚未就绪，无法计算目标坐标"}

        p = self._latest_odom.pose.pose.position
        yaw = self._latest_yaw

        # 计算全局目标角度
        target_angle = yaw + math.radians(direction_deg)
        goal_x = p.x + distance_m * math.cos(target_angle)
        goal_y = p.y + distance_m * math.sin(target_angle)

        # 发布目标点
        now_stamp = self.node.get_clock().now().to_msg()
        goal_msg = PoseStamped()
        goal_msg.header.stamp = now_stamp
        goal_msg.header.frame_id = 'odom'
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)
        self.node.get_logger().info(
            f"[RobotTools] 已发布相对目标点: 方向={direction_deg:.0f}°, "
            f"距离={distance_m:.1f}m → odom ({goal_x:.2f}, {goal_y:.2f})")
        return {
            "success": True,
            "goal_x": round(goal_x, 3),
            "goal_y": round(goal_y, 3),
            "direction_deg": round(direction_deg, 1),
            "distance_m": round(distance_m, 2),
        }

    def label_region(self, name: str, x_min: float, y_min: float,
                     x_max: float, y_max: float) -> dict:
        """为 odom 坐标系下的一个矩形区域添加语义标签。

        Args:
            name: 区域名称（如 "workspace_1", "corridor", "door"）
            x_min, y_min: 矩形左下角坐标（odom，米）
            x_max, y_max: 矩形右上角坐标（odom，米）
        """
        self._semantic_labels[name] = {
            "x_min": x_min, "y_min": y_min,
            "x_max": x_max, "y_max": y_max
        }
        self.node.get_logger().info(
            f"[RobotTools] 语义标签 '{name}': "
            f"({x_min:.2f},{y_min:.2f})-({x_max:.2f},{y_max:.2f})")
        return {"success": True, "total_labels": len(self._semantic_labels)}

    def get_semantic_labels(self) -> dict:
        """获取所有已标注的语义区域标签。

        Returns:
            {"labels": dict, "count": int}
        """
        return {
            "labels": self._semantic_labels.copy(),
            "count": len(self._semantic_labels)
        }

    def get_current_region(self) -> dict:
        """查询机器人当前所在的语义区域。

        Returns:
            {"current_region": str 或 None, "pose": {x, y}}
        """
        if self._latest_odom is None:
            return {"error": "odom 数据尚未就绪"}

        p = self._latest_odom.pose.pose.position
        current_region = None
        for name, bounds in self._semantic_labels.items():
            if (bounds["x_min"] <= p.x <= bounds["x_max"] and
                    bounds["y_min"] <= p.y <= bounds["y_max"]):
                current_region = name
                break

        return {
            "current_region": current_region,
            "pose": {"x": round(p.x, 3), "y": round(p.y, 3)}
        }

    def rotate_robot(self, angle_deg: float, angular_speed: float = 0.5) -> dict:
        """原地旋转机器人指定角度（阻塞等待完成）。

        当前方被障碍物遮挡无法通过路径点绕行时，Gemini 可调用此函数
        让机器人原地转向，转完后再调用 get_front_image() 观察新方向。

        Args:
            angle_deg: 旋转角度（度）。正值=逆时针（左转），负值=顺时针（右转）。
            angular_speed: 旋转角速度（rad/s），默认 0.5。

        Returns:
            {"success": bool, "rotated_deg": float, "new_yaw_deg": float}
        """
        if self._latest_yaw is None:
            return {"error": "odom 数据尚未就绪，无法获取当前朝向"}

        # 记录起始 yaw，用于判断实际旋转量
        start_yaw = self._latest_yaw
        angle_rad = math.radians(angle_deg)
        target_yaw = start_yaw + angle_rad
        # 归一化到 [-pi, pi]
        target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))

        direction = 1.0 if angle_rad >= 0 else -1.0
        speed = abs(angular_speed) * direction

        self.node.get_logger().info(
            f"[RobotTools] 开始原地旋转 {angle_deg:.1f}°，"
            f"起始 yaw: {math.degrees(start_yaw):.1f}°，"
            f"目标 yaw: {math.degrees(target_yaw):.1f}°")

        cmd = Twist()
        cmd.angular.z = speed
        timeout = abs(angle_rad) / abs(angular_speed) + 5.0  # 超时保护
        start_time = time.monotonic()

        while time.monotonic() - start_time < timeout:
            # 手动触发 ROS2 回调，确保 odom 数据被更新
            # （service 回调线程中 sleep 会阻塞 executor 调度 odom 回调）
            try:
                rclpy.spin_once(self.node, timeout_sec=0.0)
            except Exception:
                pass  # 忽略 spin_once 的异常（如节点销毁）

            if self._latest_yaw is None:
                time.sleep(0.05)
                continue

            # 计算剩余角度差
            yaw_error = math.atan2(
                math.sin(target_yaw - self._latest_yaw),
                math.cos(target_yaw - self._latest_yaw))

            if abs(yaw_error) < math.radians(5.0):  # 5° 容差
                break

            self.cmd_pub1.publish(cmd)
            self.cmd_pub2.publish(cmd)
            time.sleep(0.05)

        # 停车（多发几次确保停下来）
        stop_cmd = Twist()
        for _ in range(5):
            self.cmd_pub1.publish(stop_cmd)
            self.cmd_pub2.publish(stop_cmd)
            time.sleep(0.02)

        # 等一下让 odom 更新到最终状态
        time.sleep(0.2)
        try:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception:
            pass

        final_yaw_deg = math.degrees(self._latest_yaw) if self._latest_yaw else 0.0
        actual_rotated = math.degrees(math.atan2(
            math.sin(self._latest_yaw - start_yaw),
            math.cos(self._latest_yaw - start_yaw))) if self._latest_yaw else 0.0

        self.node.get_logger().info(
            f"[RobotTools] 旋转完成，当前 yaw: {final_yaw_deg:.1f}°，"
            f"实际旋转: {actual_rotated:.1f}°")

        return {
            "success": True,
            "rotated_deg": round(actual_rotated, 1),
            "new_yaw_deg": round(final_yaw_deg, 1)
        }

    def get_depth_at_regions(self) -> dict:
        """获取深度图各区域的最近障碍物距离和对应的 odom 坐标。

        将深度图分为 3×3 九宫格区域，返回每个区域中最近有效深度点的
        距离和该点在 odom 坐标系下的近似位置。

        这帮助 Gemini 精确了解前方障碍物的分布：
        - 哪个方向有障碍物？距离多少米？
        - 哪个方向是空旷的（可以绕行）？

        九宫格布局（从机器人视角看）：
          top_left    | top_center    | top_right       (远处)
          mid_left    | mid_center    | mid_right       (中距)
          bottom_left | bottom_center | bottom_right    (近处)

        Returns:
            {
              "regions": {
                "top_left":     {"min_depth_m": float, "odom_x": float, "odom_y": float},
                "top_center":   {"min_depth_m": float, ...},
                ...
              },
              "summary": "左侧空旷(>3m)，中央有障碍物(0.6m)，右侧较窄(1.2m)"
            }
        """
        if self._latest_depth is None:
            return {"error": "深度图数据尚未就绪"}
        if self._latest_yaw is None or self._latest_odom is None:
            return {"error": "odom 数据尚未就绪"}

        h, w = self._latest_depth.shape
        # 机器人当前位姿
        pose = self._latest_odom.pose.pose.position
        robot_x, robot_y = pose.x, pose.y
        yaw = self._latest_yaw

        # 简化的相机内参（如果可用则用真实值，否则用估算）
        # 水平 FOV 约 60°，垂直 FOV 约 45°
        fx_est = w / (2.0 * math.tan(math.radians(30)))  # 估算 fx
        fy_est = h / (2.0 * math.tan(math.radians(22.5)))
        cx_est = w / 2.0
        cy_est = h / 2.0

        # 将图像分为 3×3 九宫格
        row_splits = [int(h * 0.2), int(h * 0.5), int(h * 0.8), h]
        col_splits = [0, int(w * 0.33), int(w * 0.67), w]
        region_names = [
            ["top_left", "top_center", "top_right"],
            ["mid_left", "mid_center", "mid_right"],
            ["bottom_left", "bottom_center", "bottom_right"],
        ]

        regions = {}
        summary_parts = []

        for row_idx in range(3):
            r_top = row_splits[row_idx]
            r_bottom = row_splits[row_idx + 1] if row_idx < 2 else row_splits[3]
            # 修正行索引
            if row_idx == 0:
                r_top = 0
                r_bottom = row_splits[0]
            elif row_idx == 1:
                r_top = row_splits[0]
                r_bottom = row_splits[1]
            else:
                r_top = row_splits[1]
                r_bottom = row_splits[2]

            for col_idx in range(3):
                c_left = col_splits[col_idx]
                c_right = col_splits[col_idx + 1]

                name = region_names[row_idx][col_idx]
                roi = self._latest_depth[r_top:r_bottom, c_left:c_right]
                valid = roi[(roi > 0.1) & (roi < 5.0)]

                if len(valid) == 0:
                    regions[name] = {
                        "min_depth_m": None,
                        "odom_x": None,
                        "odom_y": None,
                        "status": "无有效深度"
                    }
                    continue

                min_depth = float(np.min(valid))

                # 找到最近深度点的像素位置
                roi_valid_mask = (roi > 0.1) & (roi < 5.0)
                min_pos = np.unravel_index(
                    np.argmin(np.where(roi_valid_mask, roi, 999.0)),
                    roi.shape)
                # 转为全图像素坐标
                u = c_left + min_pos[1]
                v = r_top + min_pos[0]

                # 用针孔模型反投影到相机坐标系
                cam_z = min_depth
                cam_x = (u - cx_est) * cam_z / fx_est  # 右为正
                # cam_y 不需要（我们只关心 x-z 平面）

                # 相机坐标 → odom 坐标
                # cam_z = 前方距离, cam_x = 右偏距离
                odom_x = robot_x + cam_z * math.cos(yaw) - cam_x * math.sin(yaw)
                odom_y = robot_y + cam_z * math.sin(yaw) + cam_x * math.cos(yaw)

                regions[name] = {
                    "min_depth_m": round(min_depth, 2),
                    "odom_x": round(odom_x, 2),
                    "odom_y": round(odom_y, 2),
                }

        # 生成自然语言摘要（中间行最有用）
        left_d = regions.get("mid_left", {}).get("min_depth_m")
        center_d = regions.get("mid_center", {}).get("min_depth_m")
        right_d = regions.get("mid_right", {}).get("min_depth_m")

        def _desc(d):
            if d is None:
                return "无数据"
            elif d > 3.0:
                return f"空旷({d:.1f}m)"
            elif d > 1.0:
                return f"有空间({d:.1f}m)"
            elif d > 0.5:
                return f"较窄({d:.1f}m)"
            else:
                return f"有障碍物({d:.1f}m)"

        summary = f"左侧{_desc(left_d)}，中央{_desc(center_d)}，右侧{_desc(right_d)}"

        self.node.get_logger().info(f"[RobotTools] depth_regions: {summary}")
        return {
            "regions": regions,
            "summary": summary,
            "robot_pose": {"x": round(robot_x, 3), "y": round(robot_y, 3),
                           "yaw_deg": round(math.degrees(yaw), 1)}
        }

    def scan_surroundings(self, scan_angle: float = 45.0) -> dict:
        """环顾周围环境：自动左转拍照 + 右转拍照 + 转回原方向。

        当前方有障碍物需要决定绕行方向时，调用此函数可以一次获得
        左侧、前方、右侧三个方向的图像，帮助 Gemini 评估两侧空间大小。

        执行流程：
          1. 先拍前方图像（当前方向）
          2. 左转 scan_angle°，拍左侧图像
          3. 右转 2*scan_angle°（回到原方向再继续右转），拍右侧图像
          4. 左转 scan_angle°，回到原始朝向

        Args:
            scan_angle: 左右转向角度（度），默认 45°。

        Returns:
            {
              "success": bool,
              "scan_angle_deg": float,
              "front_image": PIL.Image,   # 前方图像
              "left_image": PIL.Image,    # 左侧图像
              "right_image": PIL.Image,   # 右侧图像
              "original_yaw_deg": float,
              "final_yaw_deg": float,
            }
            如果图像获取失败，对应的 image 字段为 None。

        【特殊处理】agent_node 中会将此返回值中的三张图像
        作为图像 Part 传入 Gemini，标注各自方向。
        """
        self.node.get_logger().info(
            f"[RobotTools] scan_surroundings: 开始环顾（±{scan_angle}°）")

        original_yaw_deg = (math.degrees(self._latest_yaw)
                            if self._latest_yaw is not None else 0.0)

        # --- 1. 拍前方图像 ---
        front_img = self._capture_current_image()
        self.node.get_logger().info("[RobotTools] scan: 前方图像已获取")

        # --- 2. 左转拍照 ---
        self.rotate_robot(scan_angle, angular_speed=0.6)
        time.sleep(0.5)  # 等待图像更新
        left_img = self._capture_current_image()
        self.node.get_logger().info("[RobotTools] scan: 左侧图像已获取")

        # --- 3. 右转拍照（转过 2*angle 到右侧）---
        self.rotate_robot(-2.0 * scan_angle, angular_speed=0.6)
        time.sleep(0.5)
        right_img = self._capture_current_image()
        self.node.get_logger().info("[RobotTools] scan: 右侧图像已获取")

        # --- 4. 回正（左转 angle 回到原方向）---
        self.rotate_robot(scan_angle, angular_speed=0.6)
        time.sleep(0.3)

        final_yaw_deg = (math.degrees(self._latest_yaw)
                         if self._latest_yaw is not None else 0.0)
        self.node.get_logger().info(
            f"[RobotTools] scan_surroundings 完成。"
            f"原始yaw={original_yaw_deg:.1f}°, 最终yaw={final_yaw_deg:.1f}°")

        return {
            "success": True,
            "scan_angle_deg": scan_angle,
            "front_image": front_img,
            "left_image": left_img,
            "right_image": right_img,
            "original_yaw_deg": round(original_yaw_deg, 1),
            "final_yaw_deg": round(final_yaw_deg, 1),
        }

    def _capture_current_image(self) -> PILImage.Image:
        """获取当前缓存的 RGB 图像（内部辅助方法）。

        Returns:
            PIL.Image 或 None
        """
        if self._latest_rgb is None:
            # 短暂等待
            self._rgb_event.clear()
            self._rgb_event.wait(timeout=3.0)
        if self._latest_rgb is None:
            return None
        rgb = cv2.cvtColor(self._latest_rgb, cv2.COLOR_BGR2RGB)
        return PILImage.fromarray(rgb)

    def finish_task(self, reason: str) -> dict:
        """结束当前导航任务，立即停止机器人并汇报语义地图。

        当 Gemini 判断已到达目标（如门口）、或认为无法继续前进时，
        应调用此函数来正式结束任务。调用后：
        1. 机器人立即停车（发布零速度指令）
        2. 返回当前位置 + 所有已标注的语义区域

        Args:
            reason: 结束原因（如 "已到达目标门口", "前方死路无法通行"）

        Returns:
            {"success": bool, "final_pose": dict, "semantic_map": dict, "reason": str}
        """
        # 1. 立即停车（多发几次确保停下来）
        stop_cmd = Twist()
        for _ in range(5):
            self.cmd_pub1.publish(stop_cmd)
            self.cmd_pub2.publish(stop_cmd)
            time.sleep(0.02)

        # 2. 标记任务结束（不再发布 goal_point，避免 track_path 触发新一轮 replan）
        self._task_finished = True

        # 4. 收集最终状态
        final_pose = self.get_robot_pose()
        semantic_map = self.get_semantic_labels()

        self.node.get_logger().info(
            f"[RobotTools] 任务结束: {reason}")
        self.node.get_logger().info(
            f"[RobotTools] 最终位置: {final_pose}")
        self.node.get_logger().info(
            f"[RobotTools] 语义地图: {semantic_map}")

        return {
            "success": True,
            "reason": reason,
            "final_pose": final_pose,
            "semantic_map": semantic_map
        }
