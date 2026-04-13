"""
track_path 节点：PID 路径追踪 + 深度图障碍物检测。

核心逻辑：
  1. 订阅 /path 获取路径点，用 Pure Pursuit + PID 追踪
  2. 订阅深度图检测正前方障碍物，检测到则停车并触发 Gemini 重规划
  3. 路径完成后自动触发 Gemini 请求下一段路径
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import numpy as np
import math
import time


class PIDController:
    """Simple PID controller with anti-windup."""
    def __init__(self, kp, ki, kd, output_min=-float('inf'), output_max=float('inf'), integral_max=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_min, self.output_max = output_min, output_max
        self.integral_max = integral_max
        self.prev_error = 0.0
        self.integral = 0.0

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral + error * dt))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(self.output_min, min(self.output_max, output))


class TrackPath(Node):
    def __init__(self):
        super().__init__('track_path')
        self.get_logger().info("Track Path node started")

        # ---- 参数 ----
        self.declare_parameter('obstacle_distance', 0.8)
        self.declare_parameter('obstacle_check_enabled', True)
        self.declare_parameter('depth_image_topic', '/depth_cam/depth0/image_raw')
        self.declare_parameter('obstacle_fov_deg', 60.0)
        self.declare_parameter('obstacle_roi_top_ratio', 0.3)
        self.declare_parameter('obstacle_roi_bottom_ratio', 0.8)
        self.declare_parameter('min_valid_depth', 0.1)
        self.declare_parameter('max_valid_depth', 5.0)
        self.declare_parameter('arrival_threshold', 0.15)
        self.declare_parameter('lookahead_dist', 0.3)

        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.obstacle_check_enabled = self.get_parameter('obstacle_check_enabled').value
        self.arrival_threshold = self.get_parameter('arrival_threshold').value
        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.obstacle_fov_rad = math.radians(self.get_parameter('obstacle_fov_deg').value)
        self.obstacle_roi_top = self.get_parameter('obstacle_roi_top_ratio').value
        self.obstacle_roi_bottom = self.get_parameter('obstacle_roi_bottom_ratio').value
        self.min_valid_depth = self.get_parameter('min_valid_depth').value
        self.max_valid_depth = self.get_parameter('max_valid_depth').value

        # ---- 订阅 ----
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        depth_topic = self.get_parameter('depth_image_topic').value
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub_depth = self.create_subscription(
            Image, depth_topic, self.depth_obstacle_callback, sensor_qos)

        # ---- 发布 ----
        self.cmd_pub1 = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_pub2 = self.create_publisher(Twist, '/agent0/cmd_vel', 10)

        # ---- 服务客户端：触发 Gemini 重规划 ----
        self.trigger_client = self.create_client(Trigger, '/trigger_llm_plan')

        # ---- 状态 ----
        self.bridge = CvBridge()
        self.desired_path = None
        self.current_target_idx = 0
        self.current_pose = None
        self.current_yaw = None
        self.path_completed = False
        self.obstacle_detected = False
        self._obstacle_detect_time = 0.0
        self._obstacle_replan_triggered = False
        self._obstacle_immunity_until = 0.0

        # ---- PID ----
        self.angular_pid = PIDController(kp=2.0, ki=0.1, kd=0.3,
                                         output_min=-2.0, output_max=2.0)
        self.linear_pid = PIDController(kp=0.5, ki=0.05, kd=0.1,
                                        output_min=0.0, output_max=1.0)
        self.last_control_time = None

        # ---- 10Hz 控制循环 ----
        self.timer = self.create_timer(0.1, self.control_loop)

    # ==================== 位置回调 ====================

    def odom_callback(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self.current_pose = pose_msg
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ==================== 路径回调 ====================

    def path_callback(self, msg):
        self.desired_path = msg.poses
        self.current_target_idx = 0
        self.path_completed = False
        self.obstacle_detected = False
        self._obstacle_replan_triggered = False
        self._obstacle_immunity_until = time.monotonic() + 2.0
        self.angular_pid.reset()
        self.linear_pid.reset()
        self.last_control_time = None
        self.get_logger().info(f"收到新路径，共 {len(self.desired_path)} 个路径点")

    # ==================== 深度图障碍物检测 ====================

    def depth_obstacle_callback(self, msg: Image):
        if not self.obstacle_check_enabled:
            return
        if self.desired_path is None or self.path_completed:
            return
        if time.monotonic() < self._obstacle_immunity_until:
            return

        min_dist = self._get_min_depth(msg)
        if min_dist is None:
            if self.obstacle_detected:
                self.obstacle_detected = False
                self.get_logger().info("✅ 障碍物已清除，恢复跟踪")
            return

        if min_dist < self.obstacle_distance:
            if not self.obstacle_detected:
                self.obstacle_detected = True
                self._obstacle_detect_time = time.monotonic()
                self.stop_robot()
                self.get_logger().warn(
                    f"⚠️ 障碍物！距离: {min_dist:.2f}m < {self.obstacle_distance:.2f}m，停车")
            elif time.monotonic() - self._obstacle_detect_time > 3.0 and not self._obstacle_replan_triggered:
                self._obstacle_replan_triggered = True
                self.get_logger().info("🔄 障碍物持续 3s，触发 Gemini 重规划")
                self._trigger_replan()
        else:
            if self.obstacle_detected:
                self.obstacle_detected = False
                self._obstacle_replan_triggered = False
                self.get_logger().info(f"✅ 障碍物已清除（{min_dist:.2f}m），恢复跟踪")

    def _get_min_depth(self, msg: Image):
        """从深度图中央 ROI 提取最近有效深度（米）。"""
        try:
            if msg.encoding in ('16UC1', 'mono16'):
                raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                depth_m = raw.astype(np.float32) / 1000.0
            elif msg.encoding == '32FC1':
                depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            else:
                return None
        except Exception:
            return None

        h, w = depth_m.shape
        row_top = int(h * self.obstacle_roi_top)
        row_bottom = int(h * self.obstacle_roi_bottom)
        half_w = int(w * min(math.tan(self.obstacle_fov_rad / 2.0), 0.5))
        col_left = max(0, w // 2 - half_w)
        col_right = min(w, w // 2 + half_w)

        roi = depth_m[row_top:row_bottom, col_left:col_right]
        valid = roi[(roi > self.min_valid_depth) & (roi < self.max_valid_depth)]
        return float(np.min(valid)) if len(valid) > 0 else None

    # ==================== 主控制循环 ====================

    def control_loop(self):
        if self.desired_path is None or self.current_pose is None or self.current_yaw is None:
            self.stop_robot()
            return
        if self.path_completed or self.obstacle_detected:
            self.stop_robot()
            return

        now = time.monotonic()
        dt = 0.1 if self.last_control_time is None else now - self.last_control_time
        self.last_control_time = now

        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y

        # 检查是否到达终点
        final_pt = self.desired_path[-1].pose.position
        dist_to_final = math.hypot(final_pt.x - curr_x, final_pt.y - curr_y)
        if dist_to_final < self.arrival_threshold:
            self.stop_robot()
            self.path_completed = True
            self.desired_path = None
            self.get_logger().info(f"✅ 路径完成！距终点: {dist_to_final:.3f}m，触发 Gemini 请求下一段")
            self._trigger_replan()
            return

        # 推进目标点索引
        while self.current_target_idx < len(self.desired_path) - 1:
            pt = self.desired_path[self.current_target_idx].pose.position
            if math.hypot(pt.x - curr_x, pt.y - curr_y) >= self.arrival_threshold * 2:
                break
            self.current_target_idx += 1

        # Pure Pursuit: 找前瞻目标
        target_idx = self.current_target_idx
        for i in range(self.current_target_idx, len(self.desired_path)):
            pt = self.desired_path[i].pose.position
            if math.hypot(pt.x - curr_x, pt.y - curr_y) >= self.lookahead_dist:
                target_idx = i
                break
        target_idx = min(target_idx, len(self.desired_path) - 1)
        target = self.desired_path[target_idx].pose.position

        dx = target.x - curr_x
        dy = target.y - curr_y
        dist = math.hypot(dx, dy)
        angle_error = math.atan2(math.sin(math.atan2(dy, dx) - self.current_yaw),
                                 math.cos(math.atan2(dy, dx) - self.current_yaw))

        angular_cmd = self.angular_pid.compute(angle_error, dt)
        angle_factor = max(0.0, 1.0 - abs(angle_error) / math.pi)
        linear_cmd = self.linear_pid.compute(dist, dt) * angle_factor
        if dist > 0.1 and linear_cmd < 0.03:
            linear_cmd = 0.03

        cmd = Twist()
        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd
        self.cmd_pub1.publish(cmd)
        self.cmd_pub2.publish(cmd)

    # ==================== 辅助方法 ====================

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub1.publish(cmd)
        self.cmd_pub2.publish(cmd)

    def _trigger_replan(self):
        """调用 /trigger_llm_plan 触发 Gemini 重新规划。"""
        self.stop_robot()
        if not self.trigger_client.service_is_ready():
            self.get_logger().warn("⚠️ /trigger_llm_plan 服务不可用，请手动触发。")
            return
        future = self.trigger_client.call_async(Trigger.Request())
        future.add_done_callback(self._replan_done)
        self.get_logger().info("📡 已触发 Gemini 重规划...")

    def _replan_done(self, future):
        try:
            r = future.result()
            self.get_logger().info(f"{'✅' if r.success else '⚠️'} 重规划: {r.message}")
        except Exception as e:
            self.get_logger().error(f"重规划异常: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TrackPath()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
