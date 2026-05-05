"""
track_path 节点：纯目标追踪 + 斥力场（Potential Field）避障。

核心逻辑：
  1. 订阅 /goal_point 获取 Gemini 发布的单个目标点（odom 绝对坐标）
  2. 订阅 /scan_raw（2D 雷达 LaserScan）实时获取障碍物距离
  3. 10Hz 控制循环：
     - 计算目标方向角（局部坐标系）作为吸引力方向
     - 270°（前方±135°）雷达斥力场：近距离障碍物产生反方向推力，防撞
     - 吸引力 + 斥力 合成最终行驶方向
  4. 到达目标点（< 阈值）时停车，触发 Gemini 下一轮
  5. 雷达正前方检测到极近障碍物（< emergency_dist）时紧急停车
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
import math
import time


class TrackPath(Node):
    def __init__(self):
        super().__init__('track_path')
        self.get_logger().info("Track Path node started (Attract+Repulse Navigation mode)")

        # ---- 参数 ----
        # 紧急停车
        self.declare_parameter('emergency_dist', 0.15)     # 紧急停车距离（米）
        self.declare_parameter('emergency_fov_deg', 60.0)  # 紧急停车检测 FOV（度）

        # 斥力场参数（270° 防撞）
        self.declare_parameter('repulse_fov_deg', 270.0)     # 斥力场检测 FOV（度），前方 ±135°
        self.declare_parameter('repulse_dist', 0.7)          # 斥力场外缘生效距离（米）
        self.declare_parameter('repulse_gain', 0.8)          # 斥力场增益

        # 运动限制
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('min_linear_vel', 0.03)

        # 目标到达
        self.declare_parameter('arrival_threshold', 0.3)

        # 雷达话题
        self.declare_parameter('scan_topic', '/scan_raw')

        self.emergency_dist = self.get_parameter('emergency_dist').value
        self.emergency_fov_deg = self.get_parameter('emergency_fov_deg').value
        self.repulse_fov_deg = self.get_parameter('repulse_fov_deg').value
        self.repulse_dist = self.get_parameter('repulse_dist').value
        self.repulse_gain = self.get_parameter('repulse_gain').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.min_linear_vel = self.get_parameter('min_linear_vel').value
        self.arrival_threshold = self.get_parameter('arrival_threshold').value
        scan_topic = self.get_parameter('scan_topic').value

        # ---- 订阅 ----
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_point', self.goal_callback, 10)

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub_scan = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, scan_qos)

        # ---- 发布 ----
        self.cmd_pub1 = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_pub2 = self.create_publisher(Twist, '/agent0/cmd_vel', 10)

        # ---- 服务客户端：触发 Gemini 重规划 ----
        self.trigger_client = self.create_client(Trigger, '/trigger_llm_plan')

        # ---- 状态 ----
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.goal_x = None
        self.goal_y = None
        self.goal_active = False
        self.latest_scan = None
        self._goal_reached_logged = False

        # ---- 卡住检测 ----
        self._stuck_start_time = None
        self._stuck_timeout = 5.0
        self._replan_cooldown = 10.0
        self._last_replan_time = 0.0

        # ---- 10Hz 控制循环 ----
        self.timer = self.create_timer(0.1, self.control_loop)

    # ==================== 回调 ====================

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg):
        """接收 Gemini 发布的单个目标点。"""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_active = True
        self._goal_reached_logged = False
        self.get_logger().info(
            f"📍 收到新目标点: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def scan_callback(self, msg):
        """缓存最新的雷达数据。"""
        self.latest_scan = msg

    # ==================== 核心方法 ====================

    def _get_goal_angle_local(self):
        """计算目标点相对于机器人朝向的角度（局部坐标系）。"""
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        goal_angle_global = math.atan2(dy, dx)
        angle_diff = goal_angle_global - self.current_yaw
        return math.atan2(math.sin(angle_diff), math.cos(angle_diff))

    def _check_emergency(self):
        """检查前方是否有极近障碍物需要紧急停车。"""
        if self.latest_scan is None:
            return False

        scan = self.latest_scan
        emergency_fov_rad = math.radians(self.emergency_fov_deg)

        for i in range(len(scan.ranges)):
            r = scan.ranges[i]
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(angle) > emergency_fov_rad / 2.0:
                continue
            if r < scan.range_min or r > scan.range_max or math.isinf(r) or math.isnan(r):
                continue
            if r < self.emergency_dist:
                return True

        return False

    def _compute_repulsive_force(self):
        """计算 270° 范围内障碍物的斥力场。

        Returns:
            repulse_angular: 斥力角速度修正（弧度），正=左推，负=右推
            repulse_brake: 减速因子 [0, 1]
        """
        if self.latest_scan is None:
            return 0.0, 1.0

        scan = self.latest_scan
        repulse_fov_rad = math.radians(self.repulse_fov_deg) / 2.0
        repulse_dist = self.repulse_dist
        gain = self.repulse_gain

        fx_sum = 0.0
        fy_sum = 0.0
        min_front_dist = float('inf')

        for i in range(len(scan.ranges)):
            r = scan.ranges[i]
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(angle) > repulse_fov_rad:
                continue
            if r < scan.range_min or r > scan.range_max or math.isinf(r) or math.isnan(r):
                continue

            if abs(angle) < math.radians(30) and r < min_front_dist:
                min_front_dist = r

            if r >= repulse_dist:
                continue

            # 非线性斥力：距离越近力越大（二次方衰减）
            # 在 repulse_dist 边缘力很小，越近越陡峭
            normalized = (repulse_dist - r) / (repulse_dist - self.emergency_dist)
            normalized = max(0.0, min(1.0, normalized))
            force_mag = gain * normalized * normalized * (1.0 / max(r, 0.05))
            repulse_angle = angle + math.pi
            fx_sum += force_mag * math.cos(repulse_angle)
            fy_sum += force_mag * math.sin(repulse_angle)

        # 合力角度偏移（斥力方向）
        repulse_angular = math.atan2(fy_sum, max(abs(fx_sum), 0.01))
        force_magnitude = math.hypot(fx_sum, fy_sum)
        # 放大系数：合力越大偏转越多，上限 1.5
        repulse_angular *= min(force_magnitude * 0.3, 1.5)

        # 前方减速因子
        if min_front_dist < repulse_dist:
            repulse_brake = 0.3 + 0.7 * max(0.0,
                (min_front_dist - self.emergency_dist) /
                (repulse_dist - self.emergency_dist))
        else:
            repulse_brake = 1.0

        return repulse_angular, repulse_brake

    # ==================== 主控制循环 ====================

    def control_loop(self):
        if self.current_x is None or self.current_yaw is None:
            return
        if not self.goal_active:
            return

        # 检查是否到达目标
        dist_to_goal = math.hypot(
            self.goal_x - self.current_x,
            self.goal_y - self.current_y)

        if dist_to_goal < self.arrival_threshold:
            if not self._goal_reached_logged:
                self._goal_reached_logged = True
                self.goal_active = False
                self.stop_robot()
                self.get_logger().info(
                    f"✅ 到达目标点！距离: {dist_to_goal:.3f}m，触发 Gemini 下一轮")
                self._trigger_replan()
            return

        # 目标方向（吸引力）
        goal_angle_local = self._get_goal_angle_local()

        # 紧急停车检测
        if self._check_emergency():
            now = time.monotonic()
            if self._stuck_start_time is None:
                self._stuck_start_time = now
            stuck_duration = now - self._stuck_start_time

            # 如果目标在侧面/后方（角度偏差 > 60°），允许纯角速度原地转向脱困
            if abs(goal_angle_local) > math.radians(60):
                # 纯转向：不前进，只原地旋转朝向目标方向
                angular_cmd = goal_angle_local * 1.5
                angular_cmd = max(-self.max_angular_vel,
                                  min(angular_cmd, self.max_angular_vel))
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = angular_cmd
                self.cmd_pub1.publish(cmd)
                self.cmd_pub2.publish(cmd)
                self.get_logger().info(
                    f"🔄 紧急停车但目标在侧面({math.degrees(goal_angle_local):.0f}°)，原地转向中...",
                    throttle_duration_sec=2.0)
                self._stuck_start_time = None  # 正在转向，不算卡住
                return

            # 前方有障碍但目标也在前方 → 停车等待重规划
            self.stop_robot()
            self.get_logger().warn(
                f"🛑 紧急停车！前方极近障碍物"
                f"（已持续 {stuck_duration:.1f}s）",
                throttle_duration_sec=3.0)
            if (stuck_duration > self._stuck_timeout and
                    now - self._last_replan_time > self._replan_cooldown):
                self._last_replan_time = now
                self._stuck_start_time = None
                self.goal_active = False
                self.get_logger().warn(
                    f"⚠️ 卡住超过 {self._stuck_timeout}s，触发 Gemini 重规划！")
                self._trigger_replan()
            return
        else:
            self._stuck_start_time = None

        # 斥力场修正
        repulse_angular, repulse_brake = self._compute_repulsive_force()

        # 最终方向 = 目标方向 + 斥力修正
        target_direction = goal_angle_local + repulse_angular
        target_direction = math.atan2(
            math.sin(target_direction), math.cos(target_direction))

        # 角速度：P 控制
        angular_cmd = target_direction * 2.0

        # 线速度：方向偏差越大越慢
        angle_factor = max(0.0, math.cos(target_direction))
        linear_cmd = self.max_linear_vel * angle_factor

        # 斥力场减速
        linear_cmd *= repulse_brake

        # 限幅
        linear_cmd = max(0.0, min(linear_cmd, self.max_linear_vel))
        angular_cmd = max(-self.max_angular_vel, min(angular_cmd, self.max_angular_vel))

        # 保证最小速度
        if abs(target_direction) < math.radians(60):
            if dist_to_goal > self.arrival_threshold and linear_cmd < self.min_linear_vel:
                linear_cmd = self.min_linear_vel

        # 接近目标时减速
        if dist_to_goal < 0.5:
            linear_cmd *= (dist_to_goal / 0.5)
            linear_cmd = max(linear_cmd, self.min_linear_vel * 0.5)

        # 发布速度指令
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
            self.get_logger().info(
                f"{'✅' if r.success else '⚠️'} 重规划: {r.message}")
            # 如果 Agent 返回的消息包含"任务已结束"，不再接受新目标
            if r.message and "任务已结束" in r.message:
                self.goal_active = False
                self.get_logger().info("🏁 Agent 已结束任务，track_path 停止导航")
        except Exception as e:
            self.get_logger().error(f"重规划异常: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TrackPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
