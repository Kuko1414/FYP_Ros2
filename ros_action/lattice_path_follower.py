"""
ROS 2 节点：基于连续轨迹跟踪 + Lattice修正
使用CurveGenerator生成连续参考轨迹
Lattice Planner仅用于偏离时修正
基于VRPN动捕系统的位姿反馈
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path
import numpy as np
import math
from collections import deque

try:
    from .simple_lattice_planner import QuinticPolynomial, QuarticPolynomial
    from .curve_generators.circle_trajectory import CircleTrajectory
except ImportError:
    from simple_lattice_planner import QuinticPolynomial, QuarticPolynomial
    from curve_generators.circle_trajectory import CircleTrajectory


class LatticePathFollower(Node):
    """Lattice Planner路径跟踪节点 - 参考原始lattice_planner.py实现"""
    
    def __init__(self):
        super().__init__('lattice_path_follower')
        
        # 参数
        self.declare_parameter('target_speed', 0.3)
        self.declare_parameter('control_frequency', 10.0)  # 降低频率，原始是每步规划
        self.declare_parameter('max_angular_speed', 1.5)
        
        # 获取参数
        self.target_speed = self.get_parameter('target_speed').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # 状态变量
        self.target_point = None
        self.start_point = None
        self.current_position = None
        self.current_yaw = None
        
        # 参考轨迹
        self.ref_trajectory = None
        
        # Frenet坐标状态（关键！）
        self.l0 = 0.0      # 当前横向位置
        self.l0_v = 0.0    # 当前横向速度
        self.l0_a = 0.0    # 当前横向加速度
        self.s0 = 0.0      # 当前纵向位置
        self.s0_v = 0.0    # 当前纵向速度
        self.s0_a = 0.0    # 当前纵向加速度
        
        # 当前规划路径
        self.current_path = None
        
        # 控制参数
        self.distance_tolerance = 0.2
        
        # 标志
        self.path_planned = False
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅器
        self.target_sub = self.create_subscription(
            Point, '/target_point', self.target_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/vrpn_mocap/rm_0_Test/pose', 
            self.pose_callback, qos_profile)
        
        # 发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/rm_0/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.ref_path_pub = self.create_publisher(Path, '/reference_path', 10)
        
        # 定时器：控制循环
        self.control_timer = self.create_timer(1.0/self.control_frequency, self.control_loop)
        
        self.get_logger().info('Lattice Planner路径跟踪节点已启动')
        self.get_logger().info('等待目标点和位姿数据...')
    
    def target_callback(self, msg):
        """目标点回调"""
        if self.current_position is None or self.current_yaw is None:
            self.get_logger().warn('尚未接收到位姿数据，等待中...')
            return
        
        self.target_point = (msg.x, msg.y)
        self.start_point = self.current_position
        
        self.get_logger().info(f'接收目标点: ({msg.x:.2f}, {msg.y:.2f})')
        self.get_logger().info(f'起点: ({self.start_point[0]:.2f}, {self.start_point[1]:.2f})')
        
        # 生成参考轨迹
        self._generate_reference_trajectory()
        
        # 初始化Frenet状态
        self._initialize_frenet_state()
    
    def pose_callback(self, msg):
        """位姿回调"""
        self.current_position = (msg.pose.position.x, msg.pose.position.y)
        self.current_yaw = self._yaw_from_quaternion(
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w)
    
    def _yaw_from_quaternion(self, x, y, z, w):
        """从四元数计算yaw角"""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def _generate_reference_trajectory(self):
        """使用CurveGenerator生成连续圆形参考轨迹"""
        if self.start_point is None or self.target_point is None:
            return
        
        # 计算圆心和半径
        center_x = (self.start_point[0] + self.target_point[0]) / 2.0
        center_y = (self.start_point[1] + self.target_point[1]) / 2.0
        radius = math.hypot(
            self.target_point[0] - self.start_point[0],
            self.target_point[1] - self.start_point[1]) / 2.0
        
        if radius < 0.1:
            self.get_logger().warn('起点和终点太近')
            return
        
        # 使用CircleTrajectory生成连续轨迹
        self.ref_trajectory = CircleTrajectory(center_x, center_y, radius, num_points=100)
        
        self.path_planned = True
        
        self.get_logger().info(
            f'生成圆形轨迹: 中心=({center_x:.2f}, {center_y:.2f}), 半径={radius:.2f}m')
        
        # 发布参考路径
        self._publish_reference_path()
    
    def _initialize_frenet_state(self):
        """初始化Frenet坐标状态"""
        if self.ref_trajectory is None or self.current_position is None:
            return
        
        # 将当前位置转换为Frenet坐标
        s, l = self._cartesian_to_frenet(self.current_position[0], self.current_position[1])
        
        self.s0 = s
        self.l0 = l
        self.s0_v = self.target_speed  # 初始纵向速度
        self.l0_v = 0.0
        self.s0_a = 0.0
        self.l0_a = 0.0
        
        self.get_logger().info(f'初始Frenet状态: s={s:.2f}, l={l:.2f}')
        self.get_logger().info(f'轨迹总长: {self.ref_trajectory.s[-1]:.2f}m')
        
        # 测试Frenet转换
        test_x, test_y = self.ref_trajectory.calc_position(s)
        self.get_logger().info(f'测试转换: s={s:.2f} -> ({test_x:.2f}, {test_y:.2f})')
    
    def _cartesian_to_frenet(self, x, y):
        """笛卡尔坐标转Frenet坐标"""
        if self.ref_trajectory is None:
            return 0.0, 0.0
        
        # 找到最近的参考点
        min_dist = float('inf')
        closest_idx = 0
        
        for i in range(len(self.ref_trajectory.x)):
            ref_x = self.ref_trajectory.x[i]
            ref_y = self.ref_trajectory.y[i]
            dist = math.hypot(x - ref_x, y - ref_y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # s坐标就是弧长
        s = self.ref_trajectory.s[closest_idx]
        
        # 计算横向偏移l（带符号）
        ref_x = self.ref_trajectory.x[closest_idx]
        ref_y = self.ref_trajectory.y[closest_idx]
        ref_yaw = self.ref_trajectory.yaw[closest_idx]
        
        # 从参考点到当前点的向量
        dx = x - ref_x
        dy = y - ref_y
        
        # 计算横向偏移（正值表示左侧，负值表示右侧）
        l = dx * math.cos(ref_yaw + math.pi/2) + dy * math.sin(ref_yaw + math.pi/2)
        
        return s, l
    
    def control_loop(self):
        """主控制循环 - 参考原始lattice_planner.py的逻辑"""
        if not self.path_planned or self.current_position is None:
            return
        
        # 检查是否到达目标
        if self.target_point is not None:
            dist_to_goal = self._distance(self.current_position, self.target_point)
            if dist_to_goal < self.distance_tolerance:
                self.get_logger().info('已到达目标点！')
                self._stop_robot()
                self.path_planned = False
                return
        
        # 关键：每次都用Lattice Planner重新规划路径
        path = self._lattice_plan()
        
        if path is None or len(path.x) < 2:
            self.get_logger().warn('规划失败，停止')
            self._stop_robot()
            return
        
        # 保存当前路径用于可视化
        self.current_path = path
        
        # 关键：使用规划路径的第二个点更新Frenet状态（不是第一个点！）
        # 这样可以让小车沿着规划路径前进
        if len(path.l) > 1:
            self.l0 = path.l[1]
            self.l0_v = path.l_v[1]
            self.l0_a = path.l_a[1]
            self.s0 = path.s[1]
            self.s0_v = path.s_v[1]
            self.s0_a = path.s_a[1]
        
        # 计算控制命令（使用第二个点）
        if len(path.x) > 1:
            target_x = path.x[1]
            target_y = path.y[1]
            target_yaw = path.yaw[1]
            
            cmd_vel = self._calculate_control(target_x, target_y, target_yaw)
            self.cmd_vel_pub.publish(cmd_vel)
        
        # 发布路径
        self._publish_current_path()
    
    def _lattice_plan(self):
        """使用Lattice Planner规划路径"""
        if self.ref_trajectory is None:
            return None
        
        # 采样候选路径
        paths = []
        
        # 采样目标横向位置（-0.3到0.3米，对应小车可能的横向调整）
        for target_l in np.arange(-0.3, 0.31, 0.15):
            # 采样目标速度
            for target_v in [self.target_speed * 0.9, self.target_speed, self.target_speed * 1.1]:
                # 规划时间
                T = 1.0  # 1秒规划周期
                
                try:
                    # 生成纵向轨迹（四次多项式）
                    lon_traj = QuarticPolynomial(
                        self.s0, self.s0_v, self.s0_a,
                        target_v, 0.0, T
                    )
                    
                    # 生成横向轨迹（五次多项式）
                    lat_traj = QuinticPolynomial(
                        self.l0, self.l0_v, self.l0_a,
                        target_l, 0.0, 0.0, T
                    )
                    
                    # 创建路径
                    path = type('Path', (), {})()  # 简单对象
                    t_series = np.arange(0.0, T, 0.1)
                    
                    path.t = list(t_series)
                    path.s = [lon_traj.calc_xt(t) for t in t_series]
                    path.s_v = [lon_traj.calc_dxt(t) for t in t_series]
                    path.s_a = [lon_traj.calc_ddxt(t) for t in t_series]
                    path.l = [lat_traj.calc_xt(t) for t in t_series]
                    path.l_v = [lat_traj.calc_dxt(t) for t in t_series]
                    path.l_a = [lat_traj.calc_ddxt(t) for t in t_series]
                    path.l_jerk = [lat_traj.calc_dddxt(t) for t in t_series]
                    path.s_jerk = [lon_traj.calc_dddxt(t) for t in t_series]
                    
                    # 转换到笛卡尔坐标
                    path.x, path.y = self._frenet_to_cartesian(path.s, path.l)
                    
                    if len(path.x) < 2:
                        continue
                    
                    # 计算航向和曲率
                    path.yaw, path.curv = self._calc_yaw_curvature(path.x, path.y)
                    
                    if path.yaw is None:
                        continue
                    
                    # 计算代价
                    path.cost = self._calculate_cost(path, target_v, T, target_l)
                    
                    paths.append(path)
                    
                except Exception as e:
                    self.get_logger().debug(f'路径生成异常: {e}')
                    continue
        
        # 选择代价最小的路径
        if len(paths) == 0:
            self.get_logger().warn(f'无有效路径，当前状态: s={self.s0:.2f}, l={self.l0:.2f}, s_v={self.s0_v:.2f}')
            return None
        
        paths.sort(key=lambda p: p.cost)
        self.get_logger().debug(f'生成{len(paths)}条路径，最优代价={paths[0].cost:.2f}')
        return paths[0]
    
    def _frenet_to_cartesian(self, s_series, l_series):
        """Frenet坐标转笛卡尔坐标"""
        x_series, y_series = [], []
        
        for s, l in zip(s_series, l_series):
            try:
                x_ref, y_ref = self.ref_trajectory.calc_position(s)
                yaw = self.ref_trajectory.calc_yaw(s)
                
                # 沿着垂直于参考线的方向偏移l
                x = x_ref + l * math.cos(yaw + math.pi / 2.0)
                y = y_ref + l * math.sin(yaw + math.pi / 2.0)
                
                x_series.append(x)
                y_series.append(y)
            except Exception as e:
                self.get_logger().error(f'Frenet转换失败: s={s:.2f}, l={l:.2f}, 错误={e}')
                break
        
        return x_series, y_series
    
    def _calc_yaw_curvature(self, x, y):
        """计算航向角和曲率"""
        yaw, curv = [], []
        
        for i in range(len(x) - 1):
            dx = x[i + 1] - x[i]
            dy = y[i + 1] - y[i]
            ds = math.hypot(dx, dy)
            yaw.append(math.atan2(dy, dx))
        
        if len(yaw) == 0:
            return None, None
        
        yaw.append(yaw[-1])
        
        for i in range(len(yaw) - 1):
            ds = math.hypot(x[i+1] - x[i], y[i+1] - y[i])
            if ds > 0.001:
                curv.append((yaw[i + 1] - yaw[i]) / ds)
            else:
                curv.append(0.0)
        
        curv.append(curv[-1] if curv else 0.0)
        
        return yaw, curv
    
    def _calculate_cost(self, path, target_v, T, target_l):
        """计算路径代价"""
        # 急动度代价
        l_jerk_sum = sum(np.abs(path.l_jerk))
        s_jerk_sum = sum(np.abs(path.s_jerk))
        
        # 速度差代价
        v_diff = abs(self.target_speed - path.s_v[-1])
        
        # 横向偏移代价
        offset = abs(path.l[-1])
        
        # 时间代价
        time_cost = T
        
        total_cost = (
            0.1 * (l_jerk_sum + s_jerk_sum) +
            1.0 * v_diff +
            0.1 * time_cost +
            2.0 * offset
        )
        
        return total_cost
    
    def _calculate_control(self, target_x, target_y, target_yaw):
        """计算控制命令"""
        cmd_vel = Twist()
        
        # 计算到目标点的方向
        dx = target_x - self.current_position[0]
        dy = target_y - self.current_position[1]
        desired_yaw = math.atan2(dy, dx)
        
        # 航向角误差
        heading_error = self._angle_wrap(desired_yaw - self.current_yaw)
        
        # 纯跟踪控制
        kp = 3.0
        angular_z = kp * heading_error
        angular_z = np.clip(angular_z, -self.max_angular_speed, self.max_angular_speed)
        
        # 速度（使用规划的速度）
        speed = self.s0_v
        speed = max(0.1, min(speed, self.target_speed * 1.2))
        
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = angular_z
        
        return cmd_vel
    
    def _publish_current_path(self):
        """发布当前规划路径"""
        if self.current_path is None:
            return
        
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in zip(self.current_path.x, self.current_path.y):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def _publish_reference_path(self):
        """发布参考路径"""
        if self.ref_trajectory is None:
            return
        
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(len(self.ref_trajectory.x)):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(self.ref_trajectory.x[i])
            pose.pose.position.y = float(self.ref_trajectory.y[i])
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        
        self.ref_path_pub.publish(path_msg)
        self.get_logger().info('参考轨迹已发布')
    
    def _stop_robot(self):
        """停止机器人"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
    
    def _distance(self, p1, p2):
        """计算两点间距离"""
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])
    
    def _angle_wrap(self, angle):
        """角度归一化到[-π, π]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = LatticePathFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
