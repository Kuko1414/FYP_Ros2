#!/usr/bin/env python3
"""
ROS2节点：订阅当前位置和目标点，计算多种轨迹路径并以固定速度跟踪
支持轨迹类型：circle(圆形), square(方形), eight(8字形), triangle(三角形), star(五角星)

使用方法：
    ros2 run ros_action pid_track --ros-args -p pattern:=circle
    ros2 run ros_action pid_track --ros-args -p pattern:=square
    ros2 run ros_action pid_track --ros-args -p pattern:=eight
    ros2 run ros_action pid_track --ros-args -p pattern:=triangle
    ros2 run ros_action pid_track --ros-args -p pattern:=star
    
    可选参数：
    -p speed:=0.5        # 设置速度 (m/s)
    -p scale:=1.5        # 设置轨迹缩放比例
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import math


class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')
        
        # 声明参数
        self.declare_parameter('pattern', 'circle')  # 默认为圆形
        self.declare_parameter('speed', 0.33)  # 默认速度
        self.declare_parameter('scale', 1.0)  # 轨迹缩放比例
        
        # 获取参数
        self.pattern = self.get_parameter('pattern').get_parameter_value().string_value
        self.target_speed = self.get_parameter('speed').get_parameter_value().double_value
        self.scale = self.get_parameter('scale').get_parameter_value().double_value
        
        # 验证轨迹类型
        valid_patterns = ['circle', 'square', 'eight', 'triangle', 'star']
        if self.pattern not in valid_patterns:
            self.get_logger().warn(f'未知轨迹类型: {self.pattern}，使用默认circle')
            self.get_logger().warn(f'支持的类型: {valid_patterns}')
            self.pattern = 'circle'
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'    轨迹跟踪器启动')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  轨迹类型: {self.pattern}')
        self.get_logger().info(f'  目标速度: {self.target_speed} m/s')
        self.get_logger().info(f'  缩放比例: {self.scale}')
        self.get_logger().info('=' * 50)
        
        # 参数设置
        self.radius_error_threshold = 0.1  # 路径偏差阈值 (米)
        
        # PID参数
        self.kp = 0.4  # 比例系数
        self.ki = 0.1  # 积分系数
        self.kd = 0.1  # 微分系数
        self.integral_error = 0.05  # 积分误差
        self.last_error = 0.0  # 上一次误差
        
        # 状态变量
        self.current_pose = None  # 当前位置（持续更新）
        self.start_position = None  # 起始位置
        self.target_point = None  # 目标点
        self.path_center = None  # 路径中心
        self.path_size = None  # 路径尺寸（半径或边长）
        self.path_points = []  # 路径点列表
        self.path_ready = False  # 路径是否准备好
        self.current_path_index = 0  # 当前路径点索引
        
        # 设置QoS配置以匹配VRPN mocap
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅当前位置（使用BEST_EFFORT QoS）
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/rm_0_Test/pose',
            self.pose_callback,
            qos_profile
        )
        
        # 订阅目标点
        self.target_sub = self.create_subscription(
            Point,
            '/target_point',
            self.target_callback,
            10
        )
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/rm_0/cmd_vel',
            10
        )
        
        # 发布可视化Marker（用于rviz2）
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/path_markers',
            10
        )
        
        # 定时器：控制循环 (50Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # 定时器：可视化更新 (20Hz) - 提高更新频率让 rviz2 更流畅
        self.viz_timer = self.create_timer(0.05, self.publish_visualization)
        
        self.get_logger().info('等待当前位置 (/vrpn_mocap/rm_0_Test/pose) 和目标点 (/target_point)...')

    def pose_callback(self, msg: PoseStamped):
        """接收当前位置"""
        self.current_pose = msg.pose
        
        # 如果还没有路径且已有目标点，计算路径
        if not self.path_ready and self.target_point is not None:
            self.calculate_path()

    def target_callback(self, msg: Point):
        """接收目标点（只记录第一次收到的目标点）"""
        if self.path_ready:
            self.get_logger().info('路径已生成，忽略新的目标点')
            return
        
        if self.target_point is not None:
            return
            
        self.target_point = msg
        self.get_logger().info(f'收到目标点: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})')
        
        # 记录起始位置
        if self.current_pose is not None:
            self.start_position = {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y
            }
            self.get_logger().info(f'记录起始位置: ({self.start_position["x"]:.3f}, {self.start_position["y"]:.3f})')
            self.calculate_path()

    def calculate_path(self):
        """根据轨迹类型计算路径"""
        if self.start_position is None or self.target_point is None:
            return
        
        # 获取起始位置和目标点
        p1_x = self.start_position['x']
        p1_y = self.start_position['y']
        p2_x = self.target_point.x
        p2_y = self.target_point.y
        
        # 计算两点之间的距离
        distance = math.sqrt((p2_x - p1_x)**2 + (p2_y - p1_y)**2)
        
        # 路径中心 = 两点中点
        self.path_center = {
            'x': (p1_x + p2_x) / 2.0,
            'y': (p1_y + p2_y) / 2.0
        }
        
        # 路径尺寸（半径或半边长）
        self.path_size = (distance / 2.0) * self.scale
        
        # 计算初始误差（起始点到圆的距离）
        initial_radius = distance / 2.0  # 起始点到圆心的实际距离
        initial_error = initial_radius - self.path_size  # 正值=起点在圆外，负值=起点在圆内
        
        self.get_logger().info(f'起始位置: ({p1_x:.3f}, {p1_y:.3f})')
        self.get_logger().info(f'目标点: ({p2_x:.3f}, {p2_y:.3f})')
        self.get_logger().info(f'路径中心: ({self.path_center["x"]:.3f}, {self.path_center["y"]:.3f})')
        self.get_logger().info(f'路径半径: {self.path_size:.3f}')
        self.get_logger().info(f'初始位置到圆心距离: {initial_radius:.3f}')
        self.get_logger().info(f'初始误差: {initial_error:.3f} (正=在圆外，负=在圆内)')
        
        # 根据轨迹类型生成路径
        if self.pattern == 'circle':
            self.generate_circle_path()
        elif self.pattern == 'square':
            self.generate_square_path()
        elif self.pattern == 'eight':
            self.generate_eight_path()
        elif self.pattern == 'triangle':
            self.generate_triangle_path()
        elif self.pattern == 'star':
            self.generate_star_path()
        
        self.path_ready = True
        self.current_path_index = 0
        self.get_logger().info(f'{self.pattern} 路径已生成，共 {len(self.path_points)} 个点')

    def generate_circle_path(self, num_points=200):
        """生成圆形路径，从起始点位置开始"""
        self.path_points = []
        
        # 计算起始点相对于圆心的角度
        start_angle = math.atan2(
            self.start_position['y'] - self.path_center['y'],
            self.start_position['x'] - self.path_center['x']
        )
        
        for i in range(num_points):
            # 从起始点角度开始，逆时针生成路径
            angle = start_angle + 2 * math.pi * i / num_points
            x = self.path_center['x'] + self.path_size * math.cos(angle)
            y = self.path_center['y'] + self.path_size * math.sin(angle)
            self.path_points.append({'x': x, 'y': y, 'angle': angle})
        
        self.get_logger().info(f'圆形路径点已生成，起始角度: {math.degrees(start_angle):.1f}度')

    def generate_square_path(self, points_per_side=50):
        """生成方形路径"""
        self.path_points = []
        
        # 方形的四个角点（从右上开始，逆时针）
        half_size = self.path_size
        cx, cy = self.path_center['x'], self.path_center['y']
        
        corners = [
            (cx + half_size, cy + half_size),  # 右上
            (cx - half_size, cy + half_size),  # 左上
            (cx - half_size, cy - half_size),  # 左下
            (cx + half_size, cy - half_size),  # 右下
        ]
        
        # 在每条边上生成点
        for i in range(4):
            start = corners[i]
            end = corners[(i + 1) % 4]
            
            for j in range(points_per_side):
                t = j / points_per_side
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                angle = math.atan2(end[1] - start[1], end[0] - start[0])
                self.path_points.append({'x': x, 'y': y, 'angle': angle})
        
        self.get_logger().info('方形路径点已生成')

    def generate_eight_path(self, num_points=300):
        """生成8字形路径（两个相切的圆）"""
        self.path_points = []
        
        # 8字形由两个圆组成，每个圆的半径是path_size的一半
        small_radius = self.path_size * 0.5
        cx, cy = self.path_center['x'], self.path_center['y']
        
        # 右边的圆（顺时针）
        right_center_x = cx + small_radius
        points_per_circle = num_points // 2
        
        for i in range(points_per_circle):
            # 从左边点开始，顺时针画右边的圆
            angle = math.pi - 2 * math.pi * i / points_per_circle
            x = right_center_x + small_radius * math.cos(angle)
            y = cy + small_radius * math.sin(angle)
            self.path_points.append({'x': x, 'y': y, 'angle': angle - math.pi/2})
        
        # 左边的圆（逆时针）
        left_center_x = cx - small_radius
        
        for i in range(points_per_circle):
            # 从右边点开始，逆时针画左边的圆
            angle = 2 * math.pi * i / points_per_circle
            x = left_center_x + small_radius * math.cos(angle)
            y = cy + small_radius * math.sin(angle)
            self.path_points.append({'x': x, 'y': y, 'angle': angle + math.pi/2})
        
        self.get_logger().info('8字形路径点已生成')

    def generate_triangle_path(self, points_per_side=67):
        """生成三角形路径"""
        self.path_points = []
        
        cx, cy = self.path_center['x'], self.path_center['y']
        
        # 等边三角形的三个顶点
        corners = []
        for i in range(3):
            angle = math.pi/2 + 2 * math.pi * i / 3  # 从顶部开始
            x = cx + self.path_size * math.cos(angle)
            y = cy + self.path_size * math.sin(angle)
            corners.append((x, y))
        
        # 在每条边上生成点
        for i in range(3):
            start = corners[i]
            end = corners[(i + 1) % 3]
            
            for j in range(points_per_side):
                t = j / points_per_side
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                angle = math.atan2(end[1] - start[1], end[0] - start[0])
                self.path_points.append({'x': x, 'y': y, 'angle': angle})
        
        self.get_logger().info('三角形路径点已生成')

    def generate_star_path(self, points_per_segment=40):
        """生成五角星路径"""
        self.path_points = []
        
        cx, cy = self.path_center['x'], self.path_center['y']
        outer_radius = self.path_size
        inner_radius = self.path_size * 0.382  # 黄金分割比
        
        # 生成10个点（外5内5交替）
        vertices = []
        for i in range(10):
            angle = math.pi/2 + 2 * math.pi * i / 10
            if i % 2 == 0:
                r = outer_radius
            else:
                r = inner_radius
            x = cx + r * math.cos(angle)
            y = cy + r * math.sin(angle)
            vertices.append((x, y))
        
        # 连接顶点生成路径
        for i in range(10):
            start = vertices[i]
            end = vertices[(i + 1) % 10]
            
            for j in range(points_per_segment):
                t = j / points_per_segment
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                angle = math.atan2(end[1] - start[1], end[0] - start[0])
                self.path_points.append({'x': x, 'y': y, 'angle': angle})
        
        self.get_logger().info('五角星路径点已生成')

    def find_nearest_path_index(self):
        """找到当前位置最近的路径点索引"""
        if self.current_pose is None or len(self.path_points) == 0:
            return 0
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        min_dist = float('inf')
        nearest_idx = 0
        
        for i, point in enumerate(self.path_points):
            dist = math.sqrt((current_x - point['x'])**2 + (current_y - point['y'])**2)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        return nearest_idx

    def get_lookahead_point(self, lookahead_distance=0.3):
        """获取前瞻点（用于Pure Pursuit算法）"""
        if len(self.path_points) == 0:
            return None
        
        nearest_idx = self.find_nearest_path_index()
        
        # 沿路径找到lookahead_distance距离的点
        accumulated_dist = 0.0
        current_idx = nearest_idx
        
        while accumulated_dist < lookahead_distance:
            next_idx = (current_idx + 1) % len(self.path_points)
            p1 = self.path_points[current_idx]
            p2 = self.path_points[next_idx]
            segment_dist = math.sqrt((p2['x'] - p1['x'])**2 + (p2['y'] - p1['y'])**2)
            accumulated_dist += segment_dist
            current_idx = next_idx
            
            # 防止无限循环
            if current_idx == nearest_idx:
                break
        
        return self.path_points[current_idx]

    def calculate_path_error(self):
        """计算当前位置到路径的距离误差"""
        if self.current_pose is None or len(self.path_points) == 0:
            return 0.0
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # 对于圆形，直接计算到圆周的距离误差（更准确）
        if self.pattern == 'circle':
            distance_to_center = math.sqrt(
                (current_x - self.path_center['x'])**2 + 
                (current_y - self.path_center['y'])**2
            )
            # 误差 = 当前到圆心距离 - 圆的半径
            # 正值表示在圆外，负值表示在圆内
            error = distance_to_center - self.path_size
            return error
        
        # 对于其他形状，使用最近点距离
        nearest_idx = self.find_nearest_path_index()
        nearest_point = self.path_points[nearest_idx]
        
        # 计算到最近点的距离
        error = math.sqrt((current_x - nearest_point['x'])**2 + 
                         (current_y - nearest_point['y'])**2)
        
        return error

    def publish_visualization(self):
        """发布可视化Marker到rviz2"""
        if not self.path_ready:
            return
        
        marker_array = MarkerArray()
        
        # 1. 路径线（黑色）
        path_marker = Marker()
        path_marker.header.frame_id = "world"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.02
        path_marker.color.r = 0.0
        path_marker.color.g = 0.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0
        path_marker.pose.orientation.w = 1.0
        
        for point in self.path_points:
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = 0.0
            path_marker.points.append(p)
        
        # 闭合路径
        if len(self.path_points) > 0:
            p = Point()
            p.x = self.path_points[0]['x']
            p.y = self.path_points[0]['y']
            p.z = 0.0
            path_marker.points.append(p)
        
        marker_array.markers.append(path_marker)
        
        # 2. 小车方块（蓝色）
        if self.current_pose is not None:
            car_marker = Marker()
            car_marker.header.frame_id = "world"
            car_marker.header.stamp = self.get_clock().now().to_msg()
            car_marker.ns = "car"
            car_marker.id = 1
            car_marker.type = Marker.CUBE
            car_marker.action = Marker.ADD
            car_marker.pose.position.x = self.current_pose.position.x
            car_marker.pose.position.y = self.current_pose.position.y
            car_marker.pose.position.z = 0.05
            car_marker.pose.orientation = self.current_pose.orientation
            car_marker.scale.x = 0.15
            car_marker.scale.y = 0.10
            car_marker.scale.z = 0.05
            car_marker.color.r = 0.0
            car_marker.color.g = 0.0
            car_marker.color.b = 1.0
            car_marker.color.a = 1.0
            
            marker_array.markers.append(car_marker)
        
        # 3. 未来路径（浅蓝色）
        if self.current_pose is not None:
            future_marker = Marker()
            future_marker.header.frame_id = "world"
            future_marker.header.stamp = self.get_clock().now().to_msg()
            future_marker.ns = "future_path"
            future_marker.id = 2
            future_marker.type = Marker.LINE_STRIP
            future_marker.action = Marker.ADD
            future_marker.scale.x = 0.03
            future_marker.color.r = 0.0
            future_marker.color.g = 0.5
            future_marker.color.b = 1.0
            future_marker.color.a = 0.8
            future_marker.pose.orientation.w = 1.0
            
            nearest_idx = self.find_nearest_path_index()
            num_future_points = min(100, len(self.path_points))
            
            for i in range(num_future_points):
                idx = (nearest_idx + i) % len(self.path_points)
                p = Point()
                p.x = self.path_points[idx]['x']
                p.y = self.path_points[idx]['y']
                p.z = 0.01
                future_marker.points.append(p)
            
            marker_array.markers.append(future_marker)
        
        # 4. 路径中心标记（红色）
        center_marker = Marker()
        center_marker.header.frame_id = "world"
        center_marker.header.stamp = self.get_clock().now().to_msg()
        center_marker.ns = "center"
        center_marker.id = 3
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.pose.position.x = self.path_center['x']
        center_marker.pose.position.y = self.path_center['y']
        center_marker.pose.position.z = 0.0
        center_marker.pose.orientation.w = 1.0
        center_marker.scale.x = 0.08
        center_marker.scale.y = 0.08
        center_marker.scale.z = 0.08
        center_marker.color.r = 1.0
        center_marker.color.g = 0.0
        center_marker.color.b = 0.0
        center_marker.color.a = 1.0
        
        marker_array.markers.append(center_marker)
        
        # 5. 误差和信息文字
        if self.current_pose is not None:
            path_error = abs(self.calculate_path_error())
            
            text_marker = Marker()
            text_marker.header.frame_id = "world"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "info_text"
            text_marker.id = 5
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = self.path_center['x']
            text_marker.pose.position.y = self.path_center['y']
            text_marker.pose.position.z = 0.5  # 提高位置，更容易看到
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.x = 0.2  # 添加 scale.x
            text_marker.scale.y = 0.2  # 添加 scale.y
            text_marker.scale.z = 0.2  # 增大文字大小
            text_marker.color.r = 0.0  # 改为黑色，更容易看到
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            text_marker.text = f"Pattern: {self.pattern}\nError: {path_error:.3f}m"
            
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)

    def control_loop(self):
        """控制循环：沿路径行驶，使用Pure Pursuit + PID修正"""
        if not self.path_ready:
            return
        
        if self.current_pose is None:
            return
        
        # 获取当前位置和朝向
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # 获取前瞻点
        lookahead_point = self.get_lookahead_point(lookahead_distance=0.3)
        if lookahead_point is None:
            return
        
        # 计算到前瞻点的方向
        dx = lookahead_point['x'] - current_x
        dy = lookahead_point['y'] - current_y
        target_yaw = math.atan2(dy, dx)
        
        # 计算角度误差
        yaw_error = self.normalize_angle(target_yaw - current_yaw)
        
        # 计算路径误差
        path_error = self.calculate_path_error()
        
        # PID控制
        self.integral_error += path_error * 0.02
        self.integral_error = max(-1.0, min(1.0, self.integral_error))
        derivative_error = (path_error - self.last_error) / 0.02
        
        pid_correction = (self.kp * path_error + 
                         self.ki * self.integral_error + 
                         self.kd * derivative_error)
        
        self.last_error = path_error
        
        # 创建速度命令
        cmd_vel = Twist()
        cmd_vel.linear.x = self.target_speed
        
        # 角速度 = 跟踪角度误差 + PID修正
        angular_speed = 2.0 * yaw_error + pid_correction * 0.5
        
        # 限制角速度
        max_angular_speed = 2.0
        cmd_vel.angular.z = max(-max_angular_speed, min(max_angular_speed, angular_speed))
        
        # 发布速度命令
        self.cmd_vel_pub.publish(cmd_vel)

    def get_yaw_from_quaternion(self, orientation):
        """从四元数获取yaw角"""
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        
        # 计算yaw角
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw

    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop_robot(self):
        """停止机器人"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info('机器人已停止')


def main(args=None):
    rclpy.init(args=args)
    
    node = PathTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断，正在停止...')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
