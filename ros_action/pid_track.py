#!/usr/bin/env python3
"""
ROS2节点：订阅当前位置和目标点，计算圆形路径并以固定速度跟踪
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import math


class CirclePathTracker(Node):
    def __init__(self):
        super().__init__('circle_path_tracker')
        
        # 参数设置
        self.target_speed = 0.33  # 目标速度 m/s
        self.radius_error_threshold = 0.1  # 半径偏差阈值 (米) - 减小阈值，更早开始修正
        
        # PID参数 - 增大参数让修正更快
        self.kp = 0.4  # 比例系数 - 增大加快响应
        self.ki = 0.2  # 积分系数 - 轻微积分消除稳态误差
        self.kd = 0.1  # 微分系数 - 增大抑制超调
        self.integral_error = 0.1# 积分误差
        self.last_error = 0.0  # 上一次误差
        
        # 状态变量
        self.current_pose = None  # 当前位置（持续更新）
        self.start_position = None  # 起始位置（收到target point时记录，不变）
        self.target_point = None  # 目标点
        self.circle_center = None  # 圆心
        self.circle_radius = None  # 圆的半径
        self.circle_path = []  # 圆形路径点列表
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
            '/circle_path_markers',
            10
        )
        
        # 定时器：控制循环 (50Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # 定时器：可视化更新 (10Hz)
        self.viz_timer = self.create_timer(0.1, self.publish_visualization)
        
        self.get_logger().info('CirclePathTracker节点已启动')
        self.get_logger().info('等待当前位置 (/vrpn_mocap/rm_0_Test/pose) 和目标点 (/target_point)...')

    def pose_callback(self, msg: PoseStamped):
        """接收当前位置"""
        self.current_pose = msg.pose
        
        # 如果还没有路径且已有目标点，计算圆形路径
        if not self.path_ready and self.target_point is not None:
            self.calculate_circle_path()

    def target_callback(self, msg: Point):
        """接收目标点（只记录第一次收到的目标点）"""
        # 如果路径已经生成，不再接收新的目标点
        if self.path_ready:
            self.get_logger().info('路径已生成，忽略新的目标点')
            return
        
        # 如果已经有目标点了，不再更新
        if self.target_point is not None:
            return
            
        self.target_point = msg
        self.get_logger().info(f'收到目标点: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})')
        
        # 记录起始位置（收到target point时的pose，用于画圆）
        if self.current_pose is not None:
            self.start_position = {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y
            }
            self.get_logger().info(f'记录起始位置: ({self.start_position["x"]:.3f}, {self.start_position["y"]:.3f})')
            self.calculate_circle_path()

    def calculate_circle_path(self):
        """计算圆形路径（使用start_position和target_point）"""
        if self.start_position is None or self.target_point is None:
            return
        
        # 获取起始位置坐标（用于画圆）
        p1_x = self.start_position['x']
        p1_y = self.start_position['y']
        
        # 获取目标点坐标
        p2_x = self.target_point.x
        p2_y = self.target_point.y
        
        # 计算两点之间的距离（直径）
        distance = math.sqrt((p2_x - p1_x)**2 + (p2_y - p1_y)**2)
        
        # 半径 = 直径 / 2
        self.circle_radius = distance / 2.0
        
        # 圆心 = 两点中点
        self.circle_center = {
            'x': (p1_x + p2_x) / 2.0,
            'y': (p1_y + p2_y) / 2.0
        }
        
        self.get_logger().info(f'起始位置: ({p1_x:.3f}, {p1_y:.3f})')
        self.get_logger().info(f'目标点: ({p2_x:.3f}, {p2_y:.3f})')
        self.get_logger().info(f'圆心: ({self.circle_center["x"]:.3f}, {self.circle_center["y"]:.3f})')
        self.get_logger().info(f'直径: {distance:.3f}, 半径: {self.circle_radius:.3f}')
        
        # 生成圆形路径
        self.generate_circle_path()
        
        self.path_ready = True
        self.current_path_index = 0
        self.get_logger().info(f'圆形路径已生成，共 {len(self.circle_path)} 个点')

    def generate_circle_path(self, num_points=200):
        """
        生成圆形路径点
        以圆心为中心，均匀采样整个圆周
        
        Args:
            num_points: 路径点数量
        """
        self.circle_path = []
        
        # 从0度开始，均匀采样整个圆周（逆时针方向）
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = self.circle_center['x'] + self.circle_radius * math.cos(angle)
            y = self.circle_center['y'] + self.circle_radius * math.sin(angle)
            self.circle_path.append({'x': x, 'y': y, 'angle': angle})
        
        self.get_logger().info('圆形路径点已生成（以圆心为中心）')

    def get_circle_path(self):
        """
        获取圆形路径（供外部调用）
        
        Returns:
            list: 包含路径点的列表，每个点是一个字典 {'x': x, 'y': y, 'angle': angle}
        """
        return self.circle_path.copy()

    def get_path_info(self):
        """
        获取路径信息（供外部调用）
        
        Returns:
            dict: 包含圆心、半径和路径点的字典
        """
        return {
            'center': self.circle_center,
            'radius': self.circle_radius,
            'path': self.circle_path.copy()
        }

    def find_nearest_path_index(self):
        """找到当前位置最近的路径点索引"""
        if self.current_pose is None or len(self.circle_path) == 0:
            return 0
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        min_dist = float('inf')
        nearest_idx = 0
        
        for i, point in enumerate(self.circle_path):
            dist = math.sqrt((current_x - point['x'])**2 + (current_y - point['y'])**2)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        return nearest_idx

    def publish_visualization(self):
        """发布可视化Marker到rviz2"""
        if not self.path_ready:
            return
        
        marker_array = MarkerArray()
        
        # 1. 黑色圆形路径（完整路径）
        path_marker = Marker()
        path_marker.header.frame_id = "world"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "circle_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.02  # 线宽
        path_marker.color.r = 0.0
        path_marker.color.g = 0.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0  # 黑色
        path_marker.pose.orientation.w = 1.0
        
        for point in self.circle_path:
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = 0.0
            path_marker.points.append(p)
        # 闭合圆
        p = Point()
        p.x = self.circle_path[0]['x']
        p.y = self.circle_path[0]['y']
        p.z = 0.0
        path_marker.points.append(p)
        
        marker_array.markers.append(path_marker)
        
        # 2. 蓝色小车方块
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
            car_marker.scale.x = 0.15  # 长
            car_marker.scale.y = 0.10  # 宽
            car_marker.scale.z = 0.05  # 高
            car_marker.color.r = 0.0
            car_marker.color.g = 0.0
            car_marker.color.b = 1.0
            car_marker.color.a = 1.0  # 蓝色
            
            marker_array.markers.append(car_marker)
        
        # 3. 蓝色未来路径（从当前位置开始的半圈路径）
        if self.current_pose is not None:
            future_marker = Marker()
            future_marker.header.frame_id = "world"
            future_marker.header.stamp = self.get_clock().now().to_msg()
            future_marker.ns = "future_path"
            future_marker.id = 2
            future_marker.type = Marker.LINE_STRIP
            future_marker.action = Marker.ADD
            future_marker.scale.x = 0.03  # 线宽（比完整路径粗一点）
            future_marker.color.r = 0.0
            future_marker.color.g = 0.5
            future_marker.color.b = 1.0
            future_marker.color.a = 0.8  # 浅蓝色
            future_marker.pose.orientation.w = 1.0
            
            # 找到最近的路径点，显示未来50个点
            nearest_idx = self.find_nearest_path_index()
            num_future_points = min(100, len(self.circle_path))
            
            for i in range(num_future_points):
                idx = (nearest_idx + i) % len(self.circle_path)
                p = Point()
                p.x = self.circle_path[idx]['x']
                p.y = self.circle_path[idx]['y']
                p.z = 0.01  # 稍微高一点，避免与黑色路径重叠
                future_marker.points.append(p)
            
            marker_array.markers.append(future_marker)
        
        # 4. 红色圆心标记
        center_marker = Marker()
        center_marker.header.frame_id = "world"
        center_marker.header.stamp = self.get_clock().now().to_msg()
        center_marker.ns = "center"
        center_marker.id = 3
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.pose.position.x = self.circle_center['x']
        center_marker.pose.position.y = self.circle_center['y']
        center_marker.pose.position.z = 0.0
        center_marker.pose.orientation.w = 1.0
        center_marker.scale.x = 0.08
        center_marker.scale.y = 0.08
        center_marker.scale.z = 0.08
        center_marker.color.r = 1.0
        center_marker.color.g = 0.0
        center_marker.color.b = 0.0
        center_marker.color.a = 1.0  # 红色
        
        marker_array.markers.append(center_marker)
        
        # 5. 绿色半径误差指示线（从圆心到当前位置）
        if self.current_pose is not None:
            radius_marker = Marker()
            radius_marker.header.frame_id = "world"
            radius_marker.header.stamp = self.get_clock().now().to_msg()
            radius_marker.ns = "radius_error"
            radius_marker.id = 4
            radius_marker.type = Marker.LINE_STRIP
            radius_marker.action = Marker.ADD
            radius_marker.scale.x = 0.01
            radius_marker.pose.orientation.w = 1.0
            
            # 计算半径误差来决定颜色
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            distance_to_center = math.sqrt(
                (current_x - self.circle_center['x'])**2 + 
                (current_y - self.circle_center['y'])**2
            )
            radius_error = abs(distance_to_center - self.circle_radius)
            
            # 根据误差大小显示不同颜色：绿色=正常，黄色=轻微偏离，红色=严重偏离
            if radius_error < self.radius_error_threshold:
                radius_marker.color.r = 0.0
                radius_marker.color.g = 1.0
                radius_marker.color.b = 0.0  # 绿色
            elif radius_error < self.radius_error_threshold * 2:
                radius_marker.color.r = 1.0
                radius_marker.color.g = 1.0
                radius_marker.color.b = 0.0  # 黄色
            else:
                radius_marker.color.r = 1.0
                radius_marker.color.g = 0.0
                radius_marker.color.b = 0.0  # 红色
            radius_marker.color.a = 1.0
            
            # 圆心点
            p1 = Point()
            p1.x = self.circle_center['x']
            p1.y = self.circle_center['y']
            p1.z = 0.0
            radius_marker.points.append(p1)
            
            # 当前位置点
            p2 = Point()
            p2.x = current_x
            p2.y = current_y
            p2.z = 0.0
            radius_marker.points.append(p2)
            
            marker_array.markers.append(radius_marker)
            
            # 6. 文字显示半径误差
            text_marker = Marker()
            text_marker.header.frame_id = "world"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "error_text"
            text_marker.id = 5
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = self.circle_center['x']
            text_marker.pose.position.y = self.circle_center['y']
            text_marker.pose.position.z = 0.3
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.1  # 文字大小
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"Error: {radius_error:.3f}m"
            
            marker_array.markers.append(text_marker)
        
        # 发布所有Marker
        self.marker_pub.publish(marker_array)

    def control_loop(self):
        """控制循环：沿圆形路径行驶，带PID修正"""
        if not self.path_ready:
            return
        
        if self.current_pose is None:
            return
        
        # 获取当前位置
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # 计算当前位置到圆心的距离
        distance_to_center = math.sqrt(
            (current_x - self.circle_center['x'])**2 + 
            (current_y - self.circle_center['y'])**2
        )
        
        # 计算半径误差（正值表示在圆外，负值表示在圆内）
        radius_error = distance_to_center - self.circle_radius
        
        # 基础速度
        base_linear_speed = self.target_speed  # 0.33 m/s
        base_angular_speed = self.target_speed / self.circle_radius  # v = ω * r
        
        # PID修正速度
        correction_speed = 0.0
        
        # 只有当偏差大于阈值时才进行PID修正
        if abs(radius_error) > self.radius_error_threshold:
            # PID计算
            self.integral_error += radius_error * 0.02  # dt = 0.02s
            derivative_error = (radius_error - self.last_error) / 0.02
            
            # PID输出（修正角速度）
            correction_speed = (self.kp * radius_error + 
                               self.ki * self.integral_error + 
                               self.kd * derivative_error)
            
            # 限制积分误差，防止积分饱和
            self.integral_error = max(-1.0, min(1.0, self.integral_error))
            
            self.get_logger().debug(f'半径误差: {radius_error:.3f}, 修正速度: {correction_speed:.3f}')
        
        self.last_error = radius_error
        
        # 创建速度命令
        cmd_vel = Twist()
        cmd_vel.linear.x = base_linear_speed  # 线速度保持0.33 m/s
        
        # 角速度 = 基础角速度 + 修正角速度
        # 如果在圆外（radius_error > 0），需要增加角速度向内转
        # 如果在圆内（radius_error < 0），需要减少角速度向外转
        cmd_vel.angular.z = base_angular_speed + correction_speed
        
        # 限制角速度范围
        max_angular_speed = 2.0
        cmd_vel.angular.z = max(-max_angular_speed, min(max_angular_speed, cmd_vel.angular.z))
        
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
    
    node = CirclePathTracker()
    
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
