#!/usr/bin/env python3
"""
ROS2节点：订阅当前位置和目标点，计算圆形路径并以固定速度跟踪
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist, Point
import numpy as np
import math


class CirclePathTracker(Node):
    def __init__(self):
        super().__init__('circle_path_tracker')
        
        # 参数设置
        self.target_speed = 0.33  # 目标速度 m/s
        
        # 状态变量
        self.current_pose = None  # 当前位置
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
        
        # 定时器：控制循环 (50Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
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
        
        # 如果已有当前位置，计算圆形路径
        if self.current_pose is not None:
            self.calculate_circle_path()

    def calculate_circle_path(self):
        """计算圆形路径"""
        if self.current_pose is None or self.target_point is None:
            return
        
        # 获取当前位置坐标
        p1_x = self.current_pose.position.x
        p1_y = self.current_pose.position.y
        
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
        
        self.get_logger().info(f'当前位置: ({p1_x:.3f}, {p1_y:.3f})')
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

    def control_loop(self):
        """控制循环：沿圆形路径行驶"""
        if not self.path_ready:
            return
        
        # 计算沿圆形路径的速度
        # 线速度 = 0.33 m/s
        # 角速度 = 线速度 / 半径 (v = ω * r)
        angular_speed = self.target_speed / self.circle_radius
        
        # 创建速度命令
        cmd_vel = Twist()
        cmd_vel.linear.x = self.target_speed  # 线速度 0.33 m/s
        cmd_vel.angular.z = angular_speed      # 角速度使机器人沿圆行驶
        
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
