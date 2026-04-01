#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs  # 注册 PointStamped 的 do_transform

import json
import numpy as np

class ImageConversionNode(Node):
    def __init__(self):
        super().__init__('image_conversion_node')
        
        self.bridge = CvBridge()
        
        # 声明参数
        self.declare_parameter('pixel_path_topic', '/llm_pixels')
        self.declare_parameter('depth_topic', '/depth_cam/depth0/image_raw') 
        self.declare_parameter('camera_info_topic', '/depth_cam/depth0/camera_info')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('target_frame', 'odom')  # TF2 目标坐标系，可切换为 'map'
        self.declare_parameter('tf_timeout', 0.5)        # TF2 查询超时（秒）
        
        pixel_path_topic = self.get_parameter('pixel_path_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        
        # 内参：初始值为 None，等待从 camera_info 动态获取
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.img_width = None
        self.img_height = None
        self.camera_info_received = False
        
        # 订阅和发布
        self.sub_camera_info = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.sub_depth = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.sub_pixels = self.create_subscription(String, pixel_path_topic, self.pixel_callback, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        
        self.latest_depth = None
        
        # TF2：用于将 depth_camera_link 坐标系的点变换到目标坐标系（odom/map）
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(
            f"Image Conversion Node 已启动。目标坐标系: {self.target_frame}。"
            f"等待 camera_info ({camera_info_topic}) 和 /llm_pixels ...")

    def camera_info_callback(self, msg):
        """从 CameraInfo 动态获取相机内参，只在首次接收时记录日志。"""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.img_width = msg.width
        self.img_height = msg.height
        
        if not self.camera_info_received:
            self.camera_info_received = True
            self.get_logger().info(
                f"已获取相机内参: fx={self.fx:.2f}, fy={self.fy:.2f}, "
                f"cx={self.cx:.2f}, cy={self.cy:.2f}, "
                f"分辨率={self.img_width}x{self.img_height}")

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth 换算错误: {e}")

    def pixel_callback(self, msg):
        if not self.camera_info_received:
            self.get_logger().warn("收到 LLM 像素，但尚未收到 camera_info，放弃处理！")
            return
            
        if self.latest_depth is None:
            self.get_logger().warn("收到 LLM 像素，但未收到深度图像，放弃处理！")
            return
            
        try:
            points_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("无法解析 LLM 发送的 JSON 数据！")
            return
            
        depth_img = self.latest_depth.copy()
        
        now_stamp = self.get_clock().now().to_msg()
        source_frame = 'depth_camera_link'
        
        # 预先检查 TF2 变换是否可用
        tf_available = False
        try:
            self.tf_buffer.lookup_transform(
                self.target_frame, source_frame,
                rclpy.time.Time(),  # 使用最新可用变换
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
            tf_available = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF2 变换 {source_frame} → {self.target_frame} 不可用: {e}。"
                f"将以 {source_frame} 坐标系发布路径（无变换）。")
        
        # 根据 TF 是否可用决定输出坐标系
        output_frame = self.target_frame if tf_available else source_frame
        
        path_msg = Path()
        path_msg.header.stamp = now_stamp
        path_msg.header.frame_id = output_frame
        
        valid_points = 0
        tf_fail_count = 0
        
        for i, item in enumerate(points_data):
            pt = item.get("point")
            if not pt or len(pt) < 2:
                continue
                
            # 归一化 [0,1000] 转为像素 (配合发送端)
            u = int(pt[0] / 1000.0 * self.img_width)
            v = int(pt[1] / 1000.0 * self.img_height)
            
            u = max(0, min(u, self.img_width - 1))
            v = max(0, min(v, self.img_height - 1))
            
            # 5x5 窗口中值深度采样，抗噪声和空洞
            half_w = 2
            v_min = max(0, v - half_w)
            v_max = min(depth_img.shape[0], v + half_w + 1)
            u_min = max(0, u - half_w)
            u_max = min(depth_img.shape[1], u + half_w + 1)
            patch = depth_img[v_min:v_max, u_min:u_max].flatten()
            valid_depths = patch[patch > 0]
            if len(valid_depths) == 0:
                continue
            depth_mm = float(np.median(valid_depths))
                
            z_in_meters = float(depth_mm) / 1000.0
            x_in_meters = (u - self.cx) * z_in_meters / self.fx
            y_in_meters = (v - self.cy) * z_in_meters / self.fy
            
            # --- TF2 坐标变换：depth_camera_link → target_frame ---
            if tf_available:
                point_in_cam = PointStamped()
                point_in_cam.header.stamp = now_stamp
                point_in_cam.header.frame_id = source_frame
                point_in_cam.point.x = x_in_meters
                point_in_cam.point.y = y_in_meters
                point_in_cam.point.z = z_in_meters
                
                try:
                    point_in_target = self.tf_buffer.transform(
                        point_in_cam, self.target_frame,
                        timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
                    
                    pose = PoseStamped()
                    pose.header.stamp = now_stamp
                    pose.header.frame_id = self.target_frame
                    pose.pose.position.x = point_in_target.point.x
                    pose.pose.position.y = point_in_target.point.y
                    pose.pose.position.z = point_in_target.point.z
                    pose.pose.orientation.w = 1.0
                    
                    path_msg.poses.append(pose)
                    valid_points += 1
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    tf_fail_count += 1
            else:
                # TF 不可用时，直接以相机坐标系发布（降级模式）
                pose = PoseStamped()
                pose.header.stamp = now_stamp
                pose.header.frame_id = source_frame
                pose.pose.position.x = x_in_meters
                pose.pose.position.y = y_in_meters
                pose.pose.position.z = z_in_meters
                pose.pose.orientation.w = 1.0
                
                path_msg.poses.append(pose)
                valid_points += 1
            
        if valid_points > 0:
            self.path_pub.publish(path_msg)
            frame_info = f"坐标系: {output_frame}"
            tf_info = f"（TF 变换失败 {tf_fail_count} 个点）" if tf_fail_count > 0 else ""
            self.get_logger().info(
                f"转换完成，已推送 {valid_points} 个 3D 点至 /path。{frame_info}{tf_info}")
        else:
            self.get_logger().warn("所有像素点均无有效深度数据，路径转换失败。")

def main(args=None):
    rclpy.init(args=args)
    node = ImageConversionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
