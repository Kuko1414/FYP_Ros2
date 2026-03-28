#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import json
import numpy as np

class ImageConversionNode(Node):
    def __init__(self):
        super().__init__('image_conversion_node')
        
        self.bridge = CvBridge()
        
        # 声明参数
        self.declare_parameter('pixel_path_topic', '/llm_pixels')
        self.declare_parameter('depth_topic', '/depth_cam/depth0/image_raw') 
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('rgb_width', 640)
        self.declare_parameter('rgb_height', 400)
        
        pixel_path_topic = self.get_parameter('pixel_path_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        path_topic = self.get_parameter('path_topic').value
        
        self.img_width = self.get_parameter('rgb_width').value
        self.img_height = self.get_parameter('rgb_height').value
        
        # 内参
        self.fx = 423.2929992675781
        self.fy = 423.9620056152344
        self.cx = 322.1520080566406
        self.cy = 200.72999572753906
        
        # 订阅和发布
        self.sub_depth = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.sub_pixels = self.create_subscription(String, pixel_path_topic, self.pixel_callback, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        
        self.latest_depth = None
        
        self.get_logger().info("Image Conversion Node 已启动。等待 /llm_pixels ...")

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth 换算错误: {e}")

    def pixel_callback(self, msg):
        if self.latest_depth is None:
            self.get_logger().warn("收到 LLM 像素，但未收到深度图像，放弃处理！")
            return
            
        try:
            points_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("无法解析 LLM 发送的 JSON 数据！")
            return
            
        depth_img = self.latest_depth.copy()
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'depth_camera_link'
        
        valid_points = 0
        for i, item in enumerate(points_data):
            pt = item.get("point")
            if not pt or len(pt) < 2:
                continue
                
            # 归一化 [0,1000] 转为像素 (配合发送端)
            u = int(pt[0] / 1000.0 * self.img_width)
            v = int(pt[1] / 1000.0 * self.img_height)
            
            u = max(0, min(u, self.img_width - 1))
            v = max(0, min(v, self.img_height - 1))
            
            depth_mm = depth_img[v, u]
            if depth_mm == 0:
                continue
                
            z_in_meters = float(depth_mm) / 1000.0
            x_in_meters = (u - self.cx) * z_in_meters / self.fx
            y_in_meters = (v - self.cy) * z_in_meters / self.fy
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x_in_meters
            pose.pose.position.y = y_in_meters
            pose.pose.position.z = z_in_meters
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
            valid_points += 1
            
        if valid_points > 0:
            self.path_pub.publish(path_msg)
            self.get_logger().info(f"转换完成，已推送 {valid_points} 个 3D 点至 /path。")
        else:
            self.get_logger().warn("所有像素点均无无有效深度数据，路径转换失败。")

def main(args=None):
    rclpy.init(args=args)
    node = ImageConversionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()