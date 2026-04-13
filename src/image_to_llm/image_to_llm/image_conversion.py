#!/usr/bin/env python3
"""
image_conversion 节点：将 LLM 输出的像素坐标转换为 odom 坐标系下的路径点。

核心逻辑（深度相机针孔模型反投影）：
  1. 订阅深度图 + camera_info，获取每个像素的真实深度值
  2. 从 Gemini 像素坐标中，用针孔相机模型将 (u, v, depth) 反投影为相机坐标系下的 3D 点：
       Z = depth[v, u]  (mm → m)
       X = (u - cx) * Z / fx
       Y = (v - cy) * Z / fy
  3. 将相机坐标系 3D 点转换到 odom 坐标系（利用机器人 yaw + odom 位置）
  4. 发布到 /path 供 track_path 跟踪
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import json
import math
import os
import numpy as np
from datetime import datetime
from cv_bridge import CvBridge


class ImageConversionNode(Node):
    def __init__(self):
        super().__init__('image_conversion_node')

        # ---- 参数声明 ----
        self.declare_parameter('pixel_path_topic', '/llm_pixels')
        self.declare_parameter('camera_info_topic', '/depth_cam/depth0/camera_info')
        self.declare_parameter('depth_image_topic', '/depth_cam/depth0/image_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('depth_search_radius', 5)       # 深度采样窗口半径（像素）
        self.declare_parameter('min_valid_depth', 0.1)         # 最小有效深度（米）
        self.declare_parameter('max_valid_depth', 5.0)         # 最大有效深度（米）
        self.declare_parameter('ground_plane_fallback', True)   # 深度无效时是否使用地面平面假设
        self.declare_parameter('camera_height', 0.15)           # 相机离地高度（米），用于地面平面 fallback
        self.declare_parameter('camera_pitch', 0.0)             # 相机俯仰角（弧度，正值=向下），用于地面平面 fallback
        self.declare_parameter('log_dir', '/home/kuko/humble_ws/data/log')

        pixel_path_topic = self.get_parameter('pixel_path_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_image_topic = self.get_parameter('depth_image_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.depth_search_radius = self.get_parameter('depth_search_radius').value
        self.min_valid_depth = self.get_parameter('min_valid_depth').value
        self.max_valid_depth = self.get_parameter('max_valid_depth').value
        self.ground_plane_fallback = self.get_parameter('ground_plane_fallback').value
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_pitch = self.get_parameter('camera_pitch').value
        self.log_dir = self.get_parameter('log_dir').value
        os.makedirs(self.log_dir, exist_ok=True)

        # ---- 内参（从 camera_info 动态获取）----
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.img_width = None
        self.img_height = None
        self.camera_info_received = False

        # ---- 深度图数据 ----
        self.latest_depth = None       # numpy array, 单位：米（float）
        self.depth_received = False
        self.bridge = CvBridge()

        # ---- Odom 数据 ----
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # ---- 就绪状态 ----
        self._ready_logged = False
        self._pending_pixel_msg = None

        # ---- QoS for depth image (BEST_EFFORT to match camera publisher) ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---- 订阅 ----
        self.sub_camera_info = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.sub_depth = self.create_subscription(
            Image, depth_image_topic, self.depth_callback, sensor_qos)
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        self.sub_pixels = self.create_subscription(
            String, pixel_path_topic, self.pixel_callback, 10)

        # ---- 发布 ----
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        self.get_logger().info(
            "Image Conversion Node 已启动（深度相机针孔模型反投影模式）")
        self.get_logger().info(
            f"⏳ 等待 camera_info + 深度图 + odom 就绪...")
        self.get_logger().info(
            f"  深度图话题: {depth_image_topic}")
        self.get_logger().info(
            f"  camera_info 话题: {camera_info_topic}")

    # ================================================================
    #  数据源回调
    # ================================================================

    def camera_info_callback(self, msg):
        """从 CameraInfo 获取相机内参 fx, fy, cx, cy。"""
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
            self._check_ready()

    def depth_callback(self, msg):
        """接收深度图，转换为 numpy 数组（单位：米）。
        
        支持 16UC1/mono16（mm 单位）和 32FC1（m 单位）两种编码。
        """
        try:
            if msg.encoding in ('16UC1', 'mono16'):
                depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                # uint16, 单位 mm → 转为 float, 单位 m
                self.latest_depth = depth_raw.astype(np.float32) / 1000.0
            elif msg.encoding == '32FC1':
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            else:
                self.get_logger().warn(
                    f"不支持的深度图编码: {msg.encoding}", throttle_duration_sec=10.0)
                return

            if not self.depth_received:
                self.depth_received = True
                valid_count = np.count_nonzero(
                    (self.latest_depth > self.min_valid_depth) &
                    (self.latest_depth < self.max_valid_depth))
                total = self.latest_depth.size
                self.get_logger().info(
                    f"已收到首帧深度图: {msg.width}x{msg.height}, "
                    f"encoding={msg.encoding}, "
                    f"有效像素: {valid_count}/{total} ({valid_count/total*100:.1f}%)")
                self._check_ready()
        except Exception as e:
            self.get_logger().error(f"深度图转换错误: {e}", throttle_duration_sec=5.0)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.current_x = p.x
        self.current_y = p.y
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        if not self._ready_logged:
            self._check_ready()

    def _check_ready(self):
        if self._ready_logged:
            return
        if self.camera_info_received and self.depth_received and self.current_yaw is not None:
            self._ready_logged = True
            self.get_logger().info(
                "✅ camera_info + 深度图 + odom 均已就绪，准备接收 /llm_pixels。")
            if self._pending_pixel_msg is not None:
                self.get_logger().info("📦 正在处理缓存的像素消息...")
                pending = self._pending_pixel_msg
                self._pending_pixel_msg = None
                self.pixel_callback(pending)

    # ================================================================
    #  像素坐标解码
    # ================================================================

    def _decode_pixel_coordinate(self, x_raw, y_raw, img_w, img_h):
        """兼容 [0,1000] 归一化坐标和直接像素坐标。"""
        if 0.0 <= x_raw <= 1000.0 and 0.0 <= y_raw <= 1000.0:
            u = int(x_raw / 1000.0 * img_w)
            v = int(y_raw / 1000.0 * img_h)
        else:
            u = int(x_raw)
            v = int(y_raw)
        u = max(0, min(u, img_w - 1))
        v = max(0, min(v, img_h - 1))
        return u, v

    # ================================================================
    #  深度采样（带窗口中值滤波）
    # ================================================================

    def _sample_depth(self, u, v):
        """在 (u, v) 周围的窗口内采样有效深度值，返回中值（米）。
        
        Returns:
            depth_m: 深度值（米），如果无有效深度则返回 None
        """
        if self.latest_depth is None:
            return None

        h, w = self.latest_depth.shape
        r = self.depth_search_radius

        # 窗口边界
        v_min = max(0, v - r)
        v_max = min(h, v + r + 1)
        u_min = max(0, u - r)
        u_max = min(w, u + r + 1)

        patch = self.latest_depth[v_min:v_max, u_min:u_max]

        # 筛选有效深度
        valid_mask = (patch > self.min_valid_depth) & (patch < self.max_valid_depth)
        valid_depths = patch[valid_mask]

        if len(valid_depths) == 0:
            return None

        return float(np.median(valid_depths))

    # ================================================================
    #  像素 → 相机坐标系 3D 点（针孔模型反投影）
    # ================================================================

    def _pixel_to_camera_3d(self, u, v, depth_m):
        """用针孔相机模型将 (u, v, depth) 反投影为相机坐标系下的 3D 点。
        
        相机坐标系约定（ROS 标准）：
          X = 右, Y = 下, Z = 前（光轴方向）
        
        Args:
            u, v: 像素坐标
            depth_m: 深度值（米）
            
        Returns:
            (cam_x, cam_y, cam_z): 相机坐标系下的 3D 坐标（米）
        """
        cam_z = depth_m                          # 前方距离
        cam_x = (u - self.cx) * depth_m / self.fx  # 水平偏移（右为正）
        cam_y = (v - self.cy) * depth_m / self.fy  # 垂直偏移（下为正）
        return cam_x, cam_y, cam_z

    # ================================================================
    #  地面平面 fallback：当深度无效时，假设像素在地面上
    # ================================================================

    def _ground_plane_fallback(self, u, v):
        """假设像素点在地面平面上，用相机高度和内参估算距离。
        
        原理：如果相机高度为 h，俯仰角为 pitch，则地面上的点满足：
          Y_cam = h（相机到地面的垂直距离）
          Z_cam = h * fy / (v - cy - fy * tan(pitch))
          
        Returns:
            (cam_x, cam_y, cam_z) 或 None（如果无法计算）
        """
        # v 必须在光心以下（地面在图像下半部分）
        v_offset = v - self.cy - self.fy * math.tan(self.camera_pitch)
        if v_offset <= 0:
            return None  # 像素在地平线以上，无法用地面假设

        cam_z = self.camera_height * self.fy / v_offset
        cam_x = (u - self.cx) * cam_z / self.fx
        cam_y = self.camera_height  # 地面高度

        # 合理性检查
        if cam_z < self.min_valid_depth or cam_z > self.max_valid_depth:
            return None

        return cam_x, cam_y, cam_z

    # ================================================================
    #  相机坐标系 → odom 坐标系
    # ================================================================

    def _camera_to_odom(self, cam_x, cam_y, cam_z):
        """将相机坐标系的 3D 点转换到 odom 坐标系。
        
        假设相机朝前安装（与机器人前进方向一致）：
          相机 Z（前）→ odom 中机器人前进方向
          相机 X（右）→ odom 中机器人右侧方向
          
        转换：
          odom_x = robot_x + cam_z * cos(yaw) - cam_x * sin(yaw)
          odom_y = robot_y + cam_z * sin(yaw) + cam_x * cos(yaw)
          
        Returns:
            (odom_x, odom_y)
        """
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)

        odom_x = self.current_x + cam_z * cos_yaw - cam_x * sin_yaw
        odom_y = self.current_y + cam_z * sin_yaw + cam_x * cos_yaw

        return odom_x, odom_y

    # ================================================================
    #  主逻辑：像素消息处理
    # ================================================================

    def pixel_callback(self, msg):
        # 检查就绪
        if not self.camera_info_received or self.current_yaw is None:
            if self._pending_pixel_msg is None:
                self.get_logger().warn(
                    "收到 LLM 像素，但数据源尚未就绪，已缓存等待就绪后自动处理。")
            self._pending_pixel_msg = msg
            return

        try:
            points_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("无法解析 LLM 发送的 JSON 数据！")
            return

        img_w = self.img_width
        img_h = self.img_height

        now_stamp = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.stamp = now_stamp
        path_msg.header.frame_id = 'odom'

        conversion_details = []  # 用于 debug 日志

        for i, item in enumerate(points_data):
            pt = item.get("point")
            if not pt or len(pt) < 2:
                continue

            u, v = self._decode_pixel_coordinate(float(pt[0]), float(pt[1]), img_w, img_h)

            # 尝试从深度图获取深度
            depth_m = self._sample_depth(u, v)
            method = 'depth'

            if depth_m is None and self.ground_plane_fallback:
                # 深度无效，尝试地面平面 fallback
                result = self._ground_plane_fallback(u, v)
                if result is not None:
                    cam_x, cam_y, cam_z = result
                    depth_m = cam_z
                    method = 'ground_plane'

            if depth_m is None:
                conversion_details.append({
                    'idx': i, 'raw': (pt[0], pt[1]), 'decoded': (u, v),
                    'depth': None, 'method': 'SKIPPED', 'reason': '无有效深度'
                })
                self.get_logger().warn(
                    f"像素点 {i} ({u},{v}) 无有效深度，跳过。")
                continue

            # 针孔模型反投影
            if method == 'depth':
                cam_x, cam_y, cam_z = self._pixel_to_camera_3d(u, v, depth_m)

            # 转换到 odom 坐标系
            odom_x, odom_y = self._camera_to_odom(cam_x, cam_y, cam_z)

            pose = PoseStamped()
            pose.header.stamp = now_stamp
            pose.header.frame_id = 'odom'
            pose.pose.position.x = odom_x
            pose.pose.position.y = odom_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

            conversion_details.append({
                'idx': i, 'raw': (pt[0], pt[1]), 'decoded': (u, v),
                'depth': depth_m, 'method': method,
                'cam_3d': (cam_x, cam_y, cam_z),
                'odom': (odom_x, odom_y)
            })

        if len(path_msg.poses) > 0:
            self.path_pub.publish(path_msg)
            depth_methods = [d['method'] for d in conversion_details if d.get('depth') is not None]
            depth_count = depth_methods.count('depth')
            gp_count = depth_methods.count('ground_plane')
            self.get_logger().info(
                f"✅ 已发布 {len(path_msg.poses)} 个路径点 [odom]。"
                f"深度反投影: {depth_count}, 地面假设: {gp_count}, "
                f"跳过: {len(points_data) - len(path_msg.poses)}")
        else:
            self.get_logger().warn("无法生成有效路径点（所有像素点深度均无效）。")

        # ---- 写入 debug 日志 ----
        self._write_debug_log(points_data, conversion_details, path_msg.poses)

    # ================================================================
    #  Debug 日志写入
    # ================================================================

    def _write_debug_log(self, points_data, conversion_details, poses):
        """将每次路径转换的详细信息写入 data/log/ 目录。"""
        try:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            log_path = os.path.join(self.log_dir, f'conversion_{ts}.log')

            lines = []
            lines.append(f"=== Image Conversion Debug Log (深度相机反投影模式) ===")
            lines.append(f"时间: {datetime.now().isoformat()}")
            lines.append(f"相机内参: fx={self.fx:.2f}, fy={self.fy:.2f}, "
                         f"cx={self.cx:.2f}, cy={self.cy:.2f}, "
                         f"分辨率={self.img_width}x{self.img_height}")
            lines.append(f"当前位置: x={self.current_x:.4f}, y={self.current_y:.4f}, "
                         f"yaw={math.degrees(self.current_yaw):.1f}°")
            lines.append(f"参数: depth_search_radius={self.depth_search_radius}, "
                         f"min_depth={self.min_valid_depth}m, max_depth={self.max_valid_depth}m, "
                         f"ground_fallback={self.ground_plane_fallback}")

            # 深度图统计
            if self.latest_depth is not None:
                valid = (self.latest_depth > self.min_valid_depth) & \
                        (self.latest_depth < self.max_valid_depth)
                vc = np.count_nonzero(valid)
                total = self.latest_depth.size
                lines.append(f"深度图有效率: {vc}/{total} ({vc/total*100:.1f}%)")
            lines.append(f"")

            # Gemini 原始像素坐标
            lines.append(f"--- Gemini 原始像素坐标 ({len(points_data)} 个点) ---")
            for i, item in enumerate(points_data):
                pt = item.get("point", [])
                desc = item.get("description", "")
                lines.append(f"  点{i}: pixel=({pt[0] if len(pt)>0 else '?'}, "
                             f"{pt[1] if len(pt)>1 else '?'}) desc=\"{desc}\"")
            lines.append(f"")

            # 转换详情
            lines.append(f"--- 像素 → 3D → odom 转换详情 ---")
            for d in conversion_details:
                if d.get('depth') is None:
                    lines.append(
                        f"  点{d['idx']}: raw={d['raw']} → decoded={d['decoded']} "
                        f"→ {d['method']}: {d.get('reason', '')}")
                else:
                    cam = d.get('cam_3d', (0, 0, 0))
                    odom = d.get('odom', (0, 0))
                    lines.append(
                        f"  点{d['idx']}: raw={d['raw']} → decoded={d['decoded']} "
                        f"→ depth={d['depth']:.3f}m ({d['method']}) "
                        f"→ cam=({cam[0]:.3f},{cam[1]:.3f},{cam[2]:.3f}) "
                        f"→ odom=({odom[0]:.4f},{odom[1]:.4f})")
            lines.append(f"")

            # 输出路径点
            lines.append(f"--- 最终输出路径 ({len(poses)} 个点, frame=odom) ---")
            for i, pose in enumerate(poses):
                p = pose.pose.position
                lines.append(f"  路径点{i}: x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}")
            lines.append(f"")
            lines.append(f"=== END ===")

            with open(log_path, 'w') as f:
                f.write('\n'.join(lines))
            self.get_logger().info(f"📝 Debug 日志已写入: {log_path}")

        except Exception as e:
            self.get_logger().error(f"写入 debug 日志失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageConversionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
