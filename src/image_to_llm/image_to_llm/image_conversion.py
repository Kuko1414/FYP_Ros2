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
from geometry_msgs.msg import PoseStamped, PointStamped

import tf2_ros
import tf2_geometry_msgs
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import json
import math
import os
import time
import numpy as np
from collections import deque
from datetime import datetime
from cv_bridge import CvBridge


class ImageConversionNode(Node):
    def __init__(self):
        super().__init__('image_conversion_node')

        # ---- 参数声明 ----
        self.declare_parameter('pixel_path_topic', '/llm_pixels')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('depth_search_radius', 5)       # 深度采样窗口半径（像素）
        self.declare_parameter('min_valid_depth', 0.1)         # 最小有效深度（米）
        self.declare_parameter('max_valid_depth', 5.0)         # 最大有效深度（米）
        self.declare_parameter('ground_plane_fallback', True)   # 深度无效时是否使用地面平面假设
        self.declare_parameter('camera_height', 0.15)           # 相机离地高度（米），用于地面平面 fallback
        self.declare_parameter('camera_pitch', 0.0)             # 相机俯仰角（弧度，正值=向下），用于地面平面 fallback
        self.declare_parameter('ground_plane_max_depth', 2.0)   # ground_plane fallback 最大允许深度（米）
        self.declare_parameter('direction_filter_enabled', True) # 是否启用路径点方向一致性过滤（防掉头）
        self.declare_parameter('direction_anchor_count', 2)     # 前 N 个点作为锚点，不参与方向过滤
        self.declare_parameter('direction_max_angle_deg', 90.0) # 方向偏差超过此角度的点被视为掉头并丢弃
        self.declare_parameter('log_dir', '/home/kuko/humble_ws/data/log')
        self.declare_parameter('task_id', -1)  # task 编号，-1 表示使用时间戳命名

        pixel_path_topic = self.get_parameter('pixel_path_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_image_topic = self.get_parameter('depth_image_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = odom_topic.lstrip('/')
        path_topic = self.get_parameter('path_topic').value
        self.depth_search_radius = self.get_parameter('depth_search_radius').value
        self.min_valid_depth = self.get_parameter('min_valid_depth').value
        self.max_valid_depth = self.get_parameter('max_valid_depth').value
        self.ground_plane_fallback = self.get_parameter('ground_plane_fallback').value
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_pitch = self.get_parameter('camera_pitch').value
        self.ground_plane_max_depth = self.get_parameter('ground_plane_max_depth').value
        self.direction_filter_enabled = self.get_parameter('direction_filter_enabled').value
        self.direction_anchor_count = self.get_parameter('direction_anchor_count').value
        self.direction_max_angle_rad = math.radians(self.get_parameter('direction_max_angle_deg').value)
        self.log_dir = self.get_parameter('log_dir').value
        self.task_id = self.get_parameter('task_id').value
        self._conversion_count = 0  # 当前 task 内的第几次转换
        os.makedirs(self.log_dir, exist_ok=True)

        # ---- 内参（从 camera_info 动态获取）----
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.img_width = None
        self.img_height = None
        self.camera_info_received = False

        # ROS 2 TF 配置
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 深度图数据 ----
        self.latest_depth = None       # numpy array, 单位：米（float）
        self.depth_received = False
        self.bridge = CvBridge()
        self.latest_depth_stamp = None # 保存时间戳

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
            CameraInfo, camera_info_topic, self.camera_info_callback, sensor_qos)
        self.sub_depth = self.create_subscription(
            Image, depth_image_topic, self.depth_callback, sensor_qos)
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
            
            # 保存时间戳供后续 TF 转换使用
            self.latest_depth_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"深度图转换错误: {e}", throttle_duration_sec=5.0)

    def _check_ready(self):
        if self._ready_logged:
            return
        if self.camera_info_received and self.depth_received:
            self._ready_logged = True
            self.get_logger().info(
                f"✅ camera_info + 深度图 均已就绪，准备接收 /llm_pixels。")
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

        # 合理性检查：使用 ground_plane 专用的最大深度限制
        if cam_z < self.min_valid_depth or cam_z > self.ground_plane_max_depth:
            return None

        return cam_x, cam_y, cam_z

    # ================================================================
    #  路径点方向一致性过滤（防掉头）
    # ================================================================

    def _filter_backward_points(self, poses, conversion_details):
        """过滤掉方向反转（掉头）的路径点。
        
        原理：
          1. 前 N 个点（anchor_count）作为锚点，无条件保留（近距离深度可靠）
          2. 用前几个锚点确定整体前进方向角（reference_heading）
          3. 后续每个点：计算它相对于上一个保留点的方向角，
             如果与 reference_heading 偏差 > max_angle，则判定为掉头并丢弃
        
        Args:
            poses: list of PoseStamped（已转换到 odom 的路径点）
            conversion_details: 对应的 debug 详情列表（仅已成功转换的）
            
        Returns:
            (filtered_poses, filtered_details, removed_count)
        """
        if not self.direction_filter_enabled or len(poses) <= self.direction_anchor_count:
            return poses, conversion_details, 0

        # 提取成功转换的 detail（有 odom 坐标的）
        valid_details = [d for d in conversion_details if d.get('odom') is not None]
        
        # poses 和 valid_details 应该一一对应
        if len(valid_details) != len(poses):
            # 数量不匹配时放弃过滤，安全起见
            return poses, conversion_details, 0

        # 用前几个锚点确定整体前进方向
        anchor_end = min(self.direction_anchor_count, len(poses))
        if anchor_end < 2:
            # 不足 2 个点无法判定方向
            return poses, conversion_details, 0

        # reference_heading: 从第一个锚点到最后一个锚点的方向
        p0 = poses[0].pose.position
        p_anchor = poses[anchor_end - 1].pose.position
        dx_ref = p_anchor.x - p0.x
        dy_ref = p_anchor.y - p0.y
        if math.hypot(dx_ref, dy_ref) < 0.01:
            # 锚点太密集，无法确定方向，放弃过滤
            return poses, conversion_details, 0
        reference_heading = math.atan2(dy_ref, dx_ref)

        filtered_poses = list(poses[:anchor_end])
        filtered_details = list(valid_details[:anchor_end])
        removed_count = 0
        last_kept = poses[anchor_end - 1]

        for j in range(anchor_end, len(poses)):
            curr = poses[j]
            dx = curr.pose.position.x - last_kept.pose.position.x
            dy = curr.pose.position.y - last_kept.pose.position.y
            step_dist = math.hypot(dx, dy)

            if step_dist < 0.01:
                # 距离太近，保留（不影响方向判断）
                filtered_poses.append(curr)
                filtered_details.append(valid_details[j])
                last_kept = curr
                continue

            step_heading = math.atan2(dy, dx)
            # 计算与整体前进方向的偏差
            angle_diff = abs(math.atan2(
                math.sin(step_heading - reference_heading),
                math.cos(step_heading - reference_heading)))

            if angle_diff > self.direction_max_angle_rad:
                # 方向反转，丢弃此点
                removed_count += 1
                # 标记 detail 为被方向过滤丢弃
                valid_details[j]['method'] = 'DIR_FILTERED'
                valid_details[j]['reason'] = (
                    f'方向偏差 {math.degrees(angle_diff):.1f}° > '
                    f'{math.degrees(self.direction_max_angle_rad):.0f}°')
                self.get_logger().warn(
                    f"🔄 路径点 {valid_details[j]['idx']} 方向偏差 "
                    f"{math.degrees(angle_diff):.1f}°，判定为掉头，已丢弃")
            else:
                filtered_poses.append(curr)
                filtered_details.append(valid_details[j])
                last_kept = curr

        return filtered_poses, filtered_details, removed_count

    # ================================================================
    #  主逻辑：像素消息处理
    # ================================================================

    def pixel_callback(self, msg):
        # 检查就绪（camera_info + 深度图 全部就绪才处理）
        if not self.camera_info_received or not self.depth_received:
            reasons = []
            if not self.camera_info_received:
                reasons.append("camera_info")
            if not self.depth_received:
                reasons.append("深度图")
            if self._pending_pixel_msg is None:
                self.get_logger().warn(
                    f"收到 LLM 像素，但 {'+'.join(reasons)} 尚未就绪，已缓存等待就绪后自动处理。")
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
        path_msg.header.frame_id = self.odom_frame

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

            # 构造带时间戳的相机坐标点
            pt_cam = PointStamped()
            # 解决TF extrapolation into the future错误：使用最新可用的TF（stamp赋为0）
            pt_cam.header.stamp.sec = 0
            pt_cam.header.stamp.nanosec = 0
            pt_cam.header.frame_id = 'camera_depth_optical_frame'
            pt_cam.point.x = float(cam_x)
            pt_cam.point.y = float(cam_y)
            pt_cam.point.z = float(cam_z)

            try:
                # 使用 TF2 进行转换: camera *optical* frame => odom frame
                # rclpy.duration.Duration 的 timeout 可以稍微解决一些历史树的抖动
                pt_odom = self.tf_buffer.transform(pt_cam, self.odom_frame, timeout=rclpy.duration.Duration(seconds=0.1))
                odom_x = pt_odom.point.x
                odom_y = pt_odom.point.y
            except Exception as e:
                self.get_logger().warn(f"无法将点从 {pt_cam.header.frame_id} 转换到 {self.odom_frame}: {e}")
                conversion_details.append({
                    'idx': i, 'raw': (pt[0], pt[1]), 'decoded': (u, v),
                    'depth': depth_m, 'method': 'TF_FAILED', 'reason': str(e)
                })
                continue

            pose = PoseStamped()
            pose.header.stamp = now_stamp
            pose.header.frame_id = self.odom_frame
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

        # ---- 方向一致性过滤（防掉头）----
        pre_filter_count = len(path_msg.poses)
        if len(path_msg.poses) > 0:
            filtered_poses, filtered_valid_details, dir_removed = \
                self._filter_backward_points(list(path_msg.poses), conversion_details)
            path_msg.poses = filtered_poses
            # 将被方向过滤的 detail 也合并回 conversion_details（已在原地标记）
        else:
            dir_removed = 0

        if len(path_msg.poses) > 0:
            self.path_pub.publish(path_msg)
            depth_methods = [d['method'] for d in conversion_details
                             if d.get('depth') is not None and d['method'] not in ('DIR_FILTERED',)]
            depth_count = depth_methods.count('depth')
            gp_count = depth_methods.count('ground_plane')
            skip_count = len(points_data) - pre_filter_count
            self.get_logger().info(
                f"✅ 已发布 {len(path_msg.poses)} 个路径点 [odom]。"
                f"深度反投影: {depth_count}, 地面假设: {gp_count}, "
                f"跳过: {skip_count}, 方向过滤: {dir_removed}")
        else:
            self.get_logger().warn("无法生成有效路径点（所有像素点深度均无效或方向不一致）。")

        # ---- 写入 debug 日志 ----
        self._write_debug_log(points_data, conversion_details, path_msg.poses)

    # ================================================================
    #  Debug 日志写入
    # ================================================================

    def _write_debug_log(self, points_data, conversion_details, poses):
        """将每次路径转换的详细信息写入 data/log/ 目录。"""
        try:
            # 根据 task_id 决定文件名：task_id >= 0 时用 task 编号，否则用时间戳
            if self.task_id >= 0:
                self._conversion_count += 1
                log_path = os.path.join(self.log_dir, f'conversion_task{self.task_id}_{self._conversion_count}.log')
            else:
                ts = datetime.now().strftime('%Y%m%d_%H%M%S')
                log_path = os.path.join(self.log_dir, f'conversion_{ts}.log')

            lines = []
            lines.append(f"=== Image Conversion Debug Log (深度相机反投影模式) ===")
            lines.append(f"时间: {datetime.now().isoformat()}")
            lines.append(f"相机内参: fx={self.fx:.2f}, fy={self.fy:.2f}, "
                         f"cx={self.cx:.2f}, cy={self.cy:.2f}, "
                         f"分辨率={self.img_width}x{self.img_height}")
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
                if d['method'] == 'DIR_FILTERED':
                    cam = d.get('cam_3d', (0, 0, 0))
                    odom = d.get('odom', (0, 0))
                    lines.append(
                        f"  点{d['idx']}: raw={d['raw']} → decoded={d['decoded']} "
                        f"→ depth={d['depth']:.3f}m → odom=({odom[0]:.4f},{odom[1]:.4f}) "
                        f"→ DIR_FILTERED: {d.get('reason', '')}")
                elif d.get('depth') is None:
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
