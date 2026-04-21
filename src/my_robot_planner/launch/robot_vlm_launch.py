"""
启动下游处理链路（单独终端运行，方便查看坐标转换和路径跟踪日志）：
  1. image_conversion   — 深度相机针孔模型反投影，像素→3D→odom 路径点
  2. track_path          — 纯 PID 路径点追踪 + 深度图障碍物检测

用法（终端 2）:
  ros2 launch my_robot_planner robot_vlm_launch.py

注意: 需要在另一个终端先启动 image_to_llm_node:
  ros2 launch image_to_llm llm_node_launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    task_id_arg = DeclareLaunchArgument(
        'task_id', default_value='-1',
        description='实验 task 编号（>=0 时 conversion log 按 taskN 命名，-1 时用时间戳）'
    )

    # ---- 节点 0: 静态 TF 广播 (缝合底盘与相机的坐标系) ----
    # 参数解释: x, y, z(抬高3.5厘米=总高度15.5cm), yaw, pitch, roll, parent_frame, child_frame
    tf_camera_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=['0.0', '0.0', '0.035', '0.0', '0.0', '0.0', 'camera_link0', 'camera_link']
    )

    # ---- 节点 1: image_conversion ----
    image_conversion_node = Node(
        package='image_to_llm',
        executable='image_conversion',
        name='image_conversion_node',
        output='screen',
        parameters=[{
            'pixel_path_topic': '/llm_pixels',
            'camera_info_topic': '/camera/depth/camera_info',
            'depth_image_topic': '/camera/depth/image_raw',
            'odom_topic': '/odom',
            'path_topic': '/path',
            'depth_search_radius': 5,        # 深度采样窗口半径（像素）
            'min_valid_depth': 0.1,          # 最小有效深度（米）
            'max_valid_depth': 5.0,          # 最大有效深度（米）
            'ground_plane_fallback': True,    # 深度无效时使用地面平面假设
            'camera_height': 0.155,           # 核心：使用真实的卡尺测量高度 155mm
            'camera_pitch': 0.0,             # 相机俯仰角（弧度，正值=向下）
            'task_id': LaunchConfiguration('task_id'),
        }],
    )

    # ---- 节点 2: track_path ----
    track_path_node = Node(
        package='my_robot_planner',
        executable='track_path',
        name='track_path',
        output='screen',
        parameters=[{
            'depth_image_topic': '/camera/depth/image_raw',
            'obstacle_distance': 0.20,
            'obstacle_check_enabled': True,
            'obstacle_fov_deg': 60.0,
            'obstacle_roi_top_ratio': 0.3,    # 障碍物检测 ROI 上边界（图像高度比例）
            'obstacle_roi_bottom_ratio': 0.8,  # 障碍物检测 ROI 下边界
            'min_valid_depth': 0.1,
            'max_valid_depth': 5.0,
            'arrival_threshold': 0.15,
            'lookahead_dist': 0.3
        }],
    )

    return LaunchDescription([
        task_id_arg,
        tf_camera_node,
        image_conversion_node,
        track_path_node,
    ])
