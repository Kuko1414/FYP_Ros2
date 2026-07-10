"""
启动 Agent 模式（Gemini 语义导航 + APF 雷达避障）：
  1. agent_node     — Agent 主节点（Gemini Function Calling 多轮交互）
  2. track_path     — APF 控制器（目标点引力 + 雷达斥力，自动避障导航）
  3. 静态 TF 广播    — 底盘与相机坐标系

工作流：
  - Gemini 看图判断目标方向 → publish_goal_relative(direction, distance) 发布单个目标点
  - track_path 订阅 /goal_point + /scan_raw，用 APF 算法自动避障导航
  - 到达目标点后 track_path 调用 /trigger_llm_plan 触发 Gemini 下一轮
  - Gemini 看到目标就在眼前时调用 finish_task() 停车结束

用法:
  ros2 launch image_to_llm agent_launch.py
  ros2 launch image_to_llm agent_launch.py skill_name:=agent_default
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    skill_name_arg = DeclareLaunchArgument(
        'skill_name',
        default_value='agent_default',
        description='Agent Skill 名称（对应 skills/ 目录下的 YAML 文件名）'
    )

    max_turns_arg = DeclareLaunchArgument(
        'max_turns',
        default_value='8',
        description='Agent 最大交互轮数（防止 API 费用失控）'
    )

    # ---- 节点 0: 静态 TF 广播（底盘与相机坐标系）----
    tf_camera_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=['0.0', '0.0', '0.035', '0.0', '0.0', '0.0',
                   'camera_link0', 'camera_link']
    )

    # ---- 节点 1: Agent 节点 ----
    agent_node = Node(
        package='image_to_llm',
        executable='agent_node',
        name='agent_node',
        output='screen',
        parameters=[{
            'env_path': 'src/image_to_llm/llm_config.env',
            'skill_name': LaunchConfiguration('skill_name'),
            'max_turns': LaunchConfiguration('max_turns'),
            'rgb_topic': '/camera/color/image_raw',
            'depth_image_topic': '/camera/depth/image_raw',
            'odom_topic': '/odom',
        }],
    )

    # ---- 节点 2: track_path（目标追踪 + 斥力场避障模式）----
    track_path_node = Node(
        package='my_robot_planner',
        executable='track_path',
        name='track_path',
        output='screen',
        parameters=[{
            # 紧急停车
            'emergency_dist': 0.15,       # 紧急停车距离（米）
            'emergency_fov_deg': 60.0,    # 紧急停车检测 FOV（度）
            # 斥力场（270° 防撞）
            'repulse_fov_deg': 270.0,     # 斥力场检测 FOV（度）
            'repulse_dist': 0.7,          # 斥力场外缘生效距离（米），非线性：边缘弱/近处强
            'repulse_gain': 0.9,          # 斥力场增益
            # 运动限制
            'max_linear_vel': 0.3,
            'max_angular_vel': 1.0,
            'min_linear_vel': 0.03,
            # 目标到达
            'arrival_threshold': 0.3,
            # 雷达话题
            'scan_topic': '/scan_raw',
        }],
    )

    return LaunchDescription([
        skill_name_arg,
        max_turns_arg,
        tf_camera_node,
        agent_node,
        track_path_node,
    ])
