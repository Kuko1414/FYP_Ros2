"""
启动 Gemini API 节点（单独终端运行，方便查看 LLM 交互日志）：
  - image_to_llm_node  — 发送图像给 Gemini，接收像素路径点

用法（终端 1）:
  ros2 launch image_to_llm llm_node_launch.py
  ros2 launch image_to_llm llm_node_launch.py skill_name:=default
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    skill_name_arg = DeclareLaunchArgument(
        'skill_name',
        default_value='default',
        description='要加载的 Skill 名称（对应 skills/ 目录下的 YAML 文件名，不含后缀）'
    )

    image_to_llm_node = Node(
        package='image_to_llm',
        executable='image_to_llm_node',
        name='image_to_llm_node',
        output='screen',
        parameters=[{
            'rgb_topic': '/camera/color/image_raw',
            'pixel_path_topic': '/llm_pixels',
            'env_path': 'src/image_to_llm/llm_config.env',
            'skill_name': LaunchConfiguration('skill_name'),
        }],
    )

    return LaunchDescription([
        skill_name_arg,
        image_to_llm_node,
    ])
