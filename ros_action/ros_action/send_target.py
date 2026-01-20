#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node


class TargetPublisher(Node):
    """Publish target points to /target_point topic."""

    def __init__(self) -> None:
        super().__init__('target_publisher')
        self.publisher = self.create_publisher(Point, '/target_point', 10)
        self.get_logger().info('Target publisher initialized. Use send_target(x, y) to publish.')

    def send_target(self, x: float, y: float) -> None:
        """Publish a target point."""
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f'Published target point: ({x:.2f}, {y:.2f})')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetPublisher()

    # Example usage: send a sequence of target points
    print('发送目标点示例:')
    print('输入格式: x y (用空格分隔)')
    print('输入 "q" 退出\n')

    try:
        while rclpy.ok():
            user_input = input('> ').strip()
            if user_input.lower() == 'q':
                break
            try:
                x, y = map(float, user_input.split())
                node.send_target(x, y)
            except ValueError:
                print('无效输入。请输入 "x y" 格式 (例如: 1.0 2.5)')
    except KeyboardInterrupt:
        print('\n已中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
