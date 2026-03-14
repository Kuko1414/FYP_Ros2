import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import math

class TrackPath(Node):
    def __init__(self):
        super().__init__('track_path')
        self.get_logger().info("Track Path node started")
        
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)
        
        self.sub_mocap = self.create_subscription(PoseStamped, '/vrpn_mocap/rm_0_Test/pose', self.pose_callback, 10)
        self.sub_gps = self.create_subscription(PoseStamped, '/agent0/gps', self.pose_callback, 10)
        
        self.cmd_pub1 = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_pub2 = self.create_publisher(Twist, '/agent0/cmd_vel', 10)
        
        self.desired_path = None
        self.current_target_idx = 0
        self.current_pose = None
        
        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz

    def path_callback(self, msg):
        self.desired_path = msg.poses
        self.current_target_idx = 0
        self.get_logger().info("Received new path.")

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.desired_path is None or self.current_pose is None:
            return
            
        if self.current_target_idx >= len(self.desired_path):
            self.stop_robot()
            return
            
        target = self.desired_path[self.current_target_idx].pose.position
        curr = self.current_pose.pose.position
        
        dx = target.x - curr.x
        dy = target.y - curr.y
        dist = math.hypot(dx, dy)
        
        if dist < 0.2:
            self.current_target_idx += 1
            return
            
        # Simplistic proportional control
        cmd = Twist()
        base_speed = 0.5
        cmd.linear.x = base_speed
        
        # Calculate angle to target
        # Assuming robot orientation is not provided by standard GPS point, use simplistic steering or need quaternion math
        # Let's just output linear twist towards the target for holonomic base or differential drive assuming small corrections
        
        # Without orientation, just simple P controller on twist if holonomic
        cmd.linear.x = min(dist, 1.0)
        cmd.angular.z = dx * 0.5 # rudimentary
        
        self.cmd_pub1.publish(cmd)
        self.cmd_pub2.publish(cmd)
        
    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub1.publish(cmd)
        self.cmd_pub2.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TrackPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
