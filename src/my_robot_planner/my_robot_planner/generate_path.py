import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math
import threading

class GeneratePath(Node):
    def __init__(self):
        super().__init__('generate_path')
        self.get_logger().info("Generate Path node started")
        
        self.path_pub = self.create_publisher(Path, '/path', 10)
        
        self.sub_mocap = self.create_subscription(PoseStamped, '/vrpn_mocap/rm_0_Test/pose', self.pose_callback, 10)
        self.sub_gps = self.create_subscription(PoseStamped, '/agent0/gps', self.pose_callback, 10)
        
        self.start_pose = None
        self.got_start = False
        
        # Run input thread
        self.input_thread = threading.Thread(target=self.wait_for_input)
        self.input_thread.start()

    def pose_callback(self, msg):
        if not self.got_start:
            self.start_pose = msg
            self.got_start = True
            self.get_logger().info("Received start configuration.")

    def wait_for_input(self):
        while rclpy.ok():
            try:
                user_input = input("Enter target X Y and Shape (e.g., 1.0 3.0 semicircle/straight/s_shape/polynomial): ")
                parts = user_input.split()
                if len(parts) >= 3:
                    target_x = float(parts[0])
                    target_y = float(parts[1])
                    shape = parts[2]
                    
                    if not self.got_start:
                        self.get_logger().warn("Start position not received yet! Please wait for localization or move the robot.")
                        continue
                    else:
                        start_x = self.start_pose.pose.position.x
                        start_y = self.start_pose.pose.position.y
                        
                    self.generate_and_publish_path(start_x, start_y, target_x, target_y, shape)
                else:
                    self.get_logger().warn("Invalid input format")
            except Exception as e:
                self.get_logger().error(f"Input error: {e}")

    def generate_and_publish_path(self, sx, sy, tx, ty, shape):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        
        points = 50
        
        # Simple generation
        for i in range(points + 1):
            t = i / float(points)
            p = PoseStamped()
            p.header = path.header
            
            if shape.lower() == 'straight':
                p.pose.position.x = sx + t * (tx - sx)
                p.pose.position.y = sy + t * (ty - sy)
            elif shape.lower() == 'semicircle':
                # Simplified semicircle above the line connecting start and target
                cx = (sx + tx) / 2
                cy = (sy + ty) / 2
                r = math.hypot(tx - sx, ty - sy) / 2
                angle = math.pi - t * math.pi
                dx = math.cos(angle) * r
                dy = math.sin(angle) * r
                
                # Align the semicircle to the direction
                theta = math.atan2(sy - ty, sx - tx)
                p.pose.position.x = cx + math.cos(theta) * dx - math.sin(theta) * dy
                p.pose.position.y = cy + math.sin(theta) * dx + math.cos(theta) * dy
            elif shape.lower() == 's_shape':
                # S-curve using a sine wave scaled by the distance
                dist = math.hypot(tx - sx, ty - sy)
                amplitude = dist / 4.0
                theta = math.atan2(ty - sy, tx - sx)
                offset = amplitude * math.sin(2 * math.pi * t)
                
                # Base line point
                bx = sx + t * (tx - sx)
                by = sy + t * (ty - sy)
                
                # Orthogonal offset
                p.pose.position.x = bx - math.sin(theta) * offset
                p.pose.position.y = by + math.cos(theta) * offset
            elif shape.lower() == 'polynomial':
                # Quadratic Bezier curve (polynomial curve)
                # Control point is deflected from the center orthogonally
                theta = math.atan2(ty - sy, tx - sx)
                dist = math.hypot(tx - sx, ty - sy)
                offset = dist / 2.0
                cx = (sx + tx) / 2 - math.sin(theta) * offset
                cy = (sy + ty) / 2 + math.cos(theta) * offset
                
                # B(t) = (1-t)^2 * P0 + 2(1-t)t * C + t^2 * P1
                p.pose.position.x = ((1 - t) ** 2) * sx + 2 * (1 - t) * t * cx + (t ** 2) * tx
                p.pose.position.y = ((1 - t) ** 2) * sy + 2 * (1 - t) * t * cy + (t ** 2) * ty
            else:
                self.get_logger().warn(f"Unknown shape '{shape}'. Defaulting to straight.")
                p.pose.position.x = sx + t * (tx - sx)
                p.pose.position.y = sy + t * (ty - sy)
                
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            path.poses.append(p)
            
        self.path_pub.publish(path)
        self.get_logger().info(f"Published path with {len(path.poses)} points. Shape: {shape}")

def main(args=None):
    rclpy.init(args=args)
    node = GeneratePath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
