import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from my_robot_msgs.srv import GeneratePath as GeneratePathSrv
import math
import threading

class GeneratePath(Node):
    def __init__(self):
        super().__init__('generate_path')
        self.get_logger().info("Generate Path node started")
        
        # Use ReentrantCallbackGroup so service callback can coexist with subscriptions
        self.cb_group = ReentrantCallbackGroup()
        
        self.path_pub = self.create_publisher(Path, '/path', 10)
        
        # Persistent subscriptions for position (always listening, store latest)
        self.latest_position = None
        self._pos_lock = threading.Lock()
        
        self.sub_mocap = self.create_subscription(
            PoseStamped, '/vrpn_mocap/rm_0_Test/pose', self._pose_callback, 10,
            callback_group=self.cb_group)
        self.sub_gps = self.create_subscription(
            PointStamped, '/agent0/gps', self._gps_callback, 10,
            callback_group=self.cb_group)
        
        # Service server for path generation requests
        self.srv = self.create_service(
            GeneratePathSrv, '/generate_path', self.service_callback,
            callback_group=self.cb_group)
        
        # Run input thread for manual target input
        self.input_thread = threading.Thread(target=self.wait_for_input, daemon=True)
        self.input_thread.start()

    def _pose_callback(self, msg):
        with self._pos_lock:
            self.latest_position = (msg.pose.position.x, msg.pose.position.y)

    def _gps_callback(self, msg):
        with self._pos_lock:
            self.latest_position = (msg.point.x, msg.point.y)

    def get_current_position(self):
        """Get the latest known position. Returns (x, y) or None."""
        with self._pos_lock:
            if self.latest_position is not None:
                self.get_logger().info(f"Current position: ({self.latest_position[0]:.3f}, {self.latest_position[1]:.3f})")
                return self.latest_position
        
        self.get_logger().info("Waiting for position data...")
        for _ in range(50):
            import time
            time.sleep(0.1)
            with self._pos_lock:
                if self.latest_position is not None:
                    self.get_logger().info(f"Got position: ({self.latest_position[0]:.3f}, {self.latest_position[1]:.3f})")
                    return self.latest_position
        
        self.get_logger().warn("Timeout waiting for position!")
        return None

    def service_callback(self, request, response):
        """Handle path generation service request."""
        self.get_logger().info(f"Service request: target=({request.target_x:.2f}, {request.target_y:.2f}), shape={request.shape}")
        
        start = self.get_current_position()
        if start is None:
            self.get_logger().error("Failed to get current position for service request!")
            response.success = False
            return response
        
        sx, sy = start
        path = self.generate_path(sx, sy, request.target_x, request.target_y, request.shape)
        
        self.path_pub.publish(path)
        
        response.path = path
        response.success = True
        self.get_logger().info(f"Service response: path with {len(path.poses)} points, "
                               f"start=({sx:.3f},{sy:.3f}), end=({request.target_x:.3f},{request.target_y:.3f})")
        return response

    def wait_for_input(self):
        """Thread for manual terminal input."""
        while rclpy.ok():
            try:
                user_input = input("Enter target X Y and Shape (e.g., 1.0 3.0 semicircle/straight/s_shape/polynomial): ")
                parts = user_input.split()
                if len(parts) >= 3:
                    target_x = float(parts[0])
                    target_y = float(parts[1])
                    shape = parts[2]
                    
                    start = self.get_current_position()
                    if start is None:
                        self.get_logger().warn("Failed to get current position! Please ensure localization is running.")
                        continue
                    
                    sx, sy = start
                    path = self.generate_path(sx, sy, target_x, target_y, shape)
                    self.path_pub.publish(path)
                    self.get_logger().info(f"Published path with {len(path.poses)} points. Shape: {shape}")
                else:
                    self.get_logger().warn("Invalid input format. Use: X Y shape")
            except Exception as e:
                self.get_logger().error(f"Input error: {e}")

    def generate_path(self, sx, sy, tx, ty, shape):
        """Generate a Path message from start (sx,sy) to target (tx,ty) with the given shape.
        
        All shapes guarantee: t=0 -> (sx,sy), t=1 -> (tx,ty).
        """
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        
        num_points = 50
        
        for i in range(num_points + 1):
            t = i / float(num_points)
            p = PoseStamped()
            p.header = path.header
            
            if shape.lower() == 'straight':
                p.pose.position.x = sx + t * (tx - sx)
                p.pose.position.y = sy + t * (ty - sy)
                
            elif shape.lower() == 'semicircle':
                # Semicircle from start to target, bulging to the left side of travel direction
                # Center of the chord connecting start and target
                cx = (sx + tx) / 2.0
                cy = (sy + ty) / 2.0
                r = math.hypot(tx - sx, ty - sy) / 2.0
                
                # Direction angle from start to target
                theta = math.atan2(ty - sy, tx - sx)
                
                # Parametric angle: from pi to 0 (semicircle above the chord in local frame)
                # In local frame: x_local = r*cos(angle), y_local = r*sin(angle)
                # angle goes from pi (start) to 0 (end)
                angle = math.pi * (1.0 - t)
                
                # Local coordinates (centered at midpoint, x-axis along start->target)
                local_x = r * math.cos(angle)
                local_y = r * math.sin(angle)
                
                # Rotate to world frame and translate to center
                p.pose.position.x = cx + math.cos(theta) * local_x - math.sin(theta) * local_y
                p.pose.position.y = cy + math.sin(theta) * local_x + math.cos(theta) * local_y
                
            elif shape.lower() == 's_shape':
                # S-curve using sine wave offset perpendicular to the straight line
                dist = math.hypot(tx - sx, ty - sy)
                amplitude = dist / 4.0
                theta = math.atan2(ty - sy, tx - sx)
                offset = amplitude * math.sin(2.0 * math.pi * t)
                
                # Base line point
                bx = sx + t * (tx - sx)
                by = sy + t * (ty - sy)
                
                # Perpendicular offset (left of travel direction)
                p.pose.position.x = bx - math.sin(theta) * offset
                p.pose.position.y = by + math.cos(theta) * offset
                
            elif shape.lower() == 'polynomial':
                # Quadratic Bezier curve with control point offset perpendicular to midpoint
                theta = math.atan2(ty - sy, tx - sx)
                dist = math.hypot(tx - sx, ty - sy)
                offset = dist / 3.0
                ctrl_x = (sx + tx) / 2.0 - math.sin(theta) * offset
                ctrl_y = (sy + ty) / 2.0 + math.cos(theta) * offset
                
                # B(t) = (1-t)^2 * P0 + 2(1-t)t * C + t^2 * P1
                p.pose.position.x = ((1 - t) ** 2) * sx + 2 * (1 - t) * t * ctrl_x + (t ** 2) * tx
                p.pose.position.y = ((1 - t) ** 2) * sy + 2 * (1 - t) * t * ctrl_y + (t ** 2) * ty
            else:
                self.get_logger().warn(f"Unknown shape '{shape}'. Defaulting to straight.")
                p.pose.position.x = sx + t * (tx - sx)
                p.pose.position.y = sy + t * (ty - sy)
                
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        
        # Log first and last point for verification
        if len(path.poses) >= 2:
            first = path.poses[0].pose.position
            last = path.poses[-1].pose.position
            self.get_logger().info(f"Path: first=({first.x:.3f},{first.y:.3f}), last=({last.x:.3f},{last.y:.3f})")
        
        return path

def main(args=None):
    rclpy.init(args=args)
    node = GeneratePath()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
