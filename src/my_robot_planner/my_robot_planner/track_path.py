import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from my_robot_msgs.srv import GeneratePath as GeneratePathSrv
import math
import time

class PIDController:
    """Simple PID controller with anti-windup."""
    def __init__(self, kp, ki, kd, output_min=-float('inf'), output_max=float('inf'), integral_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max
        
        self.prev_error = 0.0
        self.integral = 0.0
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
    
    def compute(self, error, dt):
        if dt <= 0:
            return 0.0
        
        # Proportional
        p_term = self.kp * error
        
        # Integral with anti-windup
        self.integral += error * dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        self.prev_error = error
        
        # Total output with clamping
        output = p_term + i_term + d_term
        return max(self.output_min, min(self.output_max, output))


class TrackPath(Node):
    def __init__(self):
        super().__init__('track_path')
        self.get_logger().info("Track Path node started")
        
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)
        
        self.sub_mocap = self.create_subscription(PoseStamped, '/vrpn_mocap/rm_0_Test/pose', self.pose_callback, 10)
        self.sub_gps = self.create_subscription(PointStamped, '/agent0/gps', self.gps_callback, 10)
        
        # Subscribe to IMU for orientation (yaw)
        self.sub_imu = self.create_subscription(Imu, '/agent0/imu', self.imu_callback, 10)
        
        self.cmd_pub1 = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_pub2 = self.create_publisher(Twist, '/agent0/cmd_vel', 10)
        
        # Service client for replan requests to Generate_Path
        self.replan_client = self.create_client(GeneratePathSrv, '/generate_path')
        
        self.desired_path = None
        self.current_target_idx = 0
        self.current_pose = None
        self.current_yaw = None
        
        # Store the current target destination and shape for replan
        self.target_x = None
        self.target_y = None
        self.target_shape = 'straight'  # default replan shape
        
        # Replan state
        self.replan_in_progress = False
        self.deviation_threshold = 0.5  # meters, triggers replan
        
        # Lookahead distance for target point selection (Pure Pursuit style)
        self.lookahead_dist = 0.3
        
        # PID controllers
        # Angular PID: corrects heading error
        self.angular_pid = PIDController(
            kp=2.0, ki=0.1, kd=0.3,
            output_min=-2.0, output_max=2.0, integral_max=1.0
        )
        # Linear PID: controls forward speed based on distance to target
        self.linear_pid = PIDController(
            kp=0.5, ki=0.05, kd=0.1,
            output_min=0.0, output_max=1.0, integral_max=1.0
        )
        
        # Timing for PID dt calculation
        self.last_control_time = None
        
        # Path completion flag
        self.path_completed = False
        
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz

    def path_callback(self, msg):
        self.desired_path = msg.poses
        self.current_target_idx = 0
        self.replan_in_progress = False
        self.path_completed = False
        
        # Reset PID controllers for new path
        self.angular_pid.reset()
        self.linear_pid.reset()
        self.last_control_time = None
        
        # Extract final target from path for potential replan
        if len(msg.poses) > 0:
            last_pose = msg.poses[-1].pose.position
            self.target_x = last_pose.x
            self.target_y = last_pose.y
        
        self.get_logger().info(f"Received new path with {len(self.desired_path)} points.")

    def pose_callback(self, msg):
        self.current_pose = msg
        # Extract yaw from mocap orientation quaternion
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def gps_callback(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose.position = msg.point
        pose_msg.pose.orientation.w = 1.0
        self.current_pose = pose_msg

    def imu_callback(self, msg):
        """Extract yaw from IMU orientation quaternion."""
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def compute_cross_track_error(self):
        """Compute the perpendicular (cross-track) distance from current position to the nearest path segment.
        
        Returns (cross_track_error, nearest_segment_idx) or (None, None).
        """
        if self.desired_path is None or self.current_pose is None or len(self.desired_path) < 2:
            return None, None
        
        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y
        
        min_dist = float('inf')
        nearest_idx = self.current_target_idx
        
        # Search segments around current target index
        search_start = max(0, self.current_target_idx - 2)
        search_end = min(len(self.desired_path) - 1, self.current_target_idx + 10)
        
        for i in range(search_start, search_end):
            # Segment from path[i] to path[i+1]
            ax = self.desired_path[i].pose.position.x
            ay = self.desired_path[i].pose.position.y
            bx = self.desired_path[i + 1].pose.position.x
            by = self.desired_path[i + 1].pose.position.y
            
            # Project current point onto segment
            abx = bx - ax
            aby = by - ay
            apx = curr_x - ax
            apy = curr_y - ay
            
            ab_sq = abx * abx + aby * aby
            if ab_sq < 1e-10:
                # Degenerate segment
                d = math.hypot(apx, apy)
            else:
                t_proj = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_sq))
                proj_x = ax + t_proj * abx
                proj_y = ay + t_proj * aby
                d = math.hypot(curr_x - proj_x, curr_y - proj_y)
            
            if d < min_dist:
                min_dist = d
                nearest_idx = i
        
        return min_dist, nearest_idx

    def find_lookahead_target(self):
        """Find the target point on the path using lookahead distance (Pure Pursuit style).
        
        Returns the index of the target point.
        """
        if self.desired_path is None or self.current_pose is None:
            return self.current_target_idx
        
        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y
        
        # Start searching from current target index
        target_idx = self.current_target_idx
        
        for i in range(self.current_target_idx, len(self.desired_path)):
            pt = self.desired_path[i].pose.position
            d = math.hypot(pt.x - curr_x, pt.y - curr_y)
            
            if d >= self.lookahead_dist:
                target_idx = i
                break
            else:
                # This point is within lookahead, advance past it
                target_idx = i + 1
        
        # Clamp to valid range
        target_idx = min(target_idx, len(self.desired_path) - 1)
        return target_idx

    def request_replan(self):
        """Send a replan request to Generate_Path via service."""
        if self.replan_in_progress:
            return
        
        if self.target_x is None or self.target_y is None:
            self.get_logger().warn("No target destination stored, cannot replan.")
            return
        
        if not self.replan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Generate_Path service not available, cannot replan.")
            return
        
        self.replan_in_progress = True
        self.get_logger().info(f"Requesting replan to ({self.target_x:.2f}, {self.target_y:.2f}), shape={self.target_shape}")
        
        request = GeneratePathSrv.Request()
        request.target_x = self.target_x
        request.target_y = self.target_y
        request.shape = self.target_shape
        
        future = self.replan_client.call_async(request)
        future.add_done_callback(self.replan_response_callback)

    def replan_response_callback(self, future):
        """Handle the replan service response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Replan successful, new path received.")
            else:
                self.get_logger().error("Replan failed!")
                self.replan_in_progress = False
        except Exception as e:
            self.get_logger().error(f"Replan service call failed: {e}")
            self.replan_in_progress = False

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self):
        if self.desired_path is None or self.current_pose is None or self.current_yaw is None:
            return
        
        if self.path_completed:
            return
        
        # If replan is in progress, stop and wait
        if self.replan_in_progress:
            self.stop_robot()
            return
        
        # Calculate dt for PID
        now = time.monotonic()
        if self.last_control_time is None:
            dt = 0.1  # first iteration, assume 10Hz
        else:
            dt = now - self.last_control_time
        self.last_control_time = now
        
        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y
        
        # Check if we've reached the final target
        final_pt = self.desired_path[-1].pose.position
        dist_to_final = math.hypot(final_pt.x - curr_x, final_pt.y - curr_y)
        if dist_to_final < 0.15:
            self.stop_robot()
            self.path_completed = True
            self.get_logger().info(f"Path tracking completed! Final distance: {dist_to_final:.3f}m")
            self.desired_path = None
            return
        
        # Compute cross-track error (perpendicular distance to path)
        cross_track_error, nearest_seg_idx = self.compute_cross_track_error()
        
        if cross_track_error is not None and cross_track_error > self.deviation_threshold:
            # Deviation too large, stop and request replan
            self.get_logger().warn(f"Cross-track error {cross_track_error:.3f}m exceeds threshold {self.deviation_threshold}m. Requesting replan.")
            self.stop_robot()
            self.request_replan()
            return
        
        # Update current_target_idx based on nearest segment
        if nearest_seg_idx is not None and nearest_seg_idx > self.current_target_idx:
            self.current_target_idx = nearest_seg_idx
        
        # Find lookahead target point
        target_idx = self.find_lookahead_target()
        target = self.desired_path[target_idx].pose.position
        
        dx = target.x - curr_x
        dy = target.y - curr_y
        dist = math.hypot(dx, dy)
        
        # Calculate heading error
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.current_yaw)
        
        # Continuous PID control
        angular_cmd = self.angular_pid.compute(angle_error, dt)
        
        # Linear speed: reduce when angle error is large, use PID on distance
        angle_factor = max(0.0, 1.0 - abs(angle_error) / math.pi)  # 0~1, reduces speed when turning
        linear_cmd = self.linear_pid.compute(dist, dt) * angle_factor
        
        # Minimum speed to keep moving (avoid stalling)
        if dist > 0.1 and linear_cmd < 0.03:
            linear_cmd = 0.03
        
        cmd = Twist()
        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd
        
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
