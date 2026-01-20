import math
from collections import deque
from typing import Deque, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


Point2D = Tuple[float, float]


class CircularTrajectoryNode(Node):
	"""Follow a semicircular path from current position to a target point."""

	def __init__(self) -> None:
		super().__init__('circular_trajectory_node')
		self.target_sub = self.create_subscription(Point, '/target_point', self.target_callback, 10)
		
		# Configure QoS for VRPN mocap pose subscriber 
		qos_profile = QoSProfile(
			reliability=ReliabilityPolicy.BEST_EFFORT,
			history=HistoryPolicy.KEEP_LAST,
			depth=10
		)
		self.pose_sub = self.create_subscription(
			PoseStamped, '/vrpn_mocap/rm_0_Test/pose', self.pose_callback, qos_profile
		)
		self.cmd_pub = self.create_publisher(Twist, '/rm_0/cmd_vel', 10)

		self.control_timer = self.create_timer(0.05, self.control_loop)
		self.deviation_timer = self.create_timer(0.5, self.deviation_check)

		self.current_position: Optional[Point2D] = None
		self.current_yaw: Optional[float] = None
		self.target_point: Optional[Point2D] = None
		self.waypoints: Deque[Point2D] = deque()

		# Original circular path parameters (fixed)
		self.initial_start_point: Optional[Point2D] = None
		self.circle_center: Optional[Point2D] = None
		self.circle_radius: Optional[float] = None
		self.circle_direction: int = 1  # 1 for counter-clockwise, -1 for clockwise

		self.distance_tolerance = 0.2
		self.deviation_threshold = 0.8  # Increased for smoother replanning
		self.max_speed = 0.5
		self.max_angular_speed = 1.8
		self.max_curvature = 2.0
		self.path_planned = False
		
		# Smooth tracking control parameters
		self.lookahead_distance = 0.5  # meters, for Pure Pursuit-like control
		self.lateral_gain = 2.0  # gain for lateral error correction
		self.speed_reduction_gain = 0.3  # how much to slow down when correcting
		self.min_speed = 0.15  # minimum forward speed
		
		# Lattice planner parameters for return path sampling
		self.return_time_samples = [1.5, 2.0, 2.5, 3.0, 3.5]  # seconds
		self.lateral_offset_samples = 5  # number of tangent points to sample
		
		# Cost weights for path evaluation
		self.k_jerk = 0.1
		self.k_time = 0.2
		self.k_lateral_deviation = 1.5
		self.k_curvature = 0.5

	def target_callback(self, msg: Point) -> None:
		self.target_point = (msg.x, msg.y)
		self.get_logger().info(f'Received target point ({msg.x:.2f}, {msg.y:.2f})')
		if self.current_position and self.current_yaw is not None:
			# Record initial start point when first planning
			self.initial_start_point = self.current_position
			self.plan_path(self.current_position, self.target_point, is_initial=True)
		else:
			self.path_planned = False

	def pose_callback(self, msg: PoseStamped) -> None:
		self.current_position = (msg.pose.position.x, msg.pose.position.y)
		self.current_yaw = self._yaw_from_quaternion(
			msg.pose.orientation.x,
			msg.pose.orientation.y,
			msg.pose.orientation.z,
			msg.pose.orientation.w,
		)
		# Trigger path planning if target exists but path not yet planned
		if self.target_point and not self.path_planned:
			self.initial_start_point = self.current_position
			self.plan_path(self.current_position, self.target_point, is_initial=True)

	def plan_path(self, start: Point2D, goal: Point2D, is_initial: bool = False) -> None:
		"""Generate or replan path to the semicircular trajectory."""
		if self._distance(start, goal) < 0.2:
			self.get_logger().warn('Start and goal are too close; stopping path generation')
			self.waypoints.clear()
			self.path_planned = False
			return

		if is_initial:
			# Initial planning: create the reference circle
			self._setup_reference_circle(start, goal)
			path = self._generate_circular_path(start, goal, samples=80)
			self.get_logger().info(f'Initial planning: {len(path)} waypoints on reference circle')
		else:
			# Replanning: use Lattice sampling to find optimal return path
			candidates = self._sample_return_paths(start)
			if not candidates:
				self.get_logger().warn('No valid return paths found')
				return
			path = self._select_optimal_path(candidates)
			self.get_logger().info(f'Replanning: selected path with cost {path["cost"]:.2f}, {len(path["waypoints"])} waypoints')
		
		self.waypoints = deque(path if is_initial else path['waypoints'])
		self.path_planned = True

	def control_loop(self) -> None:
		if not (self.current_position and self.current_yaw is not None):
			return

		if not self.waypoints or not self.circle_center:
			self._publish_stop()
			return

		# Calculate lateral error from the reference circle
		dist_to_center = self._distance(self.current_position, self.circle_center)
		lateral_error = dist_to_center - self.circle_radius
		
		# Find lookahead point on the path
		lookahead_point = self._find_lookahead_point()
		if not lookahead_point:
			# Near end of path, use last waypoint
			if len(self.waypoints) > 0:
				lookahead_point = self.waypoints[-1]
			else:
				self._publish_stop()
				return
		
		# Calculate desired heading to lookahead point
		dx = lookahead_point[0] - self.current_position[0]
		dy = lookahead_point[1] - self.current_position[1]
		target_heading = math.atan2(dy, dx)
		heading_error = self._angle_wrap(target_heading - self.current_yaw)
		
		# Calculate correction angle based on lateral error
		# If we're outside the circle, turn inward; if inside, turn outward
		if abs(lateral_error) > 0.05:  # 5cm threshold
			# Calculate angle to circle center
			to_center_x = self.circle_center[0] - self.current_position[0]
			to_center_y = self.circle_center[1] - self.current_position[1]
			angle_to_center = math.atan2(to_center_y, to_center_x)
			
			# Add lateral correction bias
			lateral_correction = self.lateral_gain * lateral_error
			# Smooth correction: blend with heading error
			heading_error = heading_error + math.copysign(min(abs(lateral_correction), 0.3), lateral_error)
		
		# Smooth angular velocity control
		angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, 1.2 * heading_error))
		
		# Adaptive speed based on heading error and lateral error
		# Slow down when making corrections, but don't stop
		error_magnitude = abs(heading_error) + abs(lateral_error) * self.speed_reduction_gain
		speed_factor = 1.0 / (1.0 + error_magnitude)
		speed_factor = max(speed_factor, self.min_speed / self.max_speed)  # Ensure minimum speed
		
		linear_x = self.max_speed * speed_factor
		
		# Gradual speed reduction for sharp turns instead of full stop
		if abs(heading_error) > 1.0:
			linear_x *= max(0.3, math.cos(heading_error))  # Keep at least 30% speed
		else:
			linear_x *= max(0.5, math.cos(heading_error))  # Keep at least 50% speed
		
		# Remove waypoints that we've passed
		self._remove_passed_waypoints()
		
		twist = Twist()
		twist.linear.x = linear_x
		twist.angular.z = angular_z
		self.cmd_pub.publish(twist)

	def _find_lookahead_point(self) -> Optional[Point2D]:
		"""Find a lookahead point on the path for smooth tracking."""
		if not self.waypoints:
			return None
		
		# Find the first waypoint that is at least lookahead_distance away
		for wp in self.waypoints:
			dist = self._distance(self.current_position, wp)
			if dist >= self.lookahead_distance:
				return wp
		
		# If all waypoints are closer, return the farthest one
		return self.waypoints[-1] if self.waypoints else None
	
	def _remove_passed_waypoints(self) -> None:
		"""Remove waypoints that the robot has passed."""
		while self.waypoints:
			wp = self.waypoints[0]
			dist = self._distance(self.current_position, wp)
			
			# Check if waypoint is behind us
			dx = wp[0] - self.current_position[0]
			dy = wp[1] - self.current_position[1]
			angle_to_wp = math.atan2(dy, dx)
			relative_angle = abs(self._angle_wrap(angle_to_wp - self.current_yaw))
			
			# Remove if close enough or if it's behind us
			if dist < self.distance_tolerance or relative_angle > math.pi / 2:
				self.waypoints.popleft()
			else:
				break

	def deviation_check(self) -> None:
		if not (self.current_position and self.target_point and self.current_yaw is not None):
			return

		if not self.waypoints or not self.circle_center:
			return

		# Only replan if deviation is large and we haven't replanned recently
		if not hasattr(self, 'last_replan_time'):
			self.last_replan_time = 0.0
		
		current_time = self.get_clock().now().nanoseconds / 1e9
		time_since_replan = current_time - self.last_replan_time
		
		# Calculate deviation from the original circle
		deviation = abs(self._distance(self.current_position, self.circle_center) - self.circle_radius)
		
		# Only replan if deviation is large AND enough time has passed
		if deviation > self.deviation_threshold and time_since_replan > 3.0:
			self.get_logger().info(
				f'Deviation {deviation:.2f} m from circle exceeds threshold; replanning'
			)
			self.last_replan_time = current_time
			self.plan_path(self.current_position, self.target_point, is_initial=False)

	def _setup_reference_circle(self, start: Point2D, goal: Point2D) -> None:
		"""Setup the reference circle parameters based on initial start and goal."""
		self.circle_center = ((start[0] + goal[0]) / 2.0, (start[1] + goal[1]) / 2.0)
		self.circle_radius = self._distance(start, goal) / 2.0
		
		# Determine circle direction based on current heading
		if self.current_yaw is not None:
			to_goal = math.atan2(goal[1] - start[1], goal[0] - start[0])
			heading_to_goal = self._angle_wrap(to_goal - self.current_yaw)
			self.circle_direction = -1 if abs(heading_to_goal) > math.pi / 2 else 1
		else:
			self.circle_direction = 1

	def _generate_circular_path(self, start: Point2D, goal: Point2D, samples: int = 60) -> List[Point2D]:
		center_x = (start[0] + goal[0]) / 2.0
		center_y = (start[1] + goal[1]) / 2.0
		radius = self._distance(start, goal) / 2.0

		if radius < 1e-6:
			return [start]

		start_angle = math.atan2(start[1] - center_y, start[0] - center_x)
		
		# Choose direction based on current heading to minimize initial turn
		if self.current_yaw is not None:
			# Calculate which direction aligns better with current heading
			to_goal = math.atan2(goal[1] - start[1], goal[0] - start[0])
			heading_to_goal = self._angle_wrap(to_goal - self.current_yaw)
			# If heading error is large, try reversed semicircle
			if abs(heading_to_goal) > math.pi / 2:
				step = -math.pi / max(samples - 1, 1)  # Clockwise
			else:
				step = math.pi / max(samples - 1, 1)  # Counter-clockwise
		else:
			step = math.pi / max(samples - 1, 1)

		path: List[Point2D] = []
		for i in range(samples):
			angle = start_angle + step * i
			x = center_x + radius * math.cos(angle)
			y = center_y + radius * math.sin(angle)
			path.append((x, y))
		return path

	def _sample_return_paths(self, current: Point2D) -> List[dict]:
		"""Sample multiple return paths using Lattice planning approach."""
		if not self.circle_center or not self.circle_radius:
			return []
		
		candidates = []
		
		# Sample different tangent points on the circle
		dx = current[0] - self.circle_center[0]
		dy = current[1] - self.circle_center[1]
		dist_to_center = math.hypot(dx, dy)
		
		if dist_to_center < 1e-6:
			return []
		
		# Base angle (closest point on circle)
		base_angle = math.atan2(dy, dx)
		
		# Sample different approach angles
		for angle_offset in [-0.6, -0.3, 0.0, 0.3, 0.6]:  # radians
			target_angle = base_angle + angle_offset
			target_on_circle = (
				self.circle_center[0] + self.circle_radius * math.cos(target_angle),
				self.circle_center[1] + self.circle_radius * math.sin(target_angle)
			)
			
			# Sample different time durations
			for t_total in self.return_time_samples:
				path_dict = self._generate_single_return_path(
					current, target_on_circle, target_angle, t_total
				)
				
				if path_dict and self._verify_return_path(path_dict):
					cost = self._evaluate_path_cost(path_dict)
					path_dict['cost'] = cost
					candidates.append(path_dict)
		
		return candidates

	def _generate_single_return_path(self, start: Point2D, target: Point2D, 
									 target_angle: float, duration: float) -> dict:
		"""Generate a single return path with quintic polynomial."""
		dt = 0.1  # time step
		num_points = int(duration / dt)
		
		# Initial and final conditions for lateral motion (similar to Lattice)
		l0 = 0.0  # normalized start
		l1 = 1.0  # normalized end
		l0_v = 0.0  # start velocity
		l1_v = 0.0  # end velocity
		l0_a = 0.0  # start acceleration
		l1_a = 0.0  # end acceleration
		
		# Generate path using quintic polynomial coefficients
		# p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
		coeffs = self._quintic_polynomial_coeffs(l0, l0_v, l0_a, l1, l1_v, l1_a, duration)
		
		path_points = []
		path_velocities = []
		path_accelerations = []
		path_jerks = []
		
		for i in range(num_points):
			t = i * dt
			# Evaluate polynomial and its derivatives
			l_t = self._eval_quintic(coeffs, t, 0)  # position
			l_v = self._eval_quintic(coeffs, t, 1)  # velocity
			l_a = self._eval_quintic(coeffs, t, 2)  # acceleration
			l_j = self._eval_quintic(coeffs, t, 3)  # jerk
			
			# Interpolate between start and target
			x = start[0] + (target[0] - start[0]) * l_t
			y = start[1] + (target[1] - start[1]) * l_t
			
			path_points.append((x, y))
			path_velocities.append(l_v / duration)  # scale by duration
			path_accelerations.append(l_a / (duration * duration))
			path_jerks.append(l_j / (duration * duration * duration))
		
		# Continue along circle to goal
		goal_angle = math.atan2(
			self.target_point[1] - self.circle_center[1],
			self.target_point[0] - self.circle_center[0]
		)
		
		angle_diff = self._angle_wrap(goal_angle - target_angle)
		if self.circle_direction < 0:
			if angle_diff > 0:
				angle_diff -= 2 * math.pi
		else:
			if angle_diff < 0:
				angle_diff += 2 * math.pi
		
		circle_points = 30
		angle_step = angle_diff / max(circle_points, 1)
		for i in range(1, circle_points + 1):
			angle = target_angle + angle_step * i
			x = self.circle_center[0] + self.circle_radius * math.cos(angle)
			y = self.circle_center[1] + self.circle_radius * math.sin(angle)
			path_points.append((x, y))
		
		return {
			'waypoints': path_points,
			'jerks': path_jerks,
			'duration': duration,
			'target_angle': target_angle
		}

	def _quintic_polynomial_coeffs(self, x0, v0, a0, x1, v1, a1, T):
		"""Calculate quintic polynomial coefficients."""
		a0 = x0
		a1 = v0
		a2 = a0 / 2.0
		
		T2 = T * T
		T3 = T2 * T
		T4 = T3 * T
		T5 = T4 * T
		
		a3 = (20.0 * x1 - 20.0 * x0 - (8.0 * v1 + 12.0 * v0) * T - (3.0 * a0 - a1) * T2) / (2.0 * T3)
		a4 = (30.0 * x0 - 30.0 * x1 + (14.0 * v1 + 16.0 * v0) * T + (3.0 * a0 - 2.0 * a1) * T2) / (2.0 * T4)
		a5 = (12.0 * x1 - 12.0 * x0 - (6.0 * v1 + 6.0 * v0) * T - (a0 - a1) * T2) / (2.0 * T5)
		
		return [a0, a1, a2, a3, a4, a5]

	def _eval_quintic(self, coeffs, t, derivative=0):
		"""Evaluate quintic polynomial or its derivatives."""
		if derivative == 0:  # position
			return coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5
		elif derivative == 1:  # velocity
			return coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t**2 + 4*coeffs[4]*t**3 + 5*coeffs[5]*t**4
		elif derivative == 2:  # acceleration
			return 2*coeffs[2] + 6*coeffs[3]*t + 12*coeffs[4]*t**2 + 20*coeffs[5]*t**3
		elif derivative == 3:  # jerk
			return 6*coeffs[3] + 24*coeffs[4]*t + 60*coeffs[5]*t**2
		return 0.0

	def _verify_return_path(self, path_dict: dict) -> bool:
		"""Verify path meets physical constraints."""
		waypoints = path_dict['waypoints']
		if len(waypoints) < 2:
			return False
		
		# Check curvature constraint
		for i in range(len(waypoints) - 2):
			p1, p2, p3 = waypoints[i], waypoints[i+1], waypoints[i+2]
			dx1, dy1 = p2[0] - p1[0], p2[1] - p1[1]
			dx2, dy2 = p3[0] - p2[0], p3[1] - p2[1]
			
			yaw1 = math.atan2(dy1, dx1)
			yaw2 = math.atan2(dy2, dx2)
			ds = math.hypot(dx1, dy1)
			
			if ds > 1e-6:
				curv = abs(self._angle_wrap(yaw2 - yaw1)) / ds
				if curv > self.max_curvature:
					return False
		
		return True

	def _evaluate_path_cost(self, path_dict: dict) -> float:
		"""Evaluate path cost using Lattice planner approach."""
		# Jerk cost (smoothness)
		jerk_sum = sum(abs(j) for j in path_dict['jerks'])
		
		# Time cost
		time_cost = path_dict['duration']
		
		# Lateral deviation from optimal circle entry point
		dx = path_dict['waypoints'][0][0] - self.circle_center[0]
		dy = path_dict['waypoints'][0][1] - self.circle_center[1]
		optimal_angle = math.atan2(dy, dx)
		angle_deviation = abs(self._angle_wrap(path_dict['target_angle'] - optimal_angle))
		
		# Curvature cost (penalize sharp turns)
		max_curv = 0.0
		waypoints = path_dict['waypoints']
		for i in range(len(waypoints) - 2):
			p1, p2, p3 = waypoints[i], waypoints[i+1], waypoints[i+2]
			dx1, dy1 = p2[0] - p1[0], p2[1] - p1[1]
			dx2, dy2 = p3[0] - p2[0], p3[1] - p2[1]
			yaw1 = math.atan2(dy1, dx1)
			yaw2 = math.atan2(dy2, dx2)
			ds = math.hypot(dx1, dy1)
			if ds > 1e-6:
				curv = abs(self._angle_wrap(yaw2 - yaw1)) / ds
				max_curv = max(max_curv, curv)
		
		cost = (self.k_jerk * jerk_sum +
				self.k_time * time_cost +
				self.k_lateral_deviation * angle_deviation +
				self.k_curvature * max_curv)
		
		return cost

	def _select_optimal_path(self, candidates: List[dict]) -> dict:
		"""Select the path with minimum cost."""
		return min(candidates, key=lambda p: p['cost'])

	def _generate_return_path(self, current: Point2D, samples: int = 40) -> List[Point2D]:
		"""Generate a smooth path from current position back to the reference circle."""
		if not self.circle_center or not self.circle_radius:
			return [current]
		
		# Find the closest point on the circle (tangent point)
		dx = current[0] - self.circle_center[0]
		dy = current[1] - self.circle_center[1]
		dist_to_center = math.hypot(dx, dy)
		
		if dist_to_center < 1e-6:
			target_on_circle = (self.circle_center[0] + self.circle_radius, self.circle_center[1])
		else:
			# Project current position onto the circle
			scale = self.circle_radius / dist_to_center
			target_on_circle = (
				self.circle_center[0] + dx * scale,
				self.circle_center[1] + dy * scale
			)
		
		# Generate smooth path from current to tangent point using cubic interpolation
		path: List[Point2D] = []
		for i in range(samples // 2):
			t = (i + 1) / (samples // 2)
			# Use smooth interpolation (ease-in-out cubic)
			t_smooth = t * t * (3.0 - 2.0 * t)
			x = current[0] + (target_on_circle[0] - current[0]) * t_smooth
			y = current[1] + (target_on_circle[1] - current[1]) * t_smooth
			path.append((x, y))
		
		# Continue along the circle to the goal
		angle_on_circle = math.atan2(
			target_on_circle[1] - self.circle_center[1],
			target_on_circle[0] - self.circle_center[0]
		)
		goal_angle = math.atan2(
			self.target_point[1] - self.circle_center[1],
			self.target_point[0] - self.circle_center[0]
		)
		
		# Calculate angular distance considering circle direction
		angle_diff = self._angle_wrap(goal_angle - angle_on_circle)
		if self.circle_direction < 0:
			if angle_diff > 0:
				angle_diff -= 2 * math.pi
		else:
			if angle_diff < 0:
				angle_diff += 2 * math.pi
		
		step = angle_diff / max(samples // 2, 1)
		for i in range(samples // 2):
			angle = angle_on_circle + step * (i + 1)
			x = self.circle_center[0] + self.circle_radius * math.cos(angle)
			y = self.circle_center[1] + self.circle_radius * math.sin(angle)
			path.append((x, y))
		
		return path

	@staticmethod
	def _distance(p1: Point2D, p2: Point2D) -> float:
		return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

	@staticmethod
	def _angle_wrap(angle: float) -> float:
		"""Wrap angle to [-pi, pi]."""
		while angle > math.pi:
			angle -= 2 * math.pi
		while angle < -math.pi:
			angle += 2 * math.pi
		return angle

	@staticmethod
	def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
		"""Convert quaternion to yaw (Z-axis)."""
		sin_yaw = 2.0 * (w * z + x * y)
		cos_yaw = 1.0 - 2.0 * (y * y + z * z)
		return math.atan2(sin_yaw, cos_yaw)

	def _publish_stop(self) -> None:
		self.cmd_pub.publish(Twist())


def main(args=None) -> None:
	rclpy.init(args=args)
	node = CircularTrajectoryNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()



