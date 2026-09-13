#!/usr/bin/env python3
"""
Follow Controller Node for SmartCart Simulation.

Tracks simulated human position, applies smooth proportional differential-drive
control, monitors LiDAR front sector for safety emergency stop, and handles
lost-human timeouts with stable turning and forward velocity.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


# Controller Constants (Authoritative Specification Contract)
TARGET_DISTANCE = 1.2          # Target following distance in meters
DEADBAND_DISTANCE = 0.05       # Deadband around target distance (m)
K_DISTANCE = 0.5               # Proportional gain for linear velocity
K_ANGLE = 0.8                  # Proportional gain for angular velocity
MAX_LINEAR_SPEED = 0.45        # Maximum linear speed (m/s)
MAX_ANGULAR_SPEED = 0.8        # Maximum angular speed (rad/s)
OBSTACLE_STOP_DISTANCE = 0.60  # Emergency stop threshold (meters)
HUMAN_TIMEOUT = 1.0            # Lost human timeout in seconds
FRONT_SECTOR_RAD = math.pi / 6 # ±30 degrees front safety cone
MIN_VALID_LIDAR_DIST = 0.20    # Filter internal reflections


class FollowController(Node):
    """Proportional human-following and obstacle safety controller."""

    def __init__(self):
        super().__init__('follow_controller')

        # Publisher for robot velocity
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.human_sub = self.create_subscription(
            Pose, '/human/pose', self.human_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/wheel/odometry', self.odom_callback, 10
        )

        # State tracking
        self.last_human_pose = None
        self.last_human_time = None
        self.last_scan = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        self.log_counter = 0

        self.get_logger().info('Follow controller started')

        # 10 Hz Control Loop Timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def human_callback(self, msg: Pose):
        """Record latest human pose and arrival timestamp."""
        self.last_human_pose = msg
        self.last_human_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan):
        """Record latest LiDAR scan."""
        self.last_scan = msg

    def odom_callback(self, msg: Odometry):
        """Extract planar robot pose (x, y, yaw) from odometry using full 3D quaternion."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def get_human_age(self) -> float:
        """Calculate time elapsed since last human pose in seconds."""
        if self.last_human_time is None:
            return float('inf')
        duration = self.get_clock().now() - self.last_human_time
        return duration.nanoseconds / 1e9

    def calculate_relative_position(self, hx: float, hy: float) -> tuple:
        """
        Transform human world position (hx, hy) to robot-local coordinates.
        Returns: (human_x_local, human_y_local)
          +X: directly in front of the robot
          +Y: to the robot's left
        """
        dx = hx - self.robot_x
        dy = hy - self.robot_y

        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)

        human_x = cos_yaw * dx + sin_yaw * dy
        human_y = -sin_yaw * dx + cos_yaw * dy
        return human_x, human_y

    def calculate_distance(self, human_x: float, human_y: float) -> float:
        """Calculate Euclidean distance to human in robot frame."""
        return math.sqrt(human_x * human_x + human_y * human_y)

    def get_front_min_range(self) -> float:
        """
        Scan LiDAR ranges in the front ±30 degree cone.
        Returns minimum valid obstacle range in meters, ignoring reflections < 0.20m.
        """
        if self.last_scan is None:
            return float('inf')

        scan = self.last_scan
        front_min = float('inf')

        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if -FRONT_SECTOR_RAD <= angle <= FRONT_SECTOR_RAD:
                if math.isnan(r) or math.isinf(r):
                    continue
                # Ignore out-of-range returns
                if r < max(scan.range_min, MIN_VALID_LIDAR_DIST) or r > scan.range_max:
                    continue
                if r < front_min:
                    front_min = r

        return front_min

    def is_obstacle_detected(self, front_min: float) -> bool:
        """Check if any front obstacle is below the emergency stop threshold."""
        return front_min < OBSTACLE_STOP_DISTANCE

    def calculate_follow_velocity(self, distance: float, human_x: float, human_y: float) -> tuple:
        """
        Calculate smooth, clamped linear and angular velocities.
        Rotates on the spot if human is behind/lateral; drives forward when facing human.
        Returns: (linear_x, angular_z)
        """
        # Bearing angle to human in robot local frame (-pi to +pi)
        bearing_angle = math.atan2(human_y, human_x)

        # Proportional angular velocity to turn toward human
        angular_z = K_ANGLE * bearing_angle
        angular_z = max(-MAX_ANGULAR_SPEED, min(angular_z, MAX_ANGULAR_SPEED))

        # Only drive forward if human is in the forward half-plane and within ±45 deg cone
        if human_x > 0.0 and abs(bearing_angle) < (math.pi / 4.0):
            distance_error = distance - TARGET_DISTANCE
            if distance_error <= DEADBAND_DISTANCE:
                linear_x = 0.0
            else:
                linear_x = K_DISTANCE * (distance_error - DEADBAND_DISTANCE)
                linear_x = max(0.0, min(linear_x, MAX_LINEAR_SPEED))
        else:
            # Target is to the side or behind: rotate to face target, do NOT drive forward
            linear_x = 0.0

        return linear_x, angular_z

    def publish_stop(self):
        """Publish zero linear and angular velocities."""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0
        stop_cmd.angular.x = 0.0
        stop_cmd.angular.y = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_pub.publish(stop_cmd)

    def control_loop(self):
        """Main 10 Hz control loop executing prioritized safety & follow logic."""
        # 1. Check human pose freshness
        human_age = self.get_human_age()
        human_lost = human_age > HUMAN_TIMEOUT

        # 2. Check front obstacle safety
        front_min = self.get_front_min_range()
        obstacle_close = self.is_obstacle_detected(front_min)

        # State evaluation according to strict specification safety priority:
        # Priority 1: Human lost -> STOP
        # Priority 2: Obstacle close -> STOP
        # Priority 3: Normal Following Controller
        state = "STOP"
        linear_x = 0.0
        angular_z = 0.0
        distance = 0.0

        if human_lost:
            state = "HUMAN_LOST"
            self.publish_stop()

        elif obstacle_close:
            state = "EMERGENCY_STOP"
            self.publish_stop()

        else:
            # Valid human pose available
            hx = self.last_human_pose.position.x
            hy = self.last_human_pose.position.y

            human_x, human_y = self.calculate_relative_position(hx, hy)
            distance = self.calculate_distance(human_x, human_y)

            if distance <= (TARGET_DISTANCE + DEADBAND_DISTANCE) and human_x > 0.0:
                state = "TARGET_REACHED"
                # Keep distance, but smoothly align with human if slightly offset
                _, angular_z = self.calculate_follow_velocity(distance, human_x, human_y)
                linear_x = 0.0
            else:
                state = "FOLLOW"
                linear_x, angular_z = self.calculate_follow_velocity(distance, human_x, human_y)

            cmd = Twist()
            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)
            self.cmd_pub.publish(cmd)

        # Throttled status log (~1 Hz)
        if self.log_counter % 10 == 0:
            human_status = "LOST" if human_lost else "DETECTED"
            obs_status = f"BLOCKED ({front_min:.2f}m)" if obstacle_close else "CLEAR"
            self.get_logger().info(
                f"[{state}] Human: {human_status} | Dist: {distance:.2f}m | "
                f"Obstacle: {obs_status} | v: {linear_x:.2f} m/s | w: {angular_z:.2f} rad/s"
            )
        self.log_counter += 1

    def destroy_node(self):
        """Ensure robot stops on node termination."""
        self.publish_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
