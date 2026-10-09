#!/usr/bin/env python3
"""
Follow Controller Node for SmartCart Simulation.

Tracks target position produced by the Perception/Target Selection layer:
- Subscribes to `/target/pose` (with fallback to `/human_1/pose` and `/human/pose`).
- Monitors `/target/status` to adjust dynamics during occlusion (coasting) or stop on lost target.
- Applies smooth proportional differential-drive skid-steer control.
- Handles 360-degree turnaround with hysteresis.
- Monitors LiDAR front sector for obstacle safety and emergency braking.
"""

import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# Controller Constants (Authoritative Specification Contract)
TARGET_DISTANCE = 1.20          # Target following distance in meters
DEADBAND_DISTANCE = 0.05       # Deadband around target distance (m)
K_DISTANCE = 0.70              # Proportional gain for linear velocity
K_ANGLE = 1.20                 # Proportional gain for angular velocity
MAX_LINEAR_SPEED = 0.45        # Maximum linear speed (m/s)
MIN_LINEAR_SPEED = 0.18        # Minimum linear speed to overcome 4WD ground friction
MAX_ANGULAR_SPEED = 1.00       # Maximum angular speed (rad/s)
MIN_ANGULAR_SPEED = 0.30       # Minimum angular speed to overcome skid friction
DEADBAND_ANGLE = 0.08          # ~4.5 degrees angular deadband (rad)
OBSTACLE_STOP_DISTANCE = 0.45  # Emergency stop threshold from LiDAR (meters)
HUMAN_TIMEOUT = 1.5            # Lost human timeout in seconds
FRONT_SECTOR_RAD = math.pi / 6 # ±30 degrees front safety cone
MIN_VALID_LIDAR_DIST = 0.18    # Filter internal sensor reflections


class FollowController(Node):
    """Proportional human-following and obstacle safety controller."""

    def __init__(self):
        super().__init__('follow_controller')

        # Parameters
        self.declare_parameter('target_pose_topic', '/target/pose')
        self.declare_parameter('target_human_id', 1)

        self.target_pose_topic = self.get_parameter('target_pose_topic').get_parameter_value().string_value
        self.target_human_id = self.get_parameter('target_human_id').get_parameter_value().integer_value

        # Publisher for robot velocity
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Primary target subscriber
        self.target_sub = self.create_subscription(
            Pose, self.target_pose_topic, self.target_callback, 10
        )

        # Fallback subscriptions for direct human pose
        if self.target_pose_topic != '/human_1/pose':
            self.create_subscription(
                Pose, '/human_1/pose', self.fallback_human_cb, 10
            )
        if self.target_pose_topic != '/human/pose':
            self.create_subscription(
                Pose, '/human/pose', self.fallback_human_cb, 10
            )

        # Perception status subscriber
        self.create_subscription(
            String, '/target/status', self.status_callback, 10
        )

        # Sensor subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/wheel/odometry', self.odom_callback, 10
        )

        # State tracking
        self.last_target_pose = None
        self.last_target_time = None
        self.target_status_data = None
        self.last_scan = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # Turnaround direction hysteresis (+1 left, -1 right)
        self.turn_direction = 1.0
        self.log_counter = 0

        self.get_logger().info(f'Follow controller initialized listening to [{self.target_pose_topic}]')

        # 10 Hz Control Loop Timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def target_callback(self, msg: Pose):
        """Record latest target pose and arrival timestamp."""
        self.last_target_pose = msg
        self.last_target_time = self.get_clock().now()

    def fallback_human_cb(self, msg: Pose):
        """Use ground-truth fallback only if primary target is not yet received."""
        if self.last_target_pose is None:
            self.last_target_pose = msg
            self.last_target_time = self.get_clock().now()

    def status_callback(self, msg: String):
        try:
            self.target_status_data = json.loads(msg.data)
        except Exception:
            pass

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def odom_callback(self, msg: Odometry):
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

    def get_target_age(self) -> float:
        if self.last_target_time is None:
            return float('inf')
        duration = self.get_clock().now() - self.last_target_time
        return duration.nanoseconds / 1e9

    def calculate_relative_position(self, hx: float, hy: float) -> tuple:
        dx = hx - self.robot_x
        dy = hy - self.robot_y

        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)

        target_x = cos_yaw * dx + sin_yaw * dy
        target_y = -sin_yaw * dx + cos_yaw * dy
        return target_x, target_y

    def calculate_distance(self, target_x: float, target_y: float) -> float:
        return math.sqrt(target_x * target_x + target_y * target_y)

    def get_front_min_range(self) -> float:
        if self.last_scan is None:
            return float('inf')

        scan = self.last_scan
        front_min = float('inf')

        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if -FRONT_SECTOR_RAD <= angle <= FRONT_SECTOR_RAD:
                if math.isnan(r) or math.isinf(r):
                    continue
                if r < max(scan.range_min, MIN_VALID_LIDAR_DIST) or r > scan.range_max:
                    continue
                if r < front_min:
                    front_min = r

        return front_min

    def is_obstacle_detected(self, front_min: float) -> bool:
        return front_min < OBSTACLE_STOP_DISTANCE

    def calculate_angular_velocity(self, target_x: float, target_y: float) -> float:
        bearing_angle = math.atan2(target_y, target_x)

        if abs(bearing_angle) > 2.8:  # ~160°
            effective_bearing = self.turn_direction * abs(bearing_angle)
        else:
            self.turn_direction = 1.0 if bearing_angle >= 0.0 else -1.0
            effective_bearing = bearing_angle

        if abs(effective_bearing) <= DEADBAND_ANGLE:
            return 0.0

        w = K_ANGLE * effective_bearing

        if w > 0:
            w = max(MIN_ANGULAR_SPEED, min(w, MAX_ANGULAR_SPEED))
        else:
            w = min(-MIN_ANGULAR_SPEED, max(w, -MAX_ANGULAR_SPEED))

        return float(w)

    def calculate_linear_velocity(self, distance: float, target_x: float, target_y: float, obstacle_close: bool, speed_scale: float = 1.0) -> float:
        if obstacle_close:
            return 0.0

        bearing_angle = math.atan2(target_y, target_x)

        if target_x > 0.0 and abs(bearing_angle) < (math.pi / 4.0):
            distance_error = distance - TARGET_DISTANCE
            if distance_error <= DEADBAND_DISTANCE:
                return 0.0
            else:
                linear_x = K_DISTANCE * (distance_error - DEADBAND_DISTANCE) * speed_scale
                linear_x = max(MIN_LINEAR_SPEED, min(linear_x, MAX_LINEAR_SPEED))
                return float(linear_x)

        return 0.0

    def publish_stop(self):
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

    def control_loop(self):
        target_age = self.get_target_age()
        target_lost = target_age > HUMAN_TIMEOUT

        perception_state = self.target_status_data.get("state", "UNKNOWN") if self.target_status_data else "UNKNOWN"
        if perception_state == "TARGET_LOST":
            target_lost = True

        front_min = self.get_front_min_range()
        obstacle_close = self.is_obstacle_detected(front_min)

        if target_lost:
            state = "TARGET_LOST"
            self.publish_stop()
            linear_x = 0.0
            angular_z = 0.0
            distance = 0.0

        else:
            hx = self.last_target_pose.position.x
            hy = self.last_target_pose.position.y

            target_x, target_y = self.calculate_relative_position(hx, hy)
            distance = self.calculate_distance(target_x, target_y)

            is_foreign_obstacle = obstacle_close and (distance > 0.85)

            if is_foreign_obstacle:
                state = "EMERGENCY_STOP"
                self.publish_stop()
                linear_x = 0.0
                angular_z = 0.0
            else:
                speed_scale = 0.60 if perception_state == "TEMPORARY_OCCLUSION" else 1.0

                angular_z = self.calculate_angular_velocity(target_x, target_y)
                linear_x = self.calculate_linear_velocity(distance, target_x, target_y, obstacle_close, speed_scale)

                if obstacle_close or (distance <= (TARGET_DISTANCE + DEADBAND_DISTANCE) and target_x > 0.0):
                    state = "TARGET_REACHED" if not obstacle_close else "PROXIMITY_HOLD"
                else:
                    state = "FOLLOW" if perception_state != "TEMPORARY_OCCLUSION" else "COAST_OCCLUDED"

                cmd = Twist()
                cmd.linear.x = float(linear_x)
                cmd.angular.z = float(angular_z)
                self.cmd_pub.publish(cmd)

        if self.log_counter % 10 == 0:
            target_status = "LOST" if target_lost else "TRACKED"
            obs_status = f"BLOCKED ({front_min:.2f}m)" if obstacle_close else "CLEAR"
            self.get_logger().info(
                f"[{state}] Target: {target_status} ({perception_state}) | Dist: {distance:.2f}m | "
                f"Obstacle: {obs_status} | v: {linear_x:.2f} m/s | w: {angular_z:.2f} rad/s"
            )
        self.log_counter += 1

    def destroy_node(self):
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
