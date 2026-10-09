#!/usr/bin/env python3
"""
Human Controller Node for SmartCart Simulation.

Controls the movement of an individual simulated human in Gazebo:
- Modular & Namespace-aware: Reusable across multiple humans (/human_1, /human_2, etc.).
- Manual/Teleop mode (default): Stationary until actively commanded via `cmd_vel`.
- Wall-clock decay: Instantly zeroes velocity within 150ms when keys are released.
- Publishes world position and orientation to `pose` (e.g. /human_1/pose).
- Supports auto patrol mode along X or Y axis for crowd background motion.
- Optional legacy publication to `/human/pose` for backward compatibility.
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


class HumanController(Node):
    """Controls simulated human movement and publishes namespaced pose."""

    def __init__(self):
        super().__init__('human_controller')

        # Parameters
        self.declare_parameter('mode', 'manual')       # 'manual' (default) or 'auto'
        self.declare_parameter('start_x', 2.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('end_x', 6.0)
        self.declare_parameter('end_y', 0.0)
        self.declare_parameter('patrol_axis', 'x')     # 'x' or 'y' for auto patrol
        self.declare_parameter('speed', 0.25)          # m/s in auto mode
        self.declare_parameter('update_rate', 20.0)    # 20 Hz for smooth walking
        self.declare_parameter('entity_name', '')      # Gazebo model entity name
        self.declare_parameter('min_x', -6.5)
        self.declare_parameter('max_x', 18.5)
        self.declare_parameter('min_y', -8.5)
        self.declare_parameter('max_y', 8.5)
        self.declare_parameter('publish_legacy_topic', False)

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.start_x = self.get_parameter('start_x').get_parameter_value().double_value
        self.start_y = self.get_parameter('start_y').get_parameter_value().double_value
        self.start_yaw = self.get_parameter('start_yaw').get_parameter_value().double_value
        self.end_x = self.get_parameter('end_x').get_parameter_value().double_value
        self.end_y = self.get_parameter('end_y').get_parameter_value().double_value
        self.patrol_axis = self.get_parameter('patrol_axis').get_parameter_value().string_value.lower()
        self.auto_speed = self.get_parameter('speed').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.min_x = self.get_parameter('min_x').get_parameter_value().double_value
        self.max_x = self.get_parameter('max_x').get_parameter_value().double_value
        self.min_y = self.get_parameter('min_y').get_parameter_value().double_value
        self.max_y = self.get_parameter('max_y').get_parameter_value().double_value
        self.publish_legacy = self.get_parameter('publish_legacy_topic').get_parameter_value().bool_value

        # Infer Gazebo entity name from parameter or node namespace
        raw_entity_name = self.get_parameter('entity_name').get_parameter_value().string_value
        ns = self.get_namespace().strip('/')
        if raw_entity_name:
            self.entity_name = raw_entity_name
        elif ns:
            self.entity_name = ns
        else:
            self.entity_name = 'human_1'

        self.dt = 1.0 / self.update_rate

        # Planar state
        self.current_x = self.start_x
        self.current_y = self.start_y
        self.current_yaw = self.start_yaw
        self.auto_direction = 1.0

        # Teleop velocity inputs
        self.teleop_vx = 0.0
        self.teleop_vy = 0.0
        self.teleop_wz = 0.0
        self.last_cmd_wall_time = None

        # Relative publisher for human pose (e.g. /<namespace>/pose)
        self.pose_pub = self.create_publisher(Pose, 'pose', 10)

        # Legacy publisher for /human/pose if enabled
        self.legacy_pose_pub = None
        if self.publish_legacy or self.entity_name in ('human_1', 'human'):
            self.legacy_pose_pub = self.create_publisher(Pose, '/human/pose', 10)

        # Relative subscriber for manual teleop commands (e.g. /<namespace>/cmd_vel)
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 1
        )

        # Legacy subscriber for /human/cmd_vel if enabled or if human_1
        if self.publish_legacy or self.entity_name in ('human_1', 'human'):
            self.legacy_cmd_sub = self.create_subscription(
                Twist, '/human/cmd_vel', self.cmd_callback, 1
            )

        # Service client for setting Gazebo entity pose
        self.set_pose_client = self.create_client(
            SetEntityPose, '/world/smartcart_world/set_pose'
        )

        # Pre-allocate entity message
        self.human_entity = Entity()
        self.human_entity.name = self.entity_name
        self.human_entity.type = Entity.MODEL

        self.service_connected = False
        self.log_counter = 0

        self.get_logger().info(
            f'Human controller for [{self.entity_name}] initialized in [{self.mode.upper()}] mode '
            f'at ({self.start_x:.1f}, {self.start_y:.1f})'
        )

        # 20 Hz Timer Loop
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def cmd_callback(self, msg: Twist):
        """Handle incoming teleoperation velocity command."""
        self.teleop_vx = msg.linear.x
        self.teleop_vy = msg.linear.y
        self.teleop_wz = msg.angular.z
        self.last_cmd_wall_time = time.time()

        # Switch to manual mode if a teleop command is received
        if self.mode != 'manual':
            self.mode = 'manual'
            self.get_logger().info(f'Switched [{self.entity_name}] to [MANUAL TELEOP] mode via cmd_vel')

    def timer_callback(self):
        """Update human position, set pose in Gazebo, and publish to ROS topic."""
        # Connect to Gazebo service if not connected
        if not self.service_connected:
            if self.set_pose_client.service_is_ready():
                self.service_connected = True
                self.get_logger().info(f'Gazebo set_pose service connected for [{self.entity_name}].')
            else:
                if self.log_counter % 40 == 0:
                    self.get_logger().info(f'[{self.entity_name}] Waiting for Gazebo set_pose service...')
                self.log_counter += 1

        if self.mode == 'manual':
            # Wall-clock fast decay timeout: if no active keypress received for > 0.15s, stop immediately
            if self.last_cmd_wall_time is not None:
                if (time.time() - self.last_cmd_wall_time) > 0.15:
                    self.teleop_vx = 0.0
                    self.teleop_vy = 0.0
                    self.teleop_wz = 0.0

            # Kinematic update in human local frame
            cos_y = math.cos(self.current_yaw)
            sin_y = math.sin(self.current_yaw)

            # World frame displacements
            dx = (self.teleop_vx * cos_y - self.teleop_vy * sin_y) * self.dt
            dy = (self.teleop_vx * sin_y + self.teleop_vy * cos_y) * self.dt

            self.current_x += dx
            self.current_y += dy
            self.current_yaw += self.teleop_wz * self.dt

            # Normalize yaw to [-pi, pi]
            self.current_yaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))

            # Clamp within supermarket boundaries
            self.current_x = max(self.min_x, min(self.current_x, self.max_x))
            self.current_y = max(self.min_y, min(self.current_y, self.max_y))

        else:
            # Auto mode: oscillate along designated patrol axis
            if self.patrol_axis == 'y':
                self.current_y += self.auto_direction * self.auto_speed * self.dt
                if self.auto_direction > 0 and self.current_y >= self.end_y:
                    self.current_y = self.end_y
                    self.auto_direction = -1.0
                    self.current_yaw = -math.pi / 2.0
                elif self.auto_direction < 0 and self.current_y <= self.start_y:
                    self.current_y = self.start_y
                    self.auto_direction = 1.0
                    self.current_yaw = math.pi / 2.0
            else:
                self.current_x += self.auto_direction * self.auto_speed * self.dt
                if self.auto_direction > 0 and self.current_x >= self.end_x:
                    self.current_x = self.end_x
                    self.auto_direction = -1.0
                    self.current_yaw = math.pi
                elif self.auto_direction < 0 and self.current_x <= self.start_x:
                    self.current_x = self.start_x
                    self.auto_direction = 1.0
                    self.current_yaw = 0.0

        # Build Pose message
        pose_msg = Pose()
        pose_msg.position.x = float(self.current_x)
        pose_msg.position.y = float(self.current_y)
        pose_msg.position.z = 0.0

        # Planar yaw quaternion
        pose_msg.orientation.w = math.cos(self.current_yaw / 2.0)
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = math.sin(self.current_yaw / 2.0)

        # Call Gazebo service if connected
        if self.service_connected:
            req = SetEntityPose.Request()
            req.entity = self.human_entity
            req.pose = pose_msg
            self.set_pose_client.call_async(req)

        # Publish relative pose topic
        self.pose_pub.publish(pose_msg)

        # Publish legacy topic if enabled
        if self.legacy_pose_pub is not None:
            self.legacy_pose_pub.publish(pose_msg)

        # Throttled status logging (~1 Hz)
        if self.log_counter % 20 == 0:
            yaw_deg = math.degrees(self.current_yaw)
            is_moving = abs(self.teleop_vx) > 0.01 or abs(self.teleop_vy) > 0.01 or abs(self.teleop_wz) > 0.01
            status_str = "WALKING" if (is_moving or self.mode == 'auto') else "STATIONARY"
            self.get_logger().info(
                f'[{self.entity_name}][{self.mode.upper()}]: ({self.current_x:+.2f}m, {self.current_y:+.2f}m) | '
                f'Yaw: {yaw_deg:+.1f}° | State: {status_str}'
            )
        self.log_counter += 1

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HumanController()
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
