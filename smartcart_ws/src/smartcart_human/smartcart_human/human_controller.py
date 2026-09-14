#!/usr/bin/env python3
"""
Human Controller Node for SmartCart Simulation.

Controls the movement of the simulated human in Gazebo:
- Manual/Teleop mode (default): Human is completely stationary until user actively commands
  velocity via `/human/cmd_vel` (geometry_msgs/msg/Twist).
- Wall-clock decay: Instantly zeroes velocity within 150ms when user releases keys.
- Publishes the human's world position and orientation to `/human/pose` (geometry_msgs/msg/Pose).
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


class HumanController(Node):
    """Controls simulated human movement and publishes /human/pose."""

    def __init__(self):
        super().__init__('human_controller')

        # Parameters (Default: manual stationary mode, moved exclusively by user)
        self.declare_parameter('mode', 'manual')       # 'manual' (default) or 'auto'
        self.declare_parameter('start_x', 2.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('end_x', 6.0)
        self.declare_parameter('speed', 0.2)           # m/s in auto mode
        self.declare_parameter('update_rate', 20.0)    # 20 Hz for smooth walking
        self.declare_parameter('entity_name', 'human')

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.start_x = self.get_parameter('start_x').get_parameter_value().double_value
        self.start_y = self.get_parameter('start_y').get_parameter_value().double_value
        self.start_yaw = self.get_parameter('start_yaw').get_parameter_value().double_value
        self.end_x = self.get_parameter('end_x').get_parameter_value().double_value
        self.auto_speed = self.get_parameter('speed').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.entity_name = self.get_parameter('entity_name').get_parameter_value().string_value

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

        # Publisher for human position
        self.pose_pub = self.create_publisher(Pose, '/human/pose', 10)

        # Subscriber for manual teleop commands (queue depth 1 for zero-latency response)
        self.cmd_sub = self.create_subscription(
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

        self.get_logger().info(f'Human controller initialized in [{self.mode.upper()}] mode (User Controlled)')

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
            self.get_logger().info('Switched human controller to [MANUAL TELEOP] mode via /human/cmd_vel')

    def timer_callback(self):
        """Update human position, set pose in Gazebo, and publish to ROS topic."""
        # Connect to Gazebo service if not connected
        if not self.service_connected:
            if self.set_pose_client.service_is_ready():
                self.service_connected = True
                self.get_logger().info('Gazebo set_pose service connected. Ready for user control.')
            else:
                if self.log_counter % 40 == 0:
                    self.get_logger().info('Waiting for Gazebo set_pose service...')
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
            self.current_x = max(-5.5, min(self.current_x, 5.5))
            self.current_y = max(-3.6, min(self.current_y, 3.6))

        else:
            # Auto mode: oscillate along X axis
            self.current_x += self.auto_direction * self.auto_speed * self.dt
            if self.current_x >= self.end_x:
                self.current_x = self.end_x
                self.auto_direction = -1.0
                self.current_yaw = math.pi
            elif self.current_x <= self.start_x:
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

        # Publish /human/pose topic
        self.pose_pub.publish(pose_msg)

        # Throttled status logging (~1 Hz)
        if self.log_counter % 20 == 0:
            yaw_deg = math.degrees(self.current_yaw)
            is_moving = abs(self.teleop_vx) > 0.01 or abs(self.teleop_vy) > 0.01 or abs(self.teleop_wz) > 0.01
            status_str = "WALKING" if is_moving else "STATIONARY"
            self.get_logger().info(
                f'[{self.mode.upper()}] Human: ({self.current_x:.2f}m, {self.current_y:.2f}m) | '
                f'Yaw: {yaw_deg:.1f}° | State: {status_str}'
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
