#!/usr/bin/env python3
"""
Human Controller Node for SmartCart Simulation.

Controls the movement of the simulated human in Gazebo and publishes the
human's world position to `/human/pose` (geometry_msgs/msg/Pose).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


class HumanController(Node):
    """Controls simulated human movement and publishes /human/pose."""

    def __init__(self):
        super().__init__('human_controller')

        # Declare and retrieve movement parameters
        self.declare_parameter('start_x', 2.0)
        self.declare_parameter('end_x', 6.0)
        self.declare_parameter('y_pos', 0.0)
        self.declare_parameter('speed', 0.2)          # m/s
        self.declare_parameter('update_rate', 10.0)    # Hz
        self.declare_parameter('entity_name', 'human')

        self.start_x = self.get_parameter('start_x').get_parameter_value().double_value
        self.end_x = self.get_parameter('end_x').get_parameter_value().double_value
        self.y_pos = self.get_parameter('y_pos').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.entity_name = self.get_parameter('entity_name').get_parameter_value().string_value

        self.dt = 1.0 / self.update_rate
        self.current_x = self.start_x
        self.direction = 1.0  # +1 moving forward, -1 moving backward

        # Publisher for human position
        self.pose_pub = self.create_publisher(Pose, '/human/pose', 10)

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

        self.get_logger().info('Human controller started')

        # Wait for service in timer loop without blocking
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        """Update human position, set pose in Gazebo, and publish to ROS topic."""
        # Check service connection
        if not self.service_connected:
            if self.set_pose_client.service_is_ready():
                self.service_connected = True
                self.get_logger().info('Gazebo set_pose service connected. Human movement started.')
            else:
                if self.log_counter % 30 == 0:
                    self.get_logger().info('Waiting for Gazebo set_pose service...')
                self.log_counter += 1

        # Advance position
        self.current_x += self.direction * self.speed * self.dt

        # Check bounds and reverse direction smoothly
        if self.current_x >= self.end_x:
            self.current_x = self.end_x
            self.direction = -1.0
            self.get_logger().info(f'Human reached x={self.end_x:.1f}m, reversing direction')
        elif self.current_x <= self.start_x:
            self.current_x = self.start_x
            self.direction = 1.0
            self.get_logger().info(f'Human reached x={self.start_x:.1f}m, moving forward')

        # Build Pose message
        pose_msg = Pose()
        pose_msg.position.x = float(self.current_x)
        pose_msg.position.y = float(self.y_pos)
        pose_msg.position.z = 0.0
        # Facing direction
        if self.direction > 0:
            pose_msg.orientation.w = 1.0  # yaw = 0
            pose_msg.orientation.z = 0.0
        else:
            pose_msg.orientation.w = 0.0  # yaw = pi
            pose_msg.orientation.z = 1.0

        # Call Gazebo service if connected
        if self.service_connected:
            req = SetEntityPose.Request()
            req.entity = self.human_entity
            req.pose = pose_msg
            self.set_pose_client.call_async(req)

        # Publish /human/pose topic for controllers
        self.pose_pub.publish(pose_msg)

        # Throttled logging (~every 2 seconds)
        if self.log_counter % 20 == 0:
            self.get_logger().info(
                f'Human Pose: x={self.current_x:.2f}m, y={self.y_pos:.2f}m, '
                f'dir={"forward (+X)" if self.direction > 0 else "backward (-X)"}'
            )
        self.log_counter += 1


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
