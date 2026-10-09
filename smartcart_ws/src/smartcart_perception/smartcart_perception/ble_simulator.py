#!/usr/bin/env python3
"""
BLE Customer Beacon Simulator Node for SmartCart Simulation.

Simulates a wearable/fob BLE tag carried by the designated customer (Human 1):
- Implements Log-Distance Path Loss Model with indoor shadowing noise.
- Publishes structured tag advertisement packets to `/ble/customer_tag`.
- Provides customer identity confirmation & coarse proximity estimates.
"""

import json
import math
import random
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


class BLESimulator(Node):
    """Simulates a customer BLE beacon tag with realistic RSSI attenuation."""

    def __init__(self):
        super().__init__('ble_simulator')

        # Parameters
        self.declare_parameter('customer_tag_id', 'SMARTCART-CUSTOMER-001')
        self.declare_parameter('target_human_id', 1)
        self.declare_parameter('tx_power', -59.0)          # Measured RSSI at 1.0 meter (dBm)
        self.declare_parameter('path_loss_exp', 2.4)       # Indoor supermarket path loss exponent
        self.declare_parameter('noise_std', 1.2)           # RSSI Gaussian shadow fading std dev
        self.declare_parameter('update_rate', 10.0)        # 10 Hz BLE broadcast rate

        self.tag_id = self.get_parameter('customer_tag_id').get_parameter_value().string_value
        self.target_id = self.get_parameter('target_human_id').get_parameter_value().integer_value
        self.tx_power = self.get_parameter('tx_power').get_parameter_value().double_value
        self.n_exp = self.get_parameter('path_loss_exp').get_parameter_value().double_value
        self.noise_std = self.get_parameter('noise_std').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value

        # Publisher for BLE beacon packets
        self.ble_pub = self.create_publisher(String, '/ble/customer_tag', 10)

        # State tracking
        self.customer_x = 2.0
        self.customer_y = 0.0
        self.cart_x = 0.0
        self.cart_y = 0.0
        self.customer_received = False
        self.log_counter = 0

        # Subscriptions
        self.create_subscription(
            Pose, f'/human_{self.target_id}/pose', self.customer_cb, 10
        )
        self.create_subscription(
            Odometry, '/wheel/odometry', self.odom_cb, 10
        )

        self.get_logger().info(
            f'BLE Simulator initialized for Customer Tag [{self.tag_id}] tracking Human {self.target_id}'
        )

        # 10 Hz broadcast timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)

    def customer_cb(self, msg: Pose):
        self.customer_x = msg.position.x
        self.customer_y = msg.position.y
        self.customer_received = True

    def odom_cb(self, msg: Odometry):
        self.cart_x = msg.pose.pose.position.x
        self.cart_y = msg.pose.pose.position.y

    def calculate_rssi(self, distance: float) -> tuple:
        """
        Calculate simulated RSSI using the Log-Distance Path Loss Model:
        RSSI(d) = RSSI(d0) - 10 * n * log10(d / d0) + X_sigma
        """
        effective_dist = max(distance, 0.20)
        mean_rssi = self.tx_power - (10.0 * self.n_exp * math.log10(effective_dist))
        noise = random.gauss(0.0, self.noise_std)
        noisy_rssi = mean_rssi + noise

        # Estimate coarse distance from noisy RSSI
        ratio = (self.tx_power - noisy_rssi) / (10.0 * self.n_exp)
        est_distance = math.pow(10.0, ratio)

        # Proximity category
        if effective_dist < 1.0:
            proximity = "IMMEDIATE"
        elif effective_dist < 3.0:
            proximity = "NEAR"
        elif effective_dist < 8.0:
            proximity = "FAR"
        else:
            proximity = "OUT_OF_RANGE"

        return float(noisy_rssi), float(est_distance), proximity

    def timer_callback(self):
        dx = self.customer_x - self.cart_x
        dy = self.customer_y - self.cart_y
        true_dist = math.sqrt(dx * dx + dy * dy)

        rssi, est_dist, proximity = self.calculate_rssi(true_dist)

        packet = {
            "tag_id": self.tag_id,
            "target_human_id": self.target_id,
            "rssi_dbm": round(rssi, 2),
            "approx_distance_m": round(est_dist, 2),
            "true_distance_m": round(true_dist, 2),
            "proximity": proximity,
            "timestamp": time.time()
        }

        msg = String()
        msg.data = json.dumps(packet)
        self.ble_pub.publish(msg)

        if self.log_counter % 20 == 0:
            self.get_logger().info(
                f'BLE [{self.tag_id}] RSSI: {rssi:.1f} dBm | Dist: ~{est_dist:.2f}m (True: {true_dist:.2f}m) | [{proximity}]'
            )
        self.log_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = BLESimulator()
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
