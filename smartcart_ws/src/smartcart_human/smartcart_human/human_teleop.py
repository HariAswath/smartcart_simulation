#!/usr/bin/env python3
"""
Interactive Keyboard Teleoperation Node for Human in SmartCart Simulation.

Enables real-time keyboard control of the simulated human in Gazebo:
- WASD / Arrow Keys for intuitive omnidirectional walking & turning
- Publishes velocity commands to `/human/cmd_vel` (geometry_msgs/msg/Twist)
- Live HUD displaying human coordinates, walking speed, cart distance, and follow state
"""

import sys
import select
import termios
import tty
import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry


BANNER = """
╔═══════════════════════════════════════════════════════════════════════════╗
║                   SmartCart Human Keyboard Teleop                         ║
╠═══════════════════════════════════════════════════════════════════════════╣
║  Moving & Walking:                                                        ║
║    [W] / [↑] : Walk Forward          [S] / [↓] : Walk Backward            ║
║    [A] / [←] : Turn Left (Yaw)       [D] / [→] : Turn Right (Yaw)         ║
║    [Q]       : Strafe Left           [E]       : Strafe Right             ║
║    [SPACE]/[X]: Stop Walking                                              ║
║                                                                           ║
║  Speed Adjustment:                                                        ║
║    [+] / [U] : Speed Up (+0.05 m/s)  [-] / [J] : Slow Down (-0.05 m/s)    ║
║    [Ctrl+C]  : Quit Teleop                                                ║
╚═══════════════════════════════════════════════════════════════════════════╝
"""


class HumanTeleop(Node):
    """Captures keyboard inputs and publishes /human/cmd_vel velocity commands."""

    def __init__(self):
        super().__init__('human_teleop')

        # Publisher for human velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/human/cmd_vel', 10)

        # Subscribers for live HUD
        self.human_sub = self.create_subscription(
            Pose, '/human/pose', self.human_cb, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/wheel/odometry', self.odom_cb, 10
        )

        # Teleop speeds
        self.linear_speed = 0.35      # m/s
        self.strafe_speed = 0.25      # m/s
        self.angular_speed = 0.80     # rad/s

        # Current commanded velocities
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        # State tracking for HUD
        self.human_x = 2.0
        self.human_y = 0.0
        self.human_yaw = 0.0
        self.cart_x = 0.0
        self.cart_y = 0.0
        self.running = True

        # 15 Hz Publisher Timer
        self.timer = self.create_timer(1.0 / 15.0, self.publish_cmd)

    def human_cb(self, msg: Pose):
        self.human_x = msg.position.x
        self.human_y = msg.position.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        self.human_yaw = 2.0 * math.atan2(qz, qw)

    def odom_cb(self, msg: Odometry):
        self.cart_x = msg.pose.pose.position.x
        self.cart_y = msg.pose.pose.position.y

    def publish_cmd(self):
        """Continuously publish commanded velocity."""
        cmd = Twist()
        cmd.linear.x = float(self.target_vx)
        cmd.linear.y = float(self.target_vy)
        cmd.angular.z = float(self.target_wz)
        self.cmd_pub.publish(cmd)

    def stop_human(self):
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.publish_cmd()


def get_key(settings, timeout=0.1):
    """Read a keypress or escape sequence non-blockingly."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # Arrow keys start with ESC [
            extra = sys.stdin.read(2)
            key += extra
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def run_keyboard_loop(node: HumanTeleop, settings):
    """Interactive terminal event loop."""
    print(BANNER)
    last_hud_print = 0

    try:
        while node.running and rclpy.ok():
            key = get_key(settings, timeout=0.08)

            if key:
                if key in ('\x03', '\x1b'):  # Ctrl+C or ESC
                    break
                elif key in ('w', 'W', '\x1b[A'):  # Forward
                    node.target_vx = node.linear_speed
                    node.target_vy = 0.0
                    node.target_wz = 0.0
                elif key in ('s', 'S', '\x1b[B'):  # Backward
                    node.target_vx = -node.linear_speed
                    node.target_vy = 0.0
                    node.target_wz = 0.0
                elif key in ('a', 'A', '\x1b[D'):  # Turn Left
                    node.target_wz = node.angular_speed
                elif key in ('d', 'D', '\x1b[C'):  # Turn Right
                    node.target_wz = -node.angular_speed
                elif key in ('q', 'Q'):            # Strafe Left
                    node.target_vy = node.strafe_speed
                elif key in ('e', 'E'):            # Strafe Right
                    node.target_vy = -node.strafe_speed
                elif key in (' ', 'x', 'X', 'k', 'K'):  # Stop
                    node.stop_human()
                elif key in ('+', '=', 'u', 'U'):  # Speed Up
                    node.linear_speed = min(1.0, node.linear_speed + 0.05)
                    node.angular_speed = min(2.0, node.angular_speed + 0.1)
                elif key in ('-', '_', 'j', 'J'):  # Speed Down
                    node.linear_speed = max(0.1, node.linear_speed - 0.05)
                    node.angular_speed = max(0.2, node.angular_speed - 0.1)

            # Calculate metrics for HUD
            dx = node.human_x - node.cart_x
            dy = node.human_y - node.cart_y
            dist = math.sqrt(dx * dx + dy * dy)
            yaw_deg = math.degrees(node.human_yaw)

            # Determine follow state
            if dist <= 1.25:
                cart_state = "TARGET REACHED (Cart at ~1.2m)"
            elif dist > 1.25:
                cart_state = f"FOLLOWING HUMAN (v > 0, dist={dist:.2f}m)"
            else:
                cart_state = "IDLE"

            # Print single-line dynamic HUD
            hud = (
                f"\r🚶 Human: ({node.human_x:+5.2f}, {node.human_y:+5.2f}) | "
                f"Yaw: {yaw_deg:+5.1f}° | Speed: {node.linear_speed:.2f}m/s | "
                f"🛒 Cart: ({node.cart_x:+5.2f}, {node.cart_y:+5.2f}) | "
                f"Dist: {dist:4.2f}m | [{cart_state}]"
            )
            sys.stdout.write(hud)
            sys.stdout.flush()

    except Exception as e:
        print(f"\nTeleop exception: {e}")
    finally:
        node.stop_human()
        node.running = False


def main(args=None):
    # Save terminal settings for raw key input
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = HumanTeleop()

    # Spin ROS 2 in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_keyboard_loop(node, settings)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop_human()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\nHuman Teleoperation stopped cleanly.\n")


if __name__ == '__main__':
    main()
