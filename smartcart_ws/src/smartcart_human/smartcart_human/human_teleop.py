#!/usr/bin/env python3
"""
Interactive Keyboard Teleoperation Node for Human in SmartCart Simulation.

Provides direct, responsive hold-to-move keyboard control of the simulated human:
- Human moves ONLY when user actively presses / holds movement keys.
- Instant stop when keys are released.
- WASD / Arrow Keys for walking & turning.
- Real-time ASCII HUD showing live coordinates, speeds, and cart follow state.
"""

import sys
import select
import termios
import tty
import math
import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry


BANNER = """
╔═══════════════════════════════════════════════════════════════════════════╗
║                   SmartCart Human Keyboard Teleop                         ║
╠═══════════════════════════════════════════════════════════════════════════╣
║  Direct Movement Controls (Hold key to move, Release to stop):            ║
║    [W] / [↑] : Walk Forward          [S] / [↓] : Walk Backward            ║
║    [A] / [←] : Turn Left             [D] / [→] : Turn Right               ║
║    [Q]       : Strafe Left           [E]       : Strafe Right             ║
║    [SPACE]   : Stop Instantly                                             ║
║                                                                           ║
║  Speed Adjustment:                                                        ║
║    [+] / [=] : Speed Up (+0.05 m/s)  [-] / [_] : Slow Down (-0.05 m/s)    ║
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

        # Configurable speeds
        self.linear_speed = 0.35      # m/s
        self.strafe_speed = 0.25      # m/s
        self.angular_speed = 0.85     # rad/s

        # State tracking for HUD
        self.human_x = 2.0
        self.human_y = 0.0
        self.human_yaw = 0.0
        self.cart_x = 0.0
        self.cart_y = 0.0
        self.running = True

    def human_cb(self, msg: Pose):
        self.human_x = msg.position.x
        self.human_y = msg.position.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        self.human_yaw = 2.0 * math.atan2(qz, qw)

    def odom_cb(self, msg: Odometry):
        self.cart_x = msg.pose.pose.position.x
        self.cart_y = msg.pose.pose.position.y

    def publish_vel(self, vx: float, vy: float, wz: float):
        """Publish velocity command to /human/cmd_vel."""
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        self.cmd_pub.publish(cmd)


def get_key(settings, timeout=0.03):
    """Read a keypress or escape sequence non-blockingly."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # Escape or Arrow key sequence
            # Check if more characters are available
            rlist2, _, _ = select.select([sys.stdin], [], [], 0.02)
            if rlist2:
                extra = sys.stdin.read(2)
                key += extra
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def run_keyboard_loop(node: HumanTeleop, settings):
    """Interactive real-time hold-to-move keyboard loop."""
    print(BANNER)

    target_vx = 0.0
    target_vy = 0.0
    target_wz = 0.0
    last_key_time = 0.0
    KEY_TIMEOUT = 0.15  # Stop movement if no key received for 150ms

    last_hud_time = 0.0

    try:
        while node.running and rclpy.ok():
            key = get_key(settings, timeout=0.03)
            now = time.time()

            if key:
                if key in ('\x03', '\x1b'):  # Ctrl+C or ESC
                    break
                elif key in ('w', 'W', '\x1b[A'):  # Walk Forward
                    target_vx = node.linear_speed
                    target_vy = 0.0
                    target_wz = 0.0
                    last_key_time = now
                elif key in ('s', 'S', '\x1b[B'):  # Walk Backward
                    target_vx = -node.linear_speed
                    target_vy = 0.0
                    target_wz = 0.0
                    last_key_time = now
                elif key in ('a', 'A', '\x1b[D'):  # Turn Left
                    target_vx = 0.0
                    target_vy = 0.0
                    target_wz = node.angular_speed
                    last_key_time = now
                elif key in ('d', 'D', '\x1b[C'):  # Turn Right
                    target_vx = 0.0
                    target_vy = 0.0
                    target_wz = -node.angular_speed
                    last_key_time = now
                elif key in ('q', 'Q'):            # Strafe Left
                    target_vx = 0.0
                    target_vy = node.strafe_speed
                    target_wz = 0.0
                    last_key_time = now
                elif key in ('e', 'E'):            # Strafe Right
                    target_vx = 0.0
                    target_vy = -node.strafe_speed
                    target_wz = 0.0
                    last_key_time = now
                elif key in (' ', 'x', 'X'):       # Stop Instantly
                    target_vx = 0.0
                    target_vy = 0.0
                    target_wz = 0.0
                    last_key_time = 0.0
                elif key in ('+', '=', 'u', 'U'):  # Speed Up
                    node.linear_speed = min(1.0, node.linear_speed + 0.05)
                    node.angular_speed = min(2.0, node.angular_speed + 0.1)
                elif key in ('-', '_', 'j', 'J'):  # Speed Down
                    node.linear_speed = max(0.1, node.linear_speed - 0.05)
                    node.angular_speed = max(0.2, node.angular_speed - 0.1)

            # Hold-to-move check: if key released (>150ms ago), stop human completely
            if now - last_key_time > KEY_TIMEOUT:
                target_vx = 0.0
                target_vy = 0.0
                target_wz = 0.0

            # Publish velocity at 20 Hz
            node.publish_vel(target_vx, target_vy, target_wz)

            # Refresh HUD at ~10 Hz
            if now - last_hud_time >= 0.10:
                last_hud_time = now
                dx = node.human_x - node.cart_x
                dy = node.human_y - node.cart_y
                dist = math.sqrt(dx * dx + dy * dy)
                yaw_deg = math.degrees(node.human_yaw)

                is_moving = abs(target_vx) > 0.01 or abs(target_vy) > 0.01 or abs(target_wz) > 0.01
                human_state = "MOVING" if is_moving else "STOPPED"

                if dist <= 1.25:
                    cart_state = "TARGET REACHED (~1.2m)"
                elif dist > 1.25:
                    cart_state = f"FOLLOWING ({dist:.2f}m)"
                else:
                    cart_state = "IDLE"

                hud = (
                    f"\r🚶 Human [{human_state}]: ({node.human_x:+5.2f}, {node.human_y:+5.2f}) | "
                    f"Yaw: {yaw_deg:+5.1f}° | Speed: {node.linear_speed:.2f}m/s | "
                    f"🛒 Cart: ({node.cart_x:+5.2f}, {node.cart_y:+5.2f}) | "
                    f"Dist: {dist:4.2f}m | [{cart_state}]   "
                )
                sys.stdout.write(hud)
                sys.stdout.flush()

    except Exception as e:
        print(f"\nTeleop exception: {e}")
    finally:
        node.publish_vel(0.0, 0.0, 0.0)
        node.running = False


def main(args=None):
    # Save terminal settings for raw single-key capture
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
        node.publish_vel(0.0, 0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\nHuman Teleoperation stopped cleanly.\n")


if __name__ == '__main__':
    main()
