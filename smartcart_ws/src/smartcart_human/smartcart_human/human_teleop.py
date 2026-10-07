#!/usr/bin/env python3
"""
Interactive Keyboard Teleoperation Node for Multi-Human Crowd in SmartCart Simulation.

Provides direct, responsive hold-to-move keyboard control of simulated people:
- Multi-Person selection: Select target person via parameter (`human_id:=N`) or
  interactively switch in real-time using keys [1], [2], [3], [4].
- Publishes strictly to the active person's namespace: `/human_<id>/cmd_vel`.
- Instant zero-latency stop when keys are released.
- WASD / Arrow Keys for walking & turning.
- Real-time ASCII HUD showing live coordinates of all 4 people, cart position, and follow state.
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
║             SmartCart Multi-Human Crowd Keyboard Teleop                   ║
╠═══════════════════════════════════════════════════════════════════════════╣
║  Person Selection:                                                        ║
║    [1] : Control Human 1 (Followed Target)  [2] : Control Human 2         ║
║    [3] : Control Human 3                    [4] : Control Human 4         ║
║                                                                           ║
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
    """Captures keyboard inputs and publishes to the active /human_<id>/cmd_vel topic."""

    def __init__(self):
        super().__init__('human_teleop')

        # Parameter for initial human to control
        self.declare_parameter('human_id', 1)
        self.human_id = self.get_parameter('human_id').get_parameter_value().integer_value
        if self.human_id not in (1, 2, 3, 4):
            self.human_id = 1

        # Publishers for all 4 human cmd_vel topics
        self.pubs = {
            i: self.create_publisher(Twist, f'/human_{i}/cmd_vel', 10)
            for i in (1, 2, 3, 4)
        }
        # Legacy publisher for /human/cmd_vel
        self.legacy_pub = self.create_publisher(Twist, '/human/cmd_vel', 10)

        # Pose tracking for all humans
        self.human_poses = {
            i: {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'moving': False}
            for i in (1, 2, 3, 4)
        }

        # Subscribers for all 4 humans
        self.subs = []
        for i in (1, 2, 3, 4):
            cb = self._make_human_cb(i)
            sub = self.create_subscription(Pose, f'/human_{i}/pose', cb, 10)
            self.subs.append(sub)

        # Cart odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry, '/wheel/odometry', self.odom_cb, 10
        )

        # Configurable speeds
        self.linear_speed = 0.35      # m/s
        self.strafe_speed = 0.25      # m/s
        self.angular_speed = 0.85     # rad/s

        # State tracking for cart
        self.cart_x = 0.0
        self.cart_y = 0.0
        self.running = True

        self.get_logger().info(f'Teleop ready. Active target: Human {self.human_id}')

    def _make_human_cb(self, human_idx):
        def human_cb(msg: Pose):
            self.human_poses[human_idx]['x'] = msg.position.x
            self.human_poses[human_idx]['y'] = msg.position.y
            qz = msg.orientation.z
            qw = msg.orientation.w
            self.human_poses[human_idx]['yaw'] = 2.0 * math.atan2(qz, qw)
        return human_cb

    def odom_cb(self, msg: Odometry):
        self.cart_x = msg.pose.pose.position.x
        self.cart_y = msg.pose.pose.position.y

    def switch_human(self, new_id: int):
        """Switch active human, ensuring previous human stops immediately."""
        if new_id != self.human_id and new_id in (1, 2, 3, 4):
            self.publish_vel_to(self.human_id, 0.0, 0.0, 0.0)
            self.human_poses[self.human_id]['moving'] = False
            self.human_id = new_id
            self.get_logger().info(f'Switched active control to Human {self.human_id}')

    def publish_vel_to(self, hid: int, vx: float, vy: float, wz: float):
        """Publish velocity command specifically to human <hid>."""
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        if hid in self.pubs:
            self.pubs[hid].publish(cmd)
        if hid == 1:
            self.legacy_pub.publish(cmd)

    def publish_active_vel(self, vx: float, vy: float, wz: float):
        """Publish velocity command to the currently active human."""
        self.publish_vel_to(self.human_id, vx, vy, wz)
        is_moving = abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01
        self.human_poses[self.human_id]['moving'] = is_moving


def get_key(settings, timeout=0.03):
    """Read a keypress or escape sequence non-blockingly."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # Escape or Arrow key sequence
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
                elif key in ('1', '2', '3', '4'):
                    node.switch_human(int(key))
                    target_vx = 0.0
                    target_vy = 0.0
                    target_wz = 0.0
                    last_key_time = 0.0
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

            # Hold-to-move check: if key released (>150ms ago), stop active human completely
            if now - last_key_time > KEY_TIMEOUT:
                target_vx = 0.0
                target_vy = 0.0
                target_wz = 0.0

            # Publish velocity at 20 Hz
            node.publish_active_vel(target_vx, target_vy, target_wz)

            # Refresh HUD at ~10 Hz
            if now - last_hud_time >= 0.10:
                last_hud_time = now
                h1_x = node.human_poses[1]['x']
                h1_y = node.human_poses[1]['y']
                dx = h1_x - node.cart_x
                dy = h1_y - node.cart_y
                dist_to_h1 = math.sqrt(dx * dx + dy * dy)

                active_label = f"★ [HUMAN {node.human_id} SELECTED]"
                h_summaries = []
                for i in (1, 2, 3, 4):
                    p = node.human_poses[i]
                    tag = f"H{i}" + ("*" if i == node.human_id else "")
                    mv = "M" if p['moving'] else "S"
                    h_summaries.append(f"{tag}:({p['x']:+.1f},{p['y']:+.1f}|{mv})")

                hud = (
                    f"\r{active_label} {' '.join(h_summaries)} | "
                    f"Cart:({node.cart_x:+.1f},{node.cart_y:+.1f}) DistH1:{dist_to_h1:.2f}m   "
                )
                sys.stdout.write(hud)
                sys.stdout.flush()

    except Exception as e:
        print(f"\nTeleop exception: {e}")
    finally:
        for hid in (1, 2, 3, 4):
            node.publish_vel_to(hid, 0.0, 0.0, 0.0)
        node.running = False


def main(args=None):
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
        for hid in (1, 2, 3, 4):
            node.publish_vel_to(hid, 0.0, 0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\nMulti-Human Teleoperation stopped cleanly.\n")


if __name__ == '__main__':
    main()
