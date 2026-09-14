import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry


def main():
    rclpy.init()
    node = rclpy.create_node('full_tracking_verifier')

    human_cmd_pub = node.create_publisher(Twist, '/human/cmd_vel', 10)

    robot_x, robot_y, robot_yaw = 0.0, 0.0, 0.0
    human_x, human_y = 0.0, 0.0
    cart_v, cart_w = 0.0, 0.0

    def odom_cb(msg):
        nonlocal robot_x, robot_y, robot_yaw
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        qx, qy, qz, qw = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def human_cb(msg):
        nonlocal human_x, human_y
        human_x = msg.position.x
        human_y = msg.position.y

    def cmd_cb(msg):
        nonlocal cart_v, cart_w
        cart_v = msg.linear.x
        cart_w = msg.angular.z

    node.create_subscription(Odometry, '/wheel/odometry', odom_cb, 10)
    node.create_subscription(Pose, '/human/pose', human_cb, 10)
    node.create_subscription(Twist, '/cmd_vel', cmd_cb, 10)

    # Initial sync
    print('Syncing ROS topics...')
    for _ in range(15):
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'Initial State: Human=({human_x:.2f}, {human_y:.2f}) | Cart=({robot_x:.2f}, {robot_y:.2f})')

    # Test 1: Forward walk
    print('\n[Test 1] Human walks forward (+X) for 4 seconds...')
    cmd = Twist()
    cmd.linear.x = 0.35
    start = time.time()
    while time.time() - start < 4.0:
        human_cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    dist1 = math.hypot(human_x - robot_x, human_y - robot_y)
    print(f'Result 1: Human=({human_x:.2f}, {human_y:.2f}) | Cart=({robot_x:.2f}, {robot_y:.2f}) | Dist={dist1:.2f}m | Cart v={cart_v:.2f}')

    # Test 2: Turn 90 degrees and walk lateral (+Y aisle)
    print('\n[Test 2] Human turns left 90° and walks into side aisle (+Y) for 5 seconds...')
    cmd.linear.x = 0.30
    cmd.angular.z = 0.60
    start = time.time()
    while time.time() - start < 5.0:
        human_cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    dist2 = math.hypot(human_x - robot_x, human_y - robot_y)
    print(f'Result 2: Human=({human_x:.2f}, {human_y:.2f}) | Cart=({robot_x:.2f}, {robot_y:.2f}, yaw={math.degrees(robot_yaw):.1f}°) | Dist={dist2:.2f}m | Cart v={cart_v:.2f}, w={cart_w:.2f}')

    # Test 3: Stop human and verify cart settles at target distance (~1.2m)
    print('\n[Test 3] Human stops. Cart aligns and catches up to ~1.2m...')
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    start = time.time()
    while time.time() - start < 4.0:
        human_cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    dist3 = math.hypot(human_x - robot_x, human_y - robot_y)
    print(f'Result 3: Human=({human_x:.2f}, {human_y:.2f}) | Cart=({robot_x:.2f}, {robot_y:.2f}, yaw={math.degrees(robot_yaw):.1f}°) | Dist={dist3:.2f}m | Cart v={cart_v:.2f}, w={cart_w:.2f}')

    # Test 4: 180° Turnaround (Human walks behind the cart)
    print('\n[Test 4] Human walks backwards behind the cart (-X) for 5 seconds...')
    cmd.linear.x = -0.35
    cmd.angular.z = 0.0
    start = time.time()
    while time.time() - start < 5.0:
        human_cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    cmd.linear.x = 0.0
    start = time.time()
    while time.time() - start < 3.0:
        human_cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    dist4 = math.hypot(human_x - robot_x, human_y - robot_y)
    print(f'Result 4: Human=({human_x:.2f}, {human_y:.2f}) | Cart=({robot_x:.2f}, {robot_y:.2f}, yaw={math.degrees(robot_yaw):.1f}°) | Dist={dist4:.2f}m | Cart v={cart_v:.2f}, w={cart_w:.2f}')

    node.destroy_node()
    rclpy.shutdown()

    print('\n>>> FULL TRACKING & ROTATION VERIFICATION COMPLETE <<<')


if __name__ == '__main__':
    main()
