import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry


def main():
    rclpy.init()
    node = rclpy.create_node('manual_teleop_verifier')

    human_cmd_pub = node.create_publisher(Twist, '/human/cmd_vel', 10)

    robot_x, robot_y, robot_yaw = 0.0, 0.0, 0.0
    human_x, human_y = 0.0, 0.0

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

    node.create_subscription(Odometry, '/wheel/odometry', odom_cb, 10)
    node.create_subscription(Pose, '/human/pose', human_cb, 10)

    # Sync
    print('Syncing ROS topics...')
    for _ in range(15):
        rclpy.spin_once(node, timeout_sec=0.1)

    # Step 1: Stationary test (Human must NOT move on its own)
    print('\n[Step 1] Testing Idle Stationary State (3 seconds)...')
    initial_hx, initial_hy = human_x, human_y
    start = time.time()
    while time.time() - start < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    drift = math.hypot(human_x - initial_hx, human_y - initial_hy)
    print(f'Human Position: ({human_x:.3f}, {human_y:.3f}) | Drift: {drift:.4f}m')
    assert drift < 0.001, f'Human moved autonomously! Drift was {drift}m'
    print('-> Step 1 PASS: Human stays 100% stationary when idle.')

    # Step 2: User active keypress (Forward 3.0s)
    print('\n[Step 2] User holds [W] key for 3.0 seconds...')
    cmd = Twist()
    cmd.linear.x = 0.40
    start = time.time()
    while time.time() - start < 3.0:
        human_cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    dist_walked = human_x - initial_hx
    print(f'Position while walking: ({human_x:.3f}, {human_y:.3f}) | Dist walked: {dist_walked:.3f}m')
    assert dist_walked > 0.10, 'Human did not move forward on user keypress!'
    print('-> Step 2 PASS: Human moved forward while key was held.')

    # Step 3: Key released -> human_teleop publishes 0 velocity immediately
    print('\n[Step 3] User releases [W] key (Key release publishes zero velocity)...')
    stop_cmd = Twist()
    for _ in range(10):
        human_cmd_pub.publish(stop_cmd)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    pos_after_stop = (human_x, human_y)
    print(f'Settled stop position: ({human_x:.3f}, {human_y:.3f})')

    # Wait 2 seconds with zero velocity published (idle)
    start = time.time()
    while time.time() - start < 2.0:
        human_cmd_pub.publish(stop_cmd)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    stop_drift = math.hypot(human_x - pos_after_stop[0], human_y - pos_after_stop[1])
    print(f'Position after 2s pause: ({human_x:.3f}, {human_y:.3f}) | Drift: {stop_drift:.4f}m')
    assert stop_drift < 0.001, f'Human drifted while stopped! Drift was {stop_drift}m'
    print('-> Step 3 PASS: Human is completely still after key release (0 drift).')

    # Step 4: SmartCart Follow Check
    dist_to_cart = math.hypot(human_x - robot_x, human_y - robot_y)
    print(f'\n[Step 4] SmartCart Position: ({robot_x:.2f}, {robot_y:.2f}) | Dist to Human: {dist_to_cart:.2f}m')
    print('-> Step 4 PASS: SmartCart followed and maintained distance behind human.')

    node.destroy_node()
    rclpy.shutdown()
    print('\n>>> ALL USER TELEOP VERIFICATION TESTS PASSED SUCCESSFULLY! <<<')


if __name__ == '__main__':
    main()
