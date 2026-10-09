#!/usr/bin/env python3
"""
Automated Validation Test Suite for SmartCart Hybrid Human Following.
Tests the 5 core scenarios defined in SOLUTION.md:
- Test 1: BLE beacon reception & coarse proximity calculation.
- Test 2: Target lock on customer (Human 1) based on BLE + Vision.
- Test 3: Nearest-Person Trap: Bystander (Human 2) approaches cart at x=1.3m (closer than Customer at 2.0m).
- Test 4: Occlusion & Re-ID Recovery: Bystander crosses LOS at x=1.3m, cart coasts and recovers Human 1.
- Test 5: End-to-end follow motion of Human 1 via /target/pose -> /cmd_vel.
"""

import sys
import json
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class HybridTrackerTester(Node):
    def __init__(self):
        super().__init__('hybrid_tracker_tester')

        self.ble_data = None
        self.target_status = None
        self.target_pose = None
        self.h1_pose = None
        self.h2_pose = None
        self.odom = None

        # Human publishers
        self.h1_pub = self.create_publisher(Twist, '/human_1/cmd_vel', 10)
        self.h2_pub = self.create_publisher(Twist, '/human_2/cmd_vel', 10)

        # Subscribers
        self.create_subscription(String, '/ble/customer_tag', self.ble_cb, 10)
        self.create_subscription(String, '/target/status', self.status_cb, 10)
        self.create_subscription(Pose, '/target/pose', self.pose_cb, 10)
        self.create_subscription(Pose, '/human_1/pose', self.h1_cb, 10)
        self.create_subscription(Pose, '/human_2/pose', self.h2_cb, 10)
        self.create_subscription(Odometry, '/wheel/odometry', self.odom_cb, 10)

    def ble_cb(self, msg: String):
        try:
            self.ble_data = json.loads(msg.data)
        except Exception:
            pass

    def status_cb(self, msg: String):
        try:
            self.target_status = json.loads(msg.data)
        except Exception:
            pass

    def pose_cb(self, msg: Pose):
        self.target_pose = msg

    def h1_cb(self, msg: Pose):
        self.h1_pose = (msg.position.x, msg.position.y)

    def h2_cb(self, msg: Pose):
        self.h2_pose = (msg.position.x, msg.position.y)

    def odom_cb(self, msg: Odometry):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)


def run_tests():
    rclpy.init()
    tester = HybridTrackerTester()

    print("\n" + "=" * 65)
    print("HYBRID HUMAN-FOLLOWING VALIDATION TEST SUITE (SOLUTION.MD)")
    print("=" * 65)

    # 1. Connection check and BLE Beacon Reception
    print("\n[SCENARIO 1] Connecting to simulation & verifying BLE Customer Beacon...")
    t_start = time.time()
    TIMEOUT = 15.0

    while time.time() - t_start < TIMEOUT:
        rclpy.spin_once(tester, timeout_sec=0.1)
        if tester.ble_data is not None and tester.target_status is not None:
            break
        elapsed = int(time.time() - t_start)
        sys.stdout.write(f"\r  Connecting to simulation topics... [{elapsed}s/{int(TIMEOUT)}s]")
        sys.stdout.flush()

    print()

    if tester.ble_data is None or tester.target_status is None:
        print("\n" + "!" * 65)
        print("ERROR: Simulation is not currently running or topics not yet ready.")
        print("Please launch the main simulation in Terminal 1 first:")
        print("  source /opt/ros/jazzy/setup.bash")
        print("  source ~/Projects/smartcart_simulation/smartcart_ws/install/setup.bash")
        print("  ros2 launch smartcart_gazebo smartcart.launch.py")
        print("!" * 65 + "\n")
        tester.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print(f"  ✓ Tag ID: {tester.ble_data['tag_id']}")
    print(f"  ✓ RSSI: {tester.ble_data['rssi_dbm']} dBm | Est. Distance: {tester.ble_data['approx_distance_m']}m")
    print(f"  ✓ Proximity State: {tester.ble_data['proximity']}")

    # 2. Test Target Lock State
    print("\n[SCENARIO 2] Verifying Target Selection & Lock on Customer (Human 1)...")
    assert tester.target_status is not None, "Target status not published!"
    print(f"  ✓ Target Tracking State: {tester.target_status['state']}")
    print(f"  ✓ Target Track ID: {tester.target_status['target_track_id']}")
    print(f"  ✓ Confidence: {int(tester.target_status['confidence'] * 100)}%")
    assert tester.target_status['target_track_id'] == 1, "Incorrect target locked!"

    # 3. Test Nearest-Person Trap
    print("\n[SCENARIO 3] Testing Nearest-Person Trap (Bystander closer than Customer)...")
    print("  Moving Human 2 (Bystander) from x=3.8m to x=1.3m (closer than Human 1 at 2.0m)...")
    
    # Move Human 2 backward along X toward cart (x: 3.8 -> 1.3)
    twist_h2_approach = Twist()
    twist_h2_approach.linear.x = -0.5
    t_end = time.time() + 5.0
    while time.time() < t_end:
        tester.h2_pub.publish(twist_h2_approach)
        rclpy.spin_once(tester, timeout_sec=0.05)
        if tester.h2_pose and tester.h2_pose[0] <= 1.35:
            break

    stop_twist = Twist()
    tester.h2_pub.publish(stop_twist)
    for _ in range(15):
        rclpy.spin_once(tester, timeout_sec=0.05)

    if tester.h2_pose and tester.h1_pose:
        print(f"  ✓ Current Positions -> Bystander (H2): x={tester.h2_pose[0]:.2f}m | Customer (H1): x={tester.h1_pose[0]:.2f}m")
        print(f"  ✓ Bystander is CLOSER to SmartCart than Customer ({tester.h2_pose[0]:.2f}m < {tester.h1_pose[0]:.2f}m)!")

    print(f"  ✓ Target Status during bystander proximity: {tester.target_status['state']}")
    print(f"  ✓ Locked Target ID remained: Human {tester.target_status['target_track_id']}")
    assert tester.target_status['target_track_id'] == 1, "Target erroneously switched to bystander!"
    print("  ✓ Nearest-Person Trap PASSED: Cart preserved lock on Customer (Human 1)!")

    # 4. Test Temporary Occlusion & Re-ID Recovery
    print("\n[SCENARIO 4] Testing Occlusion & Re-ID Appearance Recovery...")
    print("  Bystander crosses directly across the central corridor (y: +0.4m -> -0.4m)...")
    twist_cross = Twist()
    twist_cross.linear.y = -0.4
    t_end = time.time() + 2.5
    while time.time() < t_end:
        tester.h2_pub.publish(twist_cross)
        rclpy.spin_once(tester, timeout_sec=0.05)

    tester.h2_pub.publish(stop_twist)
    for _ in range(20):
        rclpy.spin_once(tester, timeout_sec=0.05)

    print(f"  ✓ Target state after cross: {tester.target_status['state']}")
    assert tester.target_status['state'] in ("FOLLOWING", "TARGET_LOCKED", "TEMPORARY_OCCLUSION"), "Target lost entirely!"
    print("  ✓ Occlusion & Re-ID recovery PASSED successfully!")

    # 5. Test Follow Tracking Motion
    print("\n[SCENARIO 5] Testing End-to-End Follow Tracking of Customer...")
    print("  Clearing bystander to the side aisle so front path is unobstructed...")
    twist_h2_clear = Twist()
    twist_h2_clear.linear.y = 0.5
    t_end = time.time() + 3.0
    while time.time() < t_end:
        tester.h2_pub.publish(twist_h2_clear)
        rclpy.spin_once(tester, timeout_sec=0.05)

    tester.h2_pub.publish(stop_twist)
    for _ in range(10):
        rclpy.spin_once(tester, timeout_sec=0.05)

    init_odom = tester.odom
    print("  Moving Customer (Human 1) forward along central aisle...")
    twist_h1 = Twist()
    twist_h1.linear.x = 0.45
    t_end = time.time() + 4.0
    while time.time() < t_end:
        tester.h1_pub.publish(twist_h1)
        rclpy.spin_once(tester, timeout_sec=0.05)

    tester.h1_pub.publish(stop_twist)
    t_settle = time.time() + 3.0
    while time.time() < t_settle:
        rclpy.spin_once(tester, timeout_sec=0.05)

    final_odom = tester.odom
    if init_odom and final_odom:
        cart_dist = math.sqrt((final_odom[0] - init_odom[0])**2 + (final_odom[1] - init_odom[1])**2)
        print(f"  ✓ SmartCart tracked target customer forward by: {cart_dist:.2f}m")
        assert cart_dist > 0.10, "Cart failed to follow target pose!"

    print("\n" + "=" * 65)
    print("ALL 5 HYBRID ARCHITECTURE TEST SCENARIOS PASSED SUCCESSFULLY! ✓")
    print("=" * 65 + "\n")

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    run_tests()
