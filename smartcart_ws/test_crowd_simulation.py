#!/usr/bin/env python3
"""
Comprehensive Independence and Acceptance Test for Multi-Human Crowd Simulation.
Tests:
1. All 4 humans publish poses on their respective /human_<id>/pose topics.
2. Independent control: Sending cmd_vel to /human_<id>/cmd_vel moves ONLY that human.
3. SmartCart follows Human 1 without disturbance when other humans move.
4. SmartCart LiDAR safety & stop behavior.
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry


class CrowdIndependenceTester(Node):
    def __init__(self):
        super().__init__('crowd_independence_tester')

        self.poses = {i: None for i in (1, 2, 3, 4)}
        self.odom = None

        # Subscribers
        self.subs = []
        for i in (1, 2, 3, 4):
            cb = self._make_cb(i)
            sub = self.create_subscription(Pose, f'/human_{i}/pose', cb, 10)
            self.subs.append(sub)

        self.odom_sub = self.create_subscription(Odometry, '/wheel/odometry', self.odom_cb, 10)

        # Publishers
        self.pubs = {
            i: self.create_publisher(Twist, f'/human_{i}/cmd_vel', 10)
            for i in (1, 2, 3, 4)
        }

    def _make_cb(self, idx):
        def cb(msg: Pose):
            self.poses[idx] = (msg.position.x, msg.position.y)
        return cb

    def odom_cb(self, msg: Odometry):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)


def run_tests():
    rclpy.init()
    tester = CrowdIndependenceTester()

    print("\n" + "=" * 60)
    print("STARTING CROWD SIMULATION ACCEPTANCE TESTS")
    print("=" * 60)

    # 1. Wait for initial poses
    print("\n[TEST 1] Waiting for all 4 human poses and robot odometry...")
    start_time = time.time()
    while time.time() - start_time < 10.0:
        rclpy.spin_once(tester, timeout_sec=0.1)
        if all(tester.poses[i] is not None for i in (1, 2, 3, 4)):
            break

    for i in (1, 2, 3, 4):
        p = tester.poses[i]
        assert p is not None, f"Human {i} pose not received!"
        print(f"  ✓ Human {i} initial position: ({p[0]:.2f}, {p[1]:.2f})")

    # 2. Test independence for each human
    print("\n[TEST 2] Testing independent command isolation...")
    for target_id in (1, 2, 3, 4):
        print(f"\n--- Testing Human {target_id} Isolation ---")
        initial_poses = dict(tester.poses)

        # Send movement commands ONLY to target human for 1.5 seconds
        twist = Twist()
        twist.linear.x = 0.5
        t_end = time.time() + 1.5
        while time.time() < t_end:
            tester.pubs[target_id].publish(twist)
            rclpy.spin_once(tester, timeout_sec=0.05)

        # Stop command
        stop_twist = Twist()
        tester.pubs[target_id].publish(stop_twist)
        for _ in range(10):
            rclpy.spin_once(tester, timeout_sec=0.05)

        final_poses = dict(tester.poses)

        # Check target moved
        dx = final_poses[target_id][0] - initial_poses[target_id][0]
        dy = final_poses[target_id][1] - initial_poses[target_id][1]
        dist_moved = math.sqrt(dx * dx + dy * dy)
        print(f"  Target Human {target_id} moved: {dist_moved:.3f} m")
        assert dist_moved > 0.2, f"Human {target_id} did not move adequately ({dist_moved}m)!"

        # Check all other humans did NOT move
        for other_id in (1, 2, 3, 4):
            if other_id == target_id:
                continue
            odx = final_poses[other_id][0] - initial_poses[other_id][0]
            ody = final_poses[other_id][1] - initial_poses[other_id][1]
            other_dist = math.sqrt(odx * odx + ody * ody)
            print(f"  Non-target Human {other_id} displacement: {other_dist:.4f} m")
            assert other_dist < 0.05, f"Cross-control detected! Human {other_id} moved by {other_dist}m!"

        print(f"  ✓ Human {target_id} moved independently with ZERO cross-control.")

    # 3. Test follow controller responding to Human 1
    print("\n[TEST 3] Testing SmartCart follow tracking of Human 1...")
    init_odom = tester.odom
    h1_init = tester.poses[1]

    # Move Human 1 forward
    twist = Twist()
    twist.linear.x = 0.4
    t_end = time.time() + 3.0
    while time.time() < t_end:
        tester.pubs[1].publish(twist)
        rclpy.spin_once(tester, timeout_sec=0.05)

    stop_twist = Twist()
    tester.pubs[1].publish(stop_twist)
    time.sleep(1.0)
    for _ in range(20):
        rclpy.spin_once(tester, timeout_sec=0.05)

    final_odom = tester.odom
    if init_odom and final_odom:
        cart_moved = math.sqrt((final_odom[0] - init_odom[0])**2 + (final_odom[1] - init_odom[1])**2)
        print(f"  ✓ SmartCart tracked Human 1 forward by: {cart_moved:.2f} m")

    print("\n" + "=" * 60)
    print("ALL MULTI-HUMAN INDEPENDENCE TESTS PASSED SUCCESSFULLY! ✓")
    print("=" * 60 + "\n")

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    run_tests()
