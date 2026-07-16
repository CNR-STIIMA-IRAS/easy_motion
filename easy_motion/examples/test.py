#!/usr/bin/env python3

import rclpy

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes

from easy_motion.motion_client import MotionClient


def main():
    rclpy.init()

    client = MotionClient(
        plan_to_pose_action_name="plan_to_pose",
        execute_trajectory_action_name="execute_trajectory",
    )

    pose = PoseStamped()
    pose.header.stamp = client.get_clock().now().to_msg()
    pose.header.frame_id = "tip"

    # Movimento relativo: +10 cm lungo Z di super_tip
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.1

    # Rotazione relativa nulla
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0

    result, trajectory = client.plan_to_pose(
        pose=pose,
        cartesian_motion=True,
        relative_motion=True,
        velocity_scaling=0.3,
    )

    print(f"Plan result: {result.val}")

    if result.val != MoveItErrorCodes.SUCCESS:
        print("Planning failed")
        client.destroy_node()
        rclpy.shutdown()
        return

    print(f"Trajectory points: {len(trajectory.points)}")
    print(f"Trajectory: {trajectory}")

    result = client.execute_trajectory(trajectory)

    print(f"Execute result: {result.val}")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
