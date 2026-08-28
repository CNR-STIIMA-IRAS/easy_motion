import rclpy
from geometry_msgs.msg import PoseStamped
from easy_motion.motion_client import MotionClient

def main() -> None:
    rclpy.init()
    motion_client = MotionClient()

    # Example: move_to_pose
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "tip"
    pose_msg.pose.position.x = 0.0
    pose_msg.pose.position.y = 0.0
    pose_msg.pose.position.z = 0.1
    result = motion_client.move_to_pose(pose_msg)
    print("Move to pose result:", result)


    rclpy.shutdown()

if __name__ == '__main__':
    main()
