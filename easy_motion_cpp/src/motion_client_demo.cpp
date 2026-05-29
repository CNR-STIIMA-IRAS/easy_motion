#include <easy_motion_cpp/motion_client.hpp>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <iostream>
#include <memory>
#include <optional>
#include <vector>

int main(int argc, char ** argv)
{
  (void)argv;
  rclcpp::init(argc, argv);

  try {
    auto motion_client = std::make_shared<easy_motion::MotionClient>();

    std::vector<double> joint_goal{
      1.57,
      -3.0258857637684,
      1.6139817698694139,
      -1.8017236579269869,
      -1.5701870879802997,
      -0.16411033649582998,
    };

    auto [plan_joint_result, trj1] =
      motion_client->plan_to_joint(
      joint_goal,
      std::nullopt,
      0.5,
      0.5);

    std::cout << "Plan to joint result: "
              << plan_joint_result.val
              << std::endl;

    auto exec_result =
      motion_client->execute_last_planned_trajectory();

    std::cout << "Execute last planned trajectory result: "
              << exec_result.val
              << std::endl;

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "tip";

    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.position.z = 0.05;

    pose_msg.pose.orientation.w = 1.0;
    pose_msg.pose.orientation.x = 0.0;
    pose_msg.pose.orientation.y = 0.0;
    pose_msg.pose.orientation.z = 0.0;

    auto [ik_result, ik_solution] =
      motion_client->get_ik(pose_msg);

    std::cout << "IK result: "
              << ik_result.val
              << " (solution size=" << ik_solution.size() << ")"
              << std::endl;

    auto [plan_pose_result, trj2] =
      motion_client->plan_to_pose(
      pose_msg,
      std::optional<std::vector<double>>{joint_goal},
      false,
      false,
      0.5,
      0.5);

    std::cout << "Plan to pose result: "
              << plan_pose_result.val
              << std::endl;

    auto exec2 =
      motion_client->execute_trajectory(trj2);

    std::cout << "Execute trajectory result: "
              << exec2.val
              << std::endl;

  } catch (const std::exception & e) {
    std::cerr << "motion_client_demo error: "
              << e.what()
              << std::endl;

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
