#include <memory>

#include <easy_motion_cpp/motion_client.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto client = std::make_shared<easy_motion::MotionClient>();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = 0.5;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.2;
    pose.pose.orientation.w = 1.0;

    const auto result = client->move_to_pose(pose, false);
    RCLCPP_INFO(client->get_logger(), "move_to_pose returned code: %d", result.val);

    if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_INFO(client->get_logger(), "Motion completed successfully");
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("simple_motion_client"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
