#include <easy_motion_behavior_tree/leaves/get_ik.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

bool GetIK::setRequest(Request::SharedPtr & request)
{
  std::vector<double> seed;
  if (!getInput("seed", seed)) {
    seed.clear();
  }

  auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
  if (pose) {
    request->pose = pose.value();
  } else {
    geometry_msgs::msg::PoseStamped target_pose;
    if (!getInput("frame_id", target_pose.header.frame_id)) {
      throw BT::RuntimeError("Missing parameter [pose] or [frame_id]");
    }

    std::vector<double> position;
    if (!getInput("position", position) || position.size() != 3) {
      throw BT::RuntimeError("Invalid or missing parameter [position]. Expected 3 elements");
    }

    std::vector<double> orientation;
    if (!getInput("orientation", orientation) || orientation.size() != 4) {
      throw BT::RuntimeError("Invalid or missing parameter [orientation]. Expected 4 elements");
    }

    target_pose.header.stamp = node_.lock()->now();
    target_pose.pose.position.x = position[0];
    target_pose.pose.position.y = position[1];
    target_pose.pose.position.z = position[2];
    target_pose.pose.orientation.x = orientation[0];
    target_pose.pose.orientation.y = orientation[1];
    target_pose.pose.orientation.z = orientation[2];
    target_pose.pose.orientation.w = orientation[3];
    request->pose = target_pose;
  }

  request->seed = seed;
  return true;
}

BT::NodeStatus GetIK::onResponseReceived(const Response::SharedPtr & response)
{
  RCLCPP_INFO(logger(), "GetIK service response received.");

  const int code = response->result.val;
  setOutput("result_code", code);

  if (code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    setOutput("ik_solution", response->ik_solution);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GetIK::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "GetIK error: %d", error);
  return BT::NodeStatus::FAILURE;
}

// Plugin registration.
// The class GetIK will self register with name  "GetIK".
CreateRosNodePlugin(GetIK, "GetIK");
