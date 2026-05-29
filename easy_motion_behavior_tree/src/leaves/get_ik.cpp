#include <easy_motion_behavior_tree/leaves/get_ik.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

bool GetIK::setRequest(Request::SharedPtr & request)
{
  auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
  if (!pose) {
    throw BT::RuntimeError("Missing parameter [pose]");
  }

  std::vector<double> seed;
  if (!getInput("seed", seed)) {
    seed.clear();
  }

  request->pose = pose.value();
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
