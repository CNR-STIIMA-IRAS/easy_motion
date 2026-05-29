#include <easy_motion_behavior_tree/leaves/get_fk.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

bool GetFK::setRequest(Request::SharedPtr & request)
{
  std::vector<double> joint_state;
  if (!getInput("joint_state", joint_state)) {
    throw BT::RuntimeError("Missing parameter [joint_state]");
  }

  request->joint_state = joint_state;
  return true;
}

BT::NodeStatus GetFK::onResponseReceived(const Response::SharedPtr & response)
{
  RCLCPP_INFO(logger(), "GetFK service response received.");

  const int code = response->result.val;
  setOutput("result_code", code);

  if (code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    setOutput("pose", response->pose);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GetFK::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "GetFK error: %d", error);
  return BT::NodeStatus::FAILURE;
}

// Plugin registration.
// The class GetFK will self register with name  "GetFK".
CreateRosNodePlugin(GetFK, "GetFK");
