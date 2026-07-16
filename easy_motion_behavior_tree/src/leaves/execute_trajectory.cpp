#include <easy_motion_behavior_tree/leaves/execute_trajectory.hpp>
#include <easy_motion_behavior_tree/moveit_error_to_string.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

ExecuteTrajectory::ExecuteTrajectory(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: RosActionNode<moveit_msgs::action::ExecuteTrajectory>(name, conf, params)
{
}

bool ExecuteTrajectory::setGoal(RosActionNode::Goal & goal)
{
  auto trajectory = getInput<trajectory_msgs::msg::JointTrajectory>("trajectory");
  if (!trajectory) {
    throw BT::RuntimeError("Missing parameter [trajectory]");
  }

  goal.trajectory.joint_trajectory = trajectory.value();
  return true;
}

BT::NodeStatus ExecuteTrajectory::onResultReceived(const WrappedResult & result)
{
  const int code = result.result->error_code.val;
  setOutput("result_code", code);

  if (code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    return BT::NodeStatus::SUCCESS;
  }

  moveit::core::MoveItErrorCode moveit_error(code);
  RCLCPP_ERROR(
    node_.lock()->get_logger(), "%s failed with error code: %d (%s)",
    name().c_str(), code, easy_motion::moveitErrorToString(moveit_error).c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ExecuteTrajectory::onFailure(
  BT::ActionNodeErrorCode error,
  const std::optional<WrappedResult> & result)
{
  RCLCPP_ERROR(
    node_.lock()->get_logger(), "%s: onFailure with error: %s",
    name().c_str(), toStr(error));

  if (result && result->result) {
    setOutput("result_code", result->result->error_code.val);
  } else {
    setOutput("result_code", moveit_msgs::msg::MoveItErrorCodes::FAILURE);
  }

  return BT::NodeStatus::FAILURE;
}

CreateRosNodePlugin(ExecuteTrajectory, "ExecuteTrajectory");
