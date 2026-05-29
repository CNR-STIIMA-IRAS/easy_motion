#include <easy_motion_behavior_tree/leaves/plan_to_joint.hpp>
#include <easy_motion_behavior_tree/moveit_error_to_string.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

PlanToJoint::PlanToJoint(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: RosActionNode<easy_motion_msgs::action::PlanToJoint>(name, conf, params)
{
}

bool PlanToJoint::setGoal(RosActionNode::Goal & goal)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "PlanToJoint ticked.");

  auto joint_target = getInput<std::vector<double>>("joint_target");
  if (!joint_target) {
    throw BT::RuntimeError("Missing parameter [joint_target]");
  }

  std::vector<double> joint_start;
  if (!getInput("joint_start", joint_start)) {
    joint_start.clear();
  }

  double max_velocity_scaling = 1.0;
  if (!getInput("max_velocity_scaling", max_velocity_scaling)) {
    RCLCPP_INFO(
      node_.lock()->get_logger(),
      "Missing parameter [max_velocity_scaling], set it as 1.0 by default");
  }

  double max_acceleration_scaling = 1.0;
  if (!getInput("max_acceleration_scaling", max_acceleration_scaling)) {
    RCLCPP_INFO(
      node_.lock()->get_logger(),
      "Missing parameter [max_acceleration_scaling], set it as 1.0 by default");
  }

  goal.joint_target = joint_target.value();
  goal.joint_start = joint_start;
  goal.max_velocity_scaling = max_velocity_scaling;
  goal.max_acceleration_scaling = max_acceleration_scaling;

  return true;
}

BT::NodeStatus PlanToJoint::onResultReceived(const RosActionNode::WrappedResult & wr)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "%s: onResultReceived", name().c_str());
  const int code = (wr.result->result).val;
  setOutput("result_code", code);

  if (code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    moveit::core::MoveItErrorCode error(code);
    RCLCPP_INFO(
      node_.lock()->get_logger(), "%s failed with error code: %d (%s)",
      name().c_str(), code, easy_motion::moveitErrorToString(error).c_str());
    return BT::NodeStatus::FAILURE;
  }

  setOutput("trajectory", wr.result->trajectory);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PlanToJoint::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(
    node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(),
    toStr(error));
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PlanToJoint::onFeedback(
  const std::shared_ptr<const easy_motion_msgs::action::PlanToJoint::Feedback> feedback)
{
  (void)feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class PlanToJoint will self register with name  "PlanToJoint".
CreateRosNodePlugin(PlanToJoint, "PlanToJoint");
