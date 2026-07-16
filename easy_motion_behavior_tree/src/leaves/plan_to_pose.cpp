#include <easy_motion_behavior_tree/leaves/plan_to_pose.hpp>
#include <easy_motion_behavior_tree/moveit_error_to_string.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

PlanToPose::PlanToPose(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: RosActionNode<easy_motion_msgs::action::PlanToPose>(name, conf, params)
{
}

bool PlanToPose::setGoal(RosActionNode::Goal & goal)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "PlanToPose ticked.");

  bool cartesian_motion = false;
  if (!getInput("cartesian_motion", cartesian_motion)) {
    RCLCPP_INFO(
      node_.lock()->get_logger(),
      "Missing parameter [cartesian_motion], set it as false by default");
  }

  bool relative_motion = false;
  if (!getInput("relative_motion", relative_motion)) {
    RCLCPP_INFO(
      node_.lock()->get_logger(),
      "Missing parameter [relative_motion], set it as false by default");
  }

  double velocity_scaling = 1.0;
  if (!getInput("velocity_scaling", velocity_scaling)) {
    RCLCPP_INFO(
      node_.lock()->get_logger(),
      "Missing parameter [velocity_scaling], set it as 1.0 by default");
  }

  std::vector<double> joint_start;
  if (!getInput("joint_start", joint_start)) {
    joint_start.clear();
  }

  goal.cartesian_motion = cartesian_motion;
  goal.relative_motion = relative_motion;
  goal.velocity_scaling = velocity_scaling;
  goal.joint_start = joint_start;

  auto pose_target = getInput<geometry_msgs::msg::PoseStamped>("pose_target");
  if (pose_target) {
    goal.pose_target = pose_target.value();
    return true;
  }

  geometry_msgs::msg::PoseStamped pose;
  if (!getInput("frame_id", pose.header.frame_id)) {
    throw BT::RuntimeError("Missing parameter [frame_id]");
  }

  std::vector<double> position;
  if (!getInput("position", position) || position.size() != 3) {
    throw BT::RuntimeError("Invalid or missing parameter [position]. Expected 3 elements");
  }

  std::vector<double> orientation;
  if (!getInput("orientation", orientation) || orientation.size() != 4) {
    throw BT::RuntimeError("Invalid or missing parameter [orientation]. Expected 4 elements");
  }

  pose.header.stamp = node_.lock()->now();
  pose.pose.position.x = position[0];
  pose.pose.position.y = position[1];
  pose.pose.position.z = position[2];
  pose.pose.orientation.x = orientation[0];
  pose.pose.orientation.y = orientation[1];
  pose.pose.orientation.z = orientation[2];
  pose.pose.orientation.w = orientation[3];

  goal.pose_target = pose;
  return true;
}

BT::NodeStatus PlanToPose::onResultReceived(const RosActionNode::WrappedResult & wr)
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

BT::NodeStatus PlanToPose::onFailure(
  BT::ActionNodeErrorCode error,
  const std::optional<WrappedResult> & result)
{
  RCLCPP_ERROR(
    node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(),
    toStr(error));

  if (result && result->result) {
    const int code = result->result->result.val;
    setOutput("result_code", code);

    moveit::core::MoveItErrorCode moveit_error(code);
    RCLCPP_ERROR(
      node_.lock()->get_logger(), "%s failed with error code: %d (%s)",
      name().c_str(), code, easy_motion::moveitErrorToString(moveit_error).c_str());
  } else {
    setOutput("result_code", moveit_msgs::msg::MoveItErrorCodes::FAILURE);
  }

  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PlanToPose::onFeedback(
  const std::shared_ptr<const easy_motion_msgs::action::PlanToPose::Feedback> feedback)
{
  (void)feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class PlanToPose will self register with name  "PlanToPose".
CreateRosNodePlugin(PlanToPose, "PlanToPose");
