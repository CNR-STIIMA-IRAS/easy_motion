#include "easy_motion_cpp/motion_client.hpp"

#include <rclcpp/version.h>

#include <stdexcept>

#if RCLCPP_VERSION_MAJOR >= 28
#define EASY_MOTION_ROS2_JAZZY
#endif

namespace easy_motion
{

MotionClient::MotionClient(
  const std::string & move_to_pose_action_name,
  const std::string & move_to_joint_action_name,
  const std::string & plan_to_pose_action_name,
  const std::string & plan_to_joint_action_name,
  const std::string & execute_trajectory_action_name,
  const std::string & gripper_action_name,
  rclcpp::NodeOptions options)
: Node("motion_client_node", options.use_global_arguments(false))
{
  this->declare_parameter<std::string>("move_to_pose_action_name", move_to_pose_action_name);
  this->declare_parameter<std::string>("move_to_joint_action_name", move_to_joint_action_name);
  this->declare_parameter<std::string>("plan_to_pose_action_name", plan_to_pose_action_name);
  this->declare_parameter<std::string>("plan_to_joint_action_name", plan_to_joint_action_name);
  this->declare_parameter<std::string>(
    "execute_trajectory_action_name",
    execute_trajectory_action_name);
  this->declare_parameter<std::string>("gripper_action_name", gripper_action_name);

  const auto move_to_pose_name = this->get_parameter("move_to_pose_action_name").as_string();
  const auto move_to_joint_name = this->get_parameter("move_to_joint_action_name").as_string();
  const auto plan_to_pose_name = this->get_parameter("plan_to_pose_action_name").as_string();
  const auto plan_to_joint_name = this->get_parameter("plan_to_joint_action_name").as_string();
  const auto execute_trajectory_name =
    this->get_parameter("execute_trajectory_action_name").as_string();
  const auto gripper_name = this->get_parameter("gripper_action_name").as_string();

  move_to_pose_client_ = rclcpp_action::create_client<MoveToPose>(this, move_to_pose_name);
  move_to_joint_client_ = rclcpp_action::create_client<MoveToJoint>(this, move_to_joint_name);
  plan_to_pose_client_ = rclcpp_action::create_client<PlanToPose>(this, plan_to_pose_name);
  plan_to_joint_client_ = rclcpp_action::create_client<PlanToJoint>(this, plan_to_joint_name);

  execute_trajectory_client_ =
    rclcpp_action::create_client<ExecuteTrajectory>(this, execute_trajectory_name);

  gripper_client_ = rclcpp_action::create_client<GripperCommand>(this, gripper_name);

  attach_object_client_ =
    this->create_client<easy_motion_msgs::srv::AttachObject>("attach_object");

  detach_object_client_ =
    this->create_client<easy_motion_msgs::srv::DetachObject>("detach_object");

  get_ik_client_ =
    this->create_client<easy_motion_msgs::srv::GetIK>("get_ik");

  get_fk_client_ =
    this->create_client<easy_motion_msgs::srv::GetFK>("get_fk");

  const auto constructor_wait = std::chrono::seconds(kConstructorWaitSec);

  if (!move_to_pose_client_->wait_for_action_server(constructor_wait)) {
    throw std::runtime_error("MoveToPose action server not available");
  }

  if (!move_to_joint_client_->wait_for_action_server(constructor_wait)) {
    throw std::runtime_error("MoveToJoint action server not available");
  }

  if (!plan_to_pose_client_->wait_for_action_server(constructor_wait)) {
    throw std::runtime_error("PlanToPose action server not available");
  }

  if (!plan_to_joint_client_->wait_for_action_server(constructor_wait)) {
    throw std::runtime_error("PlanToJoint action server not available");
  }

  if (!execute_trajectory_client_->wait_for_action_server(constructor_wait)) {
    throw std::runtime_error("ExecuteTrajectory action server not available");
  }

  if (!attach_object_client_->wait_for_service(constructor_wait)) {
    throw std::runtime_error("AttachObject service not available");
  }

  if (!detach_object_client_->wait_for_service(constructor_wait)) {
    throw std::runtime_error("DetachObject service not available");
  }

  if (!gripper_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(
      this->get_logger(),
      "Gripper action server '%s' not found",
      gripper_name.c_str());
  }
}

moveit_msgs::msg::MoveItErrorCodes MotionClient::move_to_pose(
  const geometry_msgs::msg::PoseStamped & pose,
  bool cartesian_motion,
  bool relative_motion,
  double velocity_scaling)
{
  MoveToPose::Goal goal;
  goal.pose_target = pose;
  goal.cartesian_motion = cartesian_motion;
  goal.relative_motion = relative_motion;
  goal.velocity_scaling = velocity_scaling;

  const auto wrapped_result =
    send_action_goal<MoveToPose>(move_to_pose_client_, goal, "move_to_pose");

  return wrapped_result.result->result;
}

moveit_msgs::msg::MoveItErrorCodes MotionClient::move_to_joint(
  const std::vector<double> & joint_positions,
  double velocity_scaling)
{
  MoveToJoint::Goal goal;
  goal.joint_target = joint_positions;
  goal.velocity_scaling = velocity_scaling;

  const auto wrapped_result =
    send_action_goal<MoveToJoint>(move_to_joint_client_, goal, "move_to_joint");

  return wrapped_result.result->result;
}

std::pair<moveit_msgs::msg::MoveItErrorCodes, trajectory_msgs::msg::JointTrajectory>
MotionClient::plan_to_pose(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::optional<std::vector<double>> & joint_start,
  bool cartesian_motion,
  bool relative_motion,
  double velocity_scaling)
{
  PlanToPose::Goal goal;
  goal.pose_target = pose;
  goal.cartesian_motion = cartesian_motion;
  goal.relative_motion = relative_motion;
  goal.velocity_scaling = velocity_scaling;

  if (joint_start.has_value()) {
    goal.joint_start = joint_start.value();
  }

  const auto wrapped_result =
    send_action_goal<PlanToPose>(plan_to_pose_client_, goal, "plan_to_pose");

  if (wrapped_result.result->result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    last_planned_trj_ = wrapped_result.result->trajectory;
  }

  return {wrapped_result.result->result, wrapped_result.result->trajectory};
}

std::pair<moveit_msgs::msg::MoveItErrorCodes, trajectory_msgs::msg::JointTrajectory>
MotionClient::plan_to_joint(
  const std::vector<double> & joint_target,
  const std::optional<std::vector<double>> & joint_start,
  double velocity_scaling)
{
  PlanToJoint::Goal goal;
  goal.joint_target = joint_target;
  goal.velocity_scaling = velocity_scaling;

  if (joint_start.has_value()) {
    goal.joint_start = joint_start.value();
  }

  const auto wrapped_result =
    send_action_goal<PlanToJoint>(plan_to_joint_client_, goal, "plan_to_joint");

  if (wrapped_result.result->result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    last_planned_trj_ = wrapped_result.result->trajectory;
  }

  return {wrapped_result.result->result, wrapped_result.result->trajectory};
}

moveit_msgs::msg::MoveItErrorCodes MotionClient::execute_last_planned_trajectory()
{
  if (!last_planned_trj_.has_value()) {
    throw std::runtime_error("No existing trajectory available");
  }

  return execute_trajectory(last_planned_trj_.value());
}

moveit_msgs::msg::MoveItErrorCodes MotionClient::execute_trajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<std::string> & controller_names)
{
  ExecuteTrajectory::Goal goal;
  goal.trajectory.joint_trajectory = trajectory;

#ifdef EASY_MOTION_ROS2_JAZZY
  goal.controller_names = controller_names;
#else
  if (!controller_names.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "controller_names ignored: ExecuteTrajectory does not support it in this ROS 2 distro");
  }
#endif

  const auto wrapped_result =
    send_action_goal<ExecuteTrajectory>(
    execute_trajectory_client_,
    goal,
    "execute_trajectory");

  return wrapped_result.result->error_code;
}

bool MotionClient::attach_object(
  const std::string & object_id,
  const std::string & target_frame_id)
{
  if (!attach_object_client_->wait_for_service(std::chrono::seconds(kCallWaitSec))) {
    throw std::runtime_error("AttachObject service not available");
  }

  auto request =
    std::make_shared<easy_motion_msgs::srv::AttachObject::Request>();

  request->object_id = object_id;
  request->target_frame_id = target_frame_id;

  auto future = attach_object_client_->async_send_request(request);

  spin_until_complete(future, "Calling attach_object");

  return future.get()->success;
}

bool MotionClient::detach_object(const std::string & object_id)
{
  if (!detach_object_client_->wait_for_service(std::chrono::seconds(kCallWaitSec))) {
    throw std::runtime_error("DetachObject service not available");
  }

  auto request =
    std::make_shared<easy_motion_msgs::srv::DetachObject::Request>();

  request->object_id = object_id;

  auto future = detach_object_client_->async_send_request(request);

  spin_until_complete(future, "Calling detach_object");

  return future.get()->success;
}

std::pair<moveit_msgs::msg::MoveItErrorCodes, std::vector<double>>
MotionClient::get_ik(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::optional<std::vector<double>> & seed)
{
  if (!get_ik_client_->wait_for_service(std::chrono::seconds(kCallWaitSec))) {
    throw std::runtime_error("GetIK service not available");
  }

  auto request =
    std::make_shared<easy_motion_msgs::srv::GetIK::Request>();

  request->pose = pose;

  if (seed.has_value()) {
    request->seed = seed.value();
  }

  auto future = get_ik_client_->async_send_request(request);

  spin_until_complete(future, "Calling get_ik");

  const auto response = future.get();

  return {response->result, response->ik_solution};
}

std::pair<moveit_msgs::msg::MoveItErrorCodes, geometry_msgs::msg::PoseStamped>
MotionClient::get_fk(const std::vector<double> & joint_state)
{
  if (!get_fk_client_->wait_for_service(std::chrono::seconds(kCallWaitSec))) {
    throw std::runtime_error("GetFK service not available");
  }

  auto request =
    std::make_shared<easy_motion_msgs::srv::GetFK::Request>();

  request->joint_state = joint_state;

  auto future = get_fk_client_->async_send_request(request);

  spin_until_complete(future, "Calling get_fk");

  const auto response = future.get();

  return {response->result, response->pose};
}

std::pair<bool, bool> MotionClient::gripper_command(
  double position,
  double max_effort)
{
  GripperCommand::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = max_effort;

  const auto wrapped_result =
    send_action_goal<GripperCommand>(gripper_client_, goal, "gripper_command");

  return {wrapped_result.result->reached_goal, wrapped_result.result->stalled};
}

bool MotionClient::has_last_planned_trajectory() const
{
  return last_planned_trj_.has_value();
}

const trajectory_msgs::msg::JointTrajectory &
MotionClient::last_planned_trajectory() const
{
  if (!last_planned_trj_.has_value()) {
    throw std::runtime_error("No existing trajectory available");
  }

  return last_planned_trj_.value();
}

}  // namespace easy_motion
