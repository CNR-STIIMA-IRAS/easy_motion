#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <easy_motion_behavior_tree/bt_conversions.hpp>

#include <easy_motion_msgs/action/plan_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class PlanToPose : public BT::RosActionNode<easy_motion_msgs::action::PlanToPose>
{
public:
  PlanToPose(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
    {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose_target"),
      BT::InputPort<bool>("cartesian_motion"),
      BT::InputPort<bool>("relative_motion"),
      BT::InputPort<double>("velocity_scaling"),
      BT::InputPort<std::vector<double>>("joint_start"),
      BT::InputPort<std::string>("frame_id"),
      BT::InputPort<std::vector<double>>("position"),
      BT::InputPort<std::vector<double>>("orientation"),
      BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("trajectory"),
      BT::OutputPort<int>("result_code"),
    });
  }

  bool setGoal(Goal & goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  BT::NodeStatus onFeedback(
    const std::shared_ptr<const easy_motion_msgs::action::PlanToPose::Feedback> feedback) override;
};
