#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <easy_motion_behavior_tree/bt_conversions.hpp>

#include <easy_motion_msgs/action/plan_to_joint.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class PlanToJoint : public BT::RosActionNode<easy_motion_msgs::action::PlanToJoint>
{
public:
  PlanToJoint(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
    {
      BT::InputPort<std::vector<double>>("joint_target"),
      BT::InputPort<std::vector<double>>("joint_start"),
      BT::InputPort<double>("max_velocity_scaling"),
      BT::InputPort<double>("max_acceleration_scaling"),
      BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("trajectory"),
      BT::OutputPort<int>("result_code"),
    });
  }

  bool setGoal(Goal & goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  BT::NodeStatus onFeedback(
    const std::shared_ptr<const easy_motion_msgs::action::PlanToJoint::Feedback> feedback) override;
};
