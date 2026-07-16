#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <moveit_msgs/action/execute_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class ExecuteTrajectory : public BT::RosActionNode<moveit_msgs::action::ExecuteTrajectory>
{
public:
  ExecuteTrajectory(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<trajectory_msgs::msg::JointTrajectory>("trajectory"),
      BT::OutputPort<int>("result_code"),
    });
  }

  bool setGoal(Goal & goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult & result) override;
  BT::NodeStatus onFailure(
    BT::ActionNodeErrorCode error,
    const std::optional<WrappedResult> & result) override;
};
