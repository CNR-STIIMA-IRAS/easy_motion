#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>

class ApplyTransformation : public BT::SyncActionNode
{
public:
  ApplyTransformation(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),
      BT::InputPort<std::vector<double>>(
        "translation", {0.0, 0.0, 0.0},
        "Local translation [x, y, z] in metres"),
      BT::InputPort<std::vector<double>>(
        "orientation", {0.0, 0.0, 0.0, 1.0},
        "Local rotation quaternion [x, y, z, w]"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("transformed_pose")
    };
  }

  BT::NodeStatus tick() override;
};
