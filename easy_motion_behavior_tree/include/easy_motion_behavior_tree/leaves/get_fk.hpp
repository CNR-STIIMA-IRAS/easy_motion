#pragma once

#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <easy_motion_behavior_tree/bt_conversions.hpp>

#include <easy_motion_msgs/srv/get_fk.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using GetFKSrv = easy_motion_msgs::srv::GetFK;

class GetFK : public BT::RosServiceNode<GetFKSrv>
{
public:
  explicit GetFK(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params)
  : BT::RosServiceNode<GetFKSrv>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::vector<double>>("joint_state"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose"),
        BT::OutputPort<int>("result_code"),
      });
  }

  bool setRequest(Request::SharedPtr & request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};
