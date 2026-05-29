#pragma once

#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <easy_motion_behavior_tree/bt_conversions.hpp>

#include <easy_motion_msgs/srv/get_ik.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using GetIKSrv = easy_motion_msgs::srv::GetIK;

class GetIK : public BT::RosServiceNode<GetIKSrv>
{
public:
  explicit GetIK(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params)
  : BT::RosServiceNode<GetIKSrv>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),
        BT::InputPort<std::vector<double>>("seed"),
        BT::OutputPort<std::vector<double>>("ik_solution"),
        BT::OutputPort<int>("result_code"),
      });
  }

  bool setRequest(Request::SharedPtr & request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};
