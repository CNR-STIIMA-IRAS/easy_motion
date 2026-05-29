#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include <std_srvs/srv/trigger.hpp>

class TriggerService : public BT::RosServiceNode<std_srvs::srv::Trigger>
{
public:
  TriggerService(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params)
  : BT::RosServiceNode<std_srvs::srv::Trigger>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::OutputPort<bool>("success"),
      BT::OutputPort<std::string>("message")
    });
  }

  bool setRequest(Request::SharedPtr & request) override
  {
    (void)request;
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override
  {
    setOutput("success", response->success);
    setOutput("message", response->message);

    if (response->success) {
      RCLCPP_INFO(logger(), "Trigger service succeeded: %s", response->message.c_str());
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(logger(), "Trigger service failed: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "Trigger service error: %s", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};