#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <control_msgs/action/parallel_gripper_command.hpp>


class GripperCommand : public BT::RosActionNode<control_msgs::action::ParallelGripperCommand>
{
public:
  GripperCommand(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
    {
      BT::InputPort<double>("position"),
      BT::InputPort<double>("max_effort"),
      BT::InputPort<std::string>("joint", "robotiq_hande_left_finger_joint"),
    }
    );
  }

  bool setGoal(Goal & goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  BT::NodeStatus onFeedback(
    const std::shared_ptr<const control_msgs::action::ParallelGripperCommand::Feedback> feedback) override;

};
