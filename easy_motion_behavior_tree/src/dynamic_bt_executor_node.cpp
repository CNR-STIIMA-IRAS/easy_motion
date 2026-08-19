#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <behaviortree_ros2/tree_execution_server.hpp>


class EasyMotionTreeExecutor : public BT::TreeExecutionServer
{
public:
  explicit EasyMotionTreeExecutor(const rclcpp::NodeOptions & options)
  : BT::TreeExecutionServer(options)
  {
    // Register plugins and nodes immediately.
    executeRegistration();

    RCLCPP_INFO(
      node()->get_logger(),
      "EasyMotion dynamic BT executor ready");
  }

protected:
  bool onGoalReceived(
    const std::string & tree_name,
    const std::string & payload) override
  {
    if (tree_name.empty()) {
      RCLCPP_ERROR(
        node()->get_logger(),
        "Empty tree name");
      return false;
    }

    if (payload.empty()) {
      RCLCPP_ERROR(
        node()->get_logger(),
        "Empty XML payload");
      return false;
    }

    try {
      // Clear the previous tree definitions. Registered node types and plugins
      // remain available in the factory.
      factory().clearRegisteredBehaviorTrees();

      // The payload contains the complete XML document.
      factory().registerBehaviorTreeFromText(payload);

      const auto registered_trees =
        factory().registeredBehaviorTrees();

      const auto found =
        std::find(
        registered_trees.begin(),
        registered_trees.end(),
        tree_name);

      if (found == registered_trees.end()) {
        RCLCPP_ERROR(
          node()->get_logger(),
          "Tree [%s] not found in received XML",
          tree_name.c_str());

        return false;
      }

      RCLCPP_INFO(
        node()->get_logger(),
        "Tree [%s] accepted",
        tree_name.c_str());

      return true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node()->get_logger(),
        "Invalid Behavior Tree XML: %s",
        e.what());

      return false;
    }
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  auto server =
    std::make_shared<EasyMotionTreeExecutor>(options);

  // A finite timeout prevents the executor from deadlocking when publishers or
  // subscribers are removed while the node is spinning.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));

  executor.add_node(server->node());

  executor.spin();
  executor.remove_node(server->node());

  rclcpp::shutdown();

  return 0;
}
