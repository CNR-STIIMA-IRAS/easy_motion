#include <easy_motion_behavior_tree/bt_conversions.hpp>
#include <easy_motion_behavior_tree/leaves/apply_transformation.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <cmath>

BT::NodeStatus ApplyTransformation::tick()
{
  const auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
  if (!pose) {
    throw BT::RuntimeError("ApplyTransformation: missing input [pose]: ", pose.error());
  }

  const auto translation = getInput<std::vector<double>>("translation");
  if (!translation) {
    throw BT::RuntimeError(
            "ApplyTransformation: invalid input [translation]: ", translation.error());
  }
  if (translation->size() != 3) {
    throw BT::RuntimeError("ApplyTransformation: [translation] must contain x;y;z");
  }

  const auto orientation = getInput<std::vector<double>>("orientation");
  if (!orientation) {
    throw BT::RuntimeError(
            "ApplyTransformation: invalid input [orientation]: ", orientation.error());
  }
  if (orientation->size() != 4) {
    throw BT::RuntimeError("ApplyTransformation: [orientation] must contain x;y;z;w");
  }

  const auto & input = pose->pose;
  tf2::Quaternion pose_rotation(
    input.orientation.x, input.orientation.y,
    input.orientation.z, input.orientation.w);
  tf2::Quaternion offset_rotation(
    orientation->at(0), orientation->at(1),
    orientation->at(2), orientation->at(3));

  constexpr double quaternion_tolerance = 1e-12;
  if (pose_rotation.length2() <= quaternion_tolerance) {
    throw BT::RuntimeError("ApplyTransformation: [pose] contains a zero-length quaternion");
  }
  if (offset_rotation.length2() <= quaternion_tolerance) {
    throw BT::RuntimeError("ApplyTransformation: [orientation] is a zero-length quaternion");
  }
  pose_rotation.normalize();
  offset_rotation.normalize();

  const tf2::Transform input_transform(
    pose_rotation,
    tf2::Vector3(input.position.x, input.position.y, input.position.z));
  const tf2::Transform local_offset(
    offset_rotation,
    tf2::Vector3(translation->at(0), translation->at(1), translation->at(2)));
  const tf2::Transform result = input_transform * local_offset;

  geometry_msgs::msg::PoseStamped output = *pose;
  output.pose.position.x = result.getOrigin().x();
  output.pose.position.y = result.getOrigin().y();
  output.pose.position.z = result.getOrigin().z();
  output.pose.orientation.x = result.getRotation().x();
  output.pose.orientation.y = result.getRotation().y();
  output.pose.orientation.z = result.getRotation().z();
  output.pose.orientation.w = result.getRotation().w();

  setOutput("transformed_pose", output);
  return BT::NodeStatus::SUCCESS;
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ApplyTransformation>("ApplyTransformation");
}
