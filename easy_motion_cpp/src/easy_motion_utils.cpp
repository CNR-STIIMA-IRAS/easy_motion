#include "easy_motion_cpp/easy_motion_utils.hpp"

#include <cmath>
#include <stdexcept>

namespace easy_motion_cpp
{
namespace
{

QuaternionXYZW rotation_matrix_to_quaternion(const Affine4d & affine)
{
  const double m00 = affine[0];
  const double m01 = affine[1];
  const double m02 = affine[2];
  const double m10 = affine[4];
  const double m11 = affine[5];
  const double m12 = affine[6];
  const double m20 = affine[8];
  const double m21 = affine[9];
  const double m22 = affine[10];

  const double trace = m00 + m11 + m22;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;

  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    qw = 0.25 * s;
    qx = (m21 - m12) / s;
    qy = (m02 - m20) / s;
    qz = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    qw = (m21 - m12) / s;
    qx = 0.25 * s;
    qy = (m01 + m10) / s;
    qz = (m02 + m20) / s;
  } else if (m11 > m22) {
    const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    qw = (m02 - m20) / s;
    qx = (m01 + m10) / s;
    qy = 0.25 * s;
    qz = (m12 + m21) / s;
  } else {
    const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    qw = (m10 - m01) / s;
    qx = (m02 + m20) / s;
    qy = (m12 + m21) / s;
    qz = 0.25 * s;
  }

  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm <= 0.0) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  return {qx / norm, qy / norm, qz / norm, qw / norm};
}

}  // namespace

Affine4d build_affine(const Vector3d & translation, const QuaternionXYZW & rotation_xyzw)
{
  double x = rotation_xyzw[0];
  double y = rotation_xyzw[1];
  double z = rotation_xyzw[2];
  double w = rotation_xyzw[3];

  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm <= 0.0) {
    throw std::invalid_argument("Quaternion norm must be greater than zero");
  }
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;

  Affine4d affine = {
    1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w), translation[0],
    2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w), translation[1],
    2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y), translation[2],
    0.0, 0.0, 0.0, 1.0
  };

  // Avoid returning tiny numerical residue for identity-like transforms.
  for (double & value : affine) {
    if (std::abs(value) < 1e-15) {
      value = 0.0;
    }
  }
  affine[15] = 1.0;
  return affine;
}

Affine4d transform_to_affine(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & t = transform.transform.translation;
  const auto & q = transform.transform.rotation;
  return build_affine({t.x, t.y, t.z}, {q.x, q.y, q.z, q.w});
}

Affine4d pose_stamped_to_affine(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  return build_affine({p.x, p.y, p.z}, {q.x, q.y, q.z, q.w});
}

geometry_msgs::msg::TransformStamped affine_to_transform(
  const Affine4d & affine,
  const std::string & frame_id,
  const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = frame_id;
  transform.child_frame_id = child_frame_id;
  transform.transform.translation.x = affine[3];
  transform.transform.translation.y = affine[7];
  transform.transform.translation.z = affine[11];

  const auto q = rotation_matrix_to_quaternion(affine);
  transform.transform.rotation.x = q[0];
  transform.transform.rotation.y = q[1];
  transform.transform.rotation.z = q[2];
  transform.transform.rotation.w = q[3];
  return transform;
}

geometry_msgs::msg::PoseStamped transform_to_pose_stamped(
  const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

}  // namespace easy_motion_cpp
