#pragma once

// Explicit POD<->msg seam adapters (Layer 3). The control cores
// (riptide_control_core / riptide_dynamics / riptide_geometry) speak only Eigen
// PODs; the ROS message types are confined to this header + the controllers, so
// no ROS type ever leaks into a core. See docs/adr/0001-conan-package-taxonomy.md.

#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose.hpp"

namespace riptide_control
{

/// geometry_msgs Pose -> SE(3) POD. Returns identity-with-given-translation and
/// a normalized rotation; callers validate finiteness/quaternion norm.
inline Eigen::Isometry3d pose_from_msg(const geometry_msgs::msg::Pose & msg)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
  const Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x,
                             msg.orientation.y, msg.orientation.z);
  pose.linear() = q.normalized().toRotationMatrix();
  return pose;
}

/// SE(3) POD -> geometry_msgs Pose.
inline geometry_msgs::msg::Pose pose_to_msg(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose msg;
  const Eigen::Vector3d p = pose.translation();
  const Eigen::Quaterniond q(pose.rotation());
  msg.position.x = p.x();
  msg.position.y = p.y();
  msg.position.z = p.z();
  msg.orientation.w = q.w();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  return msg;
}

}  // namespace riptide_control
