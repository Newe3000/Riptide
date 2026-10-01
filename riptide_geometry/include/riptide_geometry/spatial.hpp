#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace riptide_geometry
{

/// World-frame 6-D task pose error x_d - x: position difference + the axis-angle
/// vector of the orientation error R_d R^T. Shared by every task-space control law.
inline Eigen::Matrix<double, 6, 1> task_pose_error(
  const Eigen::Isometry3d & X, const Eigen::Isometry3d & X_d)
{
  Eigen::Matrix<double, 6, 1> e;
  e.head<3>() = X_d.translation() - X.translation();
  const Eigen::Matrix3d R_err = X_d.rotation() * X.rotation().transpose();
  const Eigen::AngleAxisd aa(R_err);
  e.tail<3>() = aa.angle() * aa.axis();
  return e;
}

}  // namespace riptide_geometry
