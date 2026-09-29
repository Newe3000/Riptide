#pragma once

#include <map>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace riptide
{

/// Framework-agnostic snapshot of the robot, assembled once per control cycle
/// by the host controller from ros2_control state interfaces. Control laws see
/// only this struct — never raw interfaces or simulator handles.
struct RobotState
{
  double time{0.0};

  // Arm joint space (order fixed by the host controller's joint list).
  Eigen::VectorXd q;   ///< joint positions
  Eigen::VectorXd dq;  ///< joint velocities

  // Floating base (world <- base). Identity/zero while the base has no free
  // joint (world-fixed).
  Eigen::Isometry3d base_pose{Eigen::Isometry3d::Identity()};
  Eigen::Matrix<double, 6, 1> base_twist{Eigen::Matrix<double, 6, 1>::Zero()};

  Eigen::Isometry3d ee_pose{Eigen::Isometry3d::Identity()};  ///< convenience

  /// Optional sensors keyed by logical name (e.g. "ee_ft", "dvl", "imu").
  /// Lets new sensors reach control laws without an ABI change.
  std::map<std::string, Eigen::VectorXd> extra;
};

/// Desired end-effector state expressed in the world frame.
struct EndEffectorTarget
{
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  Eigen::Matrix<double, 6, 1> twist{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> accel{Eigen::Matrix<double, 6, 1>::Zero()};
};

}  // namespace riptide
