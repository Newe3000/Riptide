#pragma once

#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "riptide_dynamics/robot_state.hpp"

namespace riptide
{

/// Whole-body dynamics oracle used by the model-based control laws (impedance,
/// LQR, MPC). A control law owns one of these and refreshes it via update();
/// this keeps physics out of the ros2_control I/O path (see PROJECT_PLAN §2/§6).
///
/// Backends (Phase 2): PinocchioModel (primary), MujocoModel (oracle/cross-check).
class IDynamicsModel
{
public:
  virtual ~IDynamicsModel() = default;

  /// Refresh the model from the latest state (call once per cycle).
  virtual void update(const RobotState & state) = 0;

  virtual const Eigen::MatrixXd & massMatrix() const = 0;   ///< M(q)
  virtual const Eigen::VectorXd & nonlinear() const = 0;    ///< C(q,dq)dq + g(q)

  /// Geometric Jacobian of `frame` w.r.t. the generalized coordinates.
  virtual Eigen::MatrixXd jacobian(const std::string & frame) const = 0;

  /// Pose of `frame` in the world frame.
  virtual Eigen::Isometry3d framePose(const std::string & frame) const = 0;

  /// Modeled hydrodynamic generalized forces (added mass / drag / buoyancy),
  /// so a control law can compensate the water it is fighting. Kept separate
  /// from what the simulator applies, to study model mismatch deliberately.
  virtual Eigen::VectorXd hydroForces(const RobotState & state) const = 0;
};

}  // namespace riptide
