#pragma once

#include <memory>

#include <Eigen/Dense>

#include "riptide_control/control_law_interface.hpp"
#include "riptide_dynamics/dynamics_model_interface.hpp"

namespace riptide_control
{

/// Operational-space (Cartesian impedance) control law:
///   F   = Kp * x_err - Kd * (J * dq)          [6-D task-space wrench]
///   tau = J^T F + N (kn (q_rest - q) - dn dq) + nonlinear
/// where x_err is the world-frame EE pose error and N projects a posture task
/// into the redundant nullspace. Because the EE pose is computed from the
/// measured floating-base pose, base motion appears as x_err and is rejected —
/// this is the control law that stabilizes the EE under disturbance.
class TaskSpaceImpedance : public riptide::IControlLaw
{
public:
  bool on_configure(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging,
    std::shared_ptr<riptide::IDynamicsModel> model) override;
  Eigen::VectorXd compute(
    const riptide::RobotState & state,
    const riptide::EndEffectorTarget & target,
    double dt) override;
  void reset() override;

private:
  std::shared_ptr<riptide::IDynamicsModel> model_;
  Eigen::Matrix<double, 6, 1> kp_;
  Eigen::Matrix<double, 6, 1> kd_;
  double null_kp_{5.0};
  double null_kd_{1.0};
  double jacobian_damping_{1e-3};
  Eigen::VectorXd q_rest_;
  Eigen::VectorXd max_effort_;
};

}  // namespace riptide_control
