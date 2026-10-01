#pragma once

#include <memory>

#include <Eigen/Dense>

#include "riptide_control/control_law_interface.hpp"
#include "riptide_dynamics/dynamics_model_interface.hpp"

namespace riptide_control
{

/// Operational-space LQR control law (see docs/control_theory.md §3).
///
/// Feedback-linearizes the task dynamics to a double integrator (eq. 9-10) and
/// applies the infinite-horizon LQR that minimizes
///     J = \int (z^T Q z + w^T R w) dt,   z = [e; e_dot],  e = x - x_d.
/// For the per-axis double integrator the CARE has a closed form (eq. 14):
///     Kp = sqrt(q_p / r),   Kd = sqrt( 2*sqrt(q_p/r) + q_v/r ),
/// so the OPTIMAL task-space gains follow directly from the weights (Q, R) with
/// no online Riccati solve. The commanded task acceleration
///     w = Kp .* x_err - Kd .* (J*dq)
/// is realized through the operational-space inertia:
///     tau = J^T (Lambda w) + nonlinear + N * posture,   Lambda = (J M^-1 J^T)^-1.
/// Same seam/model as TaskSpaceImpedance; swap via the `control_law` parameter.
class TaskSpaceLqr : public riptide::IControlLaw
{
public:
  bool on_configure(
    riptide::ParamSource & params,
    const riptide::Logger & log,
    std::shared_ptr<riptide::IDynamicsModel> model) override;
  Eigen::VectorXd compute(
    const riptide::RobotState & state,
    const riptide::EndEffectorTarget & target,
    double dt) override;
  void reset() override;

private:
  std::shared_ptr<riptide::IDynamicsModel> model_;

  // LQR-optimal task gains (6-D: [pos; ori]), derived from Q, R at configure.
  Eigen::Matrix<double, 6, 1> kp_;
  Eigen::Matrix<double, 6, 1> kd_;

  // Redundancy-resolving posture task (nullspace).
  double null_kp_{5.0};
  double null_kd_{1.0};
  double jacobian_damping_{1e-3};
  Eigen::VectorXd q_rest_;
  Eigen::VectorXd max_effort_;
};

}  // namespace riptide_control
