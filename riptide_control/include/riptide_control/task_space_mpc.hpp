#pragma once

#include <array>
#include <memory>

#include <Eigen/Dense>

#include "riptide_control/control_law_interface.hpp"
#include "riptide_dynamics/dynamics_model_interface.hpp"

namespace riptide_control
{

/// Operational-space MPC control law (see docs/control_theory.md §4).
///
/// Feedback-linearizes the task to a double integrator (as the LQR law) and, at
/// each cycle, solves a finite-horizon constrained optimal-control problem
///     min  Σ_{k=0}^{N-1}(z_k^T Q z_k + w_k^T R w_k) + z_N^T P z_N
///     s.t. z_{k+1}=A_d z_k + B_d w_k,  |w_k| ≤ w_max
/// and applies w_0 (receding horizon). The terminal weight P is the DARE
/// solution and the terminal controller is the LQR gain, which are the standard
/// terminal ingredients that make the scheme provably stabilizing (Mayne et al.
/// 2000): the value function is a Lyapunov function. With the constraint
/// inactive the law recovers LQR; when w would saturate, the horizon lets it
/// plan ahead instead of merely clamping.
///
/// Because feedback linearization decouples the 6 task axes and the weights are
/// diagonal, the QP separates into 6 independent scalar-input QPs, each solved
/// by a warm-started projected fast-gradient (Nesterov) method — no external QP
/// solver required. w_0 is mapped to joint torque through the operational-space
/// inertia exactly as in TaskSpaceLqr.
class TaskSpaceMpc : public riptide::IControlLaw
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
  // Per-axis condensed QP: min 1/2 U^T H U + (G z0)^T U  s.t. |U| <= w_max.
  struct Axis
  {
    Eigen::MatrixXd H;     ///< N x N Hessian (constant)
    Eigen::MatrixXd G;     ///< N x 2 linear map from z0 to the gradient offset
    double L{1.0};         ///< Lipschitz constant (max eig of H)
    double mu{1.0};        ///< strong-convexity constant (min eig of H)
    double w_max{1e9};     ///< input box bound
    Eigen::VectorXd warm;  ///< N warm-start (shifted between cycles)
    // Preallocated work buffers (no per-cycle heap allocation).
    Eigen::VectorXd g, y, u_new, grad;
  };

  std::shared_ptr<riptide::IDynamicsModel> model_;
  std::array<Axis, 6> axis_;
  int N_{20};
  double T_{0.004};
  int max_iter_{40};

  // Redundancy-resolving posture task (nullspace), as in TaskSpaceLqr.
  double null_kp_{5.0};
  double null_kd_{1.0};
  double jacobian_damping_{1e-3};
  Eigen::VectorXd q_rest_;
  Eigen::VectorXd max_effort_;

  double solve_axis(Axis & ax, double e, double e_dot);
};

}  // namespace riptide_control
