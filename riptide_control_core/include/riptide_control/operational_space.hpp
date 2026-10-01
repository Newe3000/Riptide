#pragma once

#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "riptide_dynamics/dynamics_model_interface.hpp"
#include "riptide_geometry/spatial.hpp"

namespace riptide_control
{

/// The pure SE(3) task-pose error (x_d - x) lives in the eigen-only
/// riptide_geometry leaf; re-exported here so the task-space laws call it
/// unqualified.
using riptide_geometry::task_pose_error;

/// Map a desired task-space acceleration `a_des` (6-D, world) to joint torques
/// through the operational-space inertia (Khatib's OSC):
///     Lambda = (J M^-1 J^T)^-1,   tau = J^T (Lambda a_des)
///            + N (kp (q_rest - q) - kd dq)   [posture]  + nonlinear,
/// where N = I - J^T (Lambda J M^-1) is the DYNAMICALLY-CONSISTENT nullspace
/// projector, so the posture task adds no task-space acceleration. Clamped to
/// per-joint torque limits. The model must already be update()'d this cycle.
///
/// This single mapping is shared by the impedance, LQR and MPC laws — they
/// differ only in how they produce `a_des` (hand-set PD, CARE-optimal PD, or a
/// receding-horizon QP), which is exactly the point of the control zoo.
inline Eigen::VectorXd operational_space_torque(
  const riptide::IDynamicsModel & model,
  const Eigen::Matrix<double, 6, 1> & a_des,
  const Eigen::VectorXd & q, const Eigen::VectorXd & dq,
  const Eigen::VectorXd & q_rest, double null_kp, double null_kd,
  double damping, const Eigen::VectorXd & max_effort)
{
  const Eigen::MatrixXd J = model.jacobian("ee");        // 6 x n (world)
  const Eigen::MatrixXd & M = model.massMatrix();         // n x n
  const Eigen::Index n = J.cols();

  const Eigen::MatrixXd Minv = M.inverse();
  const Eigen::MatrixXd Lambda =
    (J * Minv * J.transpose() + damping * Eigen::MatrixXd::Identity(6, 6)).inverse();

  Eigen::VectorXd tau = J.transpose() * (Lambda * a_des);

  const Eigen::MatrixXd Jbar_T = Lambda * J * Minv;       // 6 x n
  const Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n, n) - J.transpose() * Jbar_T;
  if (q_rest.size() == n)
  {
    tau += N * (null_kp * (q_rest - q) - null_kd * dq);
  }

  tau += model.nonlinear();                               // C(q,dq)dq + g (g~0)

  for (Eigen::Index i = 0; i < n; ++i)
  {
    const double lim = (i < max_effort.size()) ? max_effort[i] : 1e9;
    tau[i] = std::clamp(tau[i], -lim, lim);
  }
  return tau;
}

}  // namespace riptide_control
