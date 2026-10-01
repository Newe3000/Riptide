#include "riptide_control/template_control_law.hpp"

#include <vector>

#include "riptide_control/operational_space.hpp"   // task_pose_error + operational_space_torque

namespace riptide_control
{
namespace
{
// Declare-on-first-use parameter helper (same pattern as the other laws).
template <typename T>
T get_or_declare(riptide::ParamSource & params, const std::string & name, const T & def)
{
  return params.declare(name, def);
}

Eigen::VectorXd to_vec(const std::vector<double> & v)
{
  return Eigen::Map<const Eigen::VectorXd>(v.data(), static_cast<Eigen::Index>(v.size()));
}
}  // namespace

// One-time setup: read your parameters and capture the dynamics model. Runs on
// controller (re)configure. Parameters live under the "template." namespace in
// riptide_controllers.yaml (or wherever the host controller reads params from).
bool TemplateControlLaw::on_configure(
  riptide::ParamSource & params,
  const riptide::Logger & log,
  std::shared_ptr<riptide::IDynamicsModel> model)
{
  model_ = std::move(model);

  const auto kp_pos = get_or_declare<std::vector<double>>(params, "template.kp_pos", {800.0, 800.0, 800.0});
  const auto kp_ori = get_or_declare<std::vector<double>>(params, "template.kp_ori", {300.0, 300.0, 300.0});
  const auto kd_pos = get_or_declare<std::vector<double>>(params, "template.kd_pos", {80.0, 80.0, 80.0});
  const auto kd_ori = get_or_declare<std::vector<double>>(params, "template.kd_ori", {35.0, 35.0, 35.0});
  kp_ << kp_pos[0], kp_pos[1], kp_pos[2], kp_ori[0], kp_ori[1], kp_ori[2];
  kd_ << kd_pos[0], kd_pos[1], kd_pos[2], kd_ori[0], kd_ori[1], kd_ori[2];

  null_kp_ = get_or_declare<double>(params, "template.null_kp", 8.0);
  null_kd_ = get_or_declare<double>(params, "template.null_kd", 2.0);
  jacobian_damping_ = get_or_declare<double>(params, "template.jacobian_damping", 1e-3);
  q_rest_ = to_vec(get_or_declare<std::vector<double>>(
    params, "template.q_rest", {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}));
  max_effort_ = to_vec(get_or_declare<std::vector<double>>(
    params, "template.max_effort", {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0}));

  if (log) { log("TemplateControlLaw configured."); }
  return true;
}

// Real-time: called every control cycle. Return the joint-torque vector (size =
// number of arm joints). No heap churn / logging / locks in here if you can help it.
Eigen::VectorXd TemplateControlLaw::compute(
  const riptide::RobotState & state,
  const riptide::EndEffectorTarget & target,
  double /*dt*/)
{
  // Refresh the model from the latest state (FK, Jacobian, M, C+g, ...). Do this
  // once at the top before querying the model.
  model_->update(state);

  // ---- Everything you can read -------------------------------------------
  //   Arm joint space (7-vectors):
  //     state.q            joint positions
  //     state.dq           joint velocities
  //   Floating base:
  //     state.base_pose    world<-base pose (Eigen::Isometry3d)
  //     state.base_twist   6-vec: [linear(world); angular(body)]
  //     state.ee_pose      convenience EE pose
  //     state.extra["k"]   optional extra sensors (add to RobotState upstream)
  //   Desired EE (world frame):
  //     target.pose        desired EE pose (Isometry3d)
  //     target.twist       feedforward EE velocity (6-vec)
  //     target.accel       feedforward EE acceleration (6-vec)
  //   Dynamics model (world frame for EE quantities):
  //     model_->framePose("ee")   current EE pose
  //     model_->jacobian("ee")    6 x n geometric Jacobian
  //     model_->massMatrix()      n x n inertia M
  //     model_->nonlinear()       C(q,dq)dq + g (+ drag if hydro is enabled)
  //     model_->hydroForces(state)  generalized hydrodynamic drag
  // ------------------------------------------------------------------------

  // Convenience quantities most task-space laws start from:
  const Eigen::Matrix<double, 6, 1> x_err =
    task_pose_error(model_->framePose("ee"), target.pose);   // x_d - x (world)
  const Eigen::Matrix<double, 6, 1> v_ee =
    model_->jacobian("ee") * state.dq;                       // arm-induced EE velocity

  // ===================== YOUR CONTROL MATH HERE ===========================
  // Produce a desired task-space acceleration a_des (6-vec: [pos; ori]).
  // The default below is a task-space PD; replace it with your approach.
  const Eigen::Matrix<double, 6, 1> a_des =
    kp_.cwiseProduct(x_err) - kd_.cwiseProduct(v_ee);
  // ========================================================================

  // Map the task acceleration to joint torques through the operational-space
  // inertia, with a dynamically-consistent nullspace posture task. (Or compute
  // the torque yourself and return any n-vector — this helper is optional.)
  return operational_space_torque(
    *model_, a_des, state.q, state.dq, q_rest_, null_kp_, null_kd_,
    jacobian_damping_, max_effort_);
}

// Clear any internal state (integrators, warm-starts) on (re)activation.
void TemplateControlLaw::reset() {}

}  // namespace riptide_control

