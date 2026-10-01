#include "riptide_control/task_space_lqr.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "riptide_control/operational_space.hpp"

namespace riptide_control
{
namespace
{
template <typename T>
T get_or_declare(riptide::ParamSource & params, const std::string & name, const T & def)
{
  return params.declare(name, def);
}

Eigen::VectorXd to_vec(const std::vector<double> & v)
{
  return Eigen::Map<const Eigen::VectorXd>(v.data(), static_cast<Eigen::Index>(v.size()));
}

// Closed-form CARE solution for the scalar double integrator (docs eq. 14):
//   min \int q_p e^2 + q_v e_dot^2 + r w^2  s.t.  e_ddot = w
//   => w = -Kp e - Kd e_dot,  Kp = sqrt(q_p/r),  Kd = sqrt(2*sqrt(q_p/r) + q_v/r).
void lqr_gains_axis(double q_p, double q_v, double r, double & kp, double & kd)
{
  const double ratio = q_p / r;          // = (Kp)^2
  kp = std::sqrt(ratio);
  kd = std::sqrt(2.0 * std::sqrt(ratio) + q_v / r);
}
}  // namespace

bool TaskSpaceLqr::on_configure(
  riptide::ParamSource & params,
  const riptide::Logger & log,
  std::shared_ptr<riptide::IDynamicsModel> model)
{
  model_ = std::move(model);

  // Cost weights. Q = diag(q_pos, q_ori, qd_pos, qd_ori); R = diag(r_pos, r_ori).
  // (Position/orientation split so the two task subspaces can be weighted apart.)
  const auto q_pos = get_or_declare<std::vector<double>>(params, "lqr.q_pos", {1000.0, 1000.0, 1000.0});
  const auto q_ori = get_or_declare<std::vector<double>>(params, "lqr.q_ori", {50.0, 50.0, 50.0});
  const auto qd_pos = get_or_declare<std::vector<double>>(params, "lqr.qd_pos", {1.0, 1.0, 1.0});
  const auto qd_ori = get_or_declare<std::vector<double>>(params, "lqr.qd_ori", {1.0, 1.0, 1.0});
  const auto r_pos = get_or_declare<std::vector<double>>(params, "lqr.r_pos", {1.0, 1.0, 1.0});
  const auto r_ori = get_or_declare<std::vector<double>>(params, "lqr.r_ori", {1.0, 1.0, 1.0});

  const std::array<double, 6> qp{q_pos[0], q_pos[1], q_pos[2], q_ori[0], q_ori[1], q_ori[2]};
  const std::array<double, 6> qv{qd_pos[0], qd_pos[1], qd_pos[2], qd_ori[0], qd_ori[1], qd_ori[2]};
  const std::array<double, 6> rr{r_pos[0], r_pos[1], r_pos[2], r_ori[0], r_ori[1], r_ori[2]};
  for (int i = 0; i < 6; ++i)
  {
    double kp = 0.0, kd = 0.0;
    lqr_gains_axis(qp[i], qv[i], rr[i], kp, kd);
    kp_[i] = kp;
    kd_[i] = kd;
  }

  null_kp_ = get_or_declare<double>(params, "lqr.null_kp", 5.0);
  null_kd_ = get_or_declare<double>(params, "lqr.null_kd", 1.0);
  jacobian_damping_ = get_or_declare<double>(params, "lqr.jacobian_damping", 1e-3);
  q_rest_ = to_vec(get_or_declare<std::vector<double>>(
    params, "lqr.q_rest", {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}));
  max_effort_ = to_vec(get_or_declare<std::vector<double>>(
    params, "lqr.max_effort", {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0}));

  if (log)
  {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "TaskSpaceLqr configured: optimal task gains "
      "Kp=[%.1f %.1f %.1f | %.1f %.1f %.1f], Kd=[%.1f %.1f %.1f | %.1f %.1f %.1f].",
      kp_[0], kp_[1], kp_[2], kp_[3], kp_[4], kp_[5],
      kd_[0], kd_[1], kd_[2], kd_[3], kd_[4], kd_[5]);
    log(buf);
  }
  return true;
}

Eigen::VectorXd TaskSpaceLqr::compute(
  const riptide::RobotState & state,
  const riptide::EndEffectorTarget & target,
  double /*dt*/)
{
  model_->update(state);

  // LQR-optimal task acceleration: w = Kp x_err - Kd (J dq), with the gains from
  // the closed-form CARE solution (on_configure). Realized through the shared
  // operational-space mapping (same as the impedance law; only the gains differ).
  const Eigen::Matrix<double, 6, 1> x_err =
    task_pose_error(model_->framePose("ee"), target.pose);
  const Eigen::Matrix<double, 6, 1> v_ee = model_->jacobian("ee") * state.dq;
  const Eigen::Matrix<double, 6, 1> w = kp_.cwiseProduct(x_err) - kd_.cwiseProduct(v_ee);

  return operational_space_torque(
    *model_, w, state.q, state.dq, q_rest_, null_kp_, null_kd_,
    jacobian_damping_, max_effort_);
}

void TaskSpaceLqr::reset() {}

}  // namespace riptide_control

PLUGINLIB_EXPORT_CLASS(riptide_control::TaskSpaceLqr, riptide::IControlLaw)
