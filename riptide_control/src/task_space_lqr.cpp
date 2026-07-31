#include "riptide_control/task_space_lqr.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace riptide_control
{
namespace
{
using ParamsIface = rclcpp::node_interfaces::NodeParametersInterface;

template <typename T>
T get_or_declare(const ParamsIface::SharedPtr & params, const std::string & name, const T & def)
{
  if (!params->has_parameter(name))
  {
    params->declare_parameter(name, rclcpp::ParameterValue(def));
  }
  return params->get_parameter(name).get_value<T>();
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
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging,
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

  RCLCPP_INFO(logging->get_logger(),
    "TaskSpaceLqr configured: optimal task gains "
    "Kp=[%.1f %.1f %.1f | %.1f %.1f %.1f], Kd=[%.1f %.1f %.1f | %.1f %.1f %.1f].",
    kp_[0], kp_[1], kp_[2], kp_[3], kp_[4], kp_[5],
    kd_[0], kd_[1], kd_[2], kd_[3], kd_[4], kd_[5]);
  return true;
}

Eigen::VectorXd TaskSpaceLqr::compute(
  const riptide::RobotState & state,
  const riptide::EndEffectorTarget & target,
  double /*dt*/)
{
  model_->update(state);

  const Eigen::Isometry3d X = model_->framePose("ee");   // world EE pose
  const Eigen::MatrixXd J = model_->jacobian("ee");       // 6 x n (world)
  const Eigen::MatrixXd & M = model_->massMatrix();        // n x n
  const Eigen::Index n = J.cols();

  // Task-space error e = x - x_d, expressed as x_err = -e = x_d - x (world).
  Eigen::Matrix<double, 6, 1> x_err;
  x_err.head<3>() = target.pose.translation() - X.translation();
  const Eigen::Matrix3d R_err = target.pose.rotation() * X.rotation().transpose();
  const Eigen::AngleAxisd aa(R_err);
  x_err.tail<3>() = aa.angle() * aa.axis();

  // Arm-induced EE velocity (base motion is rejected through x_err, as in the
  // impedance law). e_dot = -(J dq); the LQR law w = -Kp e - Kd e_dot becomes:
  const Eigen::Matrix<double, 6, 1> v_ee = J * state.dq;
  const Eigen::Matrix<double, 6, 1> w = kp_.cwiseProduct(x_err) - kd_.cwiseProduct(v_ee);

  // Operational-space inertia Lambda = (J M^-1 J^T)^-1 (damped for singularities),
  // then tau = J^T (Lambda w). Compensating C dq + g in joint space cancels the
  // nonlinear terms, so M q_ddot = J^T Lambda w and x_ddot ~ w near regulation.
  const Eigen::MatrixXd Minv = M.inverse();
  const Eigen::MatrixXd JMinvJt =
    J * Minv * J.transpose() + jacobian_damping_ * Eigen::MatrixXd::Identity(6, 6);
  const Eigen::MatrixXd Lambda = JMinvJt.inverse();

  Eigen::VectorXd tau = J.transpose() * (Lambda * w);

  // Posture task projected through the DYNAMICALLY-CONSISTENT nullspace
  //   N = I - J^T (Lambda J M^-1),   so that  J M^-1 N = 0
  // (Khatib OSC). This guarantees the posture torque produces no task-space
  // acceleration. A kinematic projector I - J^+ J is NOT M-orthogonal here and
  // would leak posture torque into the task, causing a steady-state task offset.
  const Eigen::MatrixXd Jbar_T = Lambda * J * Minv;                    // 6 x n
  const Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n, n) - J.transpose() * Jbar_T;
  if (q_rest_.size() == n)
  {
    const Eigen::VectorXd tau_posture = null_kp_ * (q_rest_ - state.q) - null_kd_ * state.dq;
    tau += N * tau_posture;
  }

  // Compensate Coriolis/gravity (gravity ~0 underwater; harmless otherwise).
  tau += model_->nonlinear();

  // Clamp to per-joint torque limits.
  for (Eigen::Index i = 0; i < n; ++i)
  {
    const double lim = (i < max_effort_.size()) ? max_effort_[i] : 1e9;
    tau[i] = std::clamp(tau[i], -lim, lim);
  }
  return tau;
}

void TaskSpaceLqr::reset() {}

}  // namespace riptide_control

PLUGINLIB_EXPORT_CLASS(riptide_control::TaskSpaceLqr, riptide::IControlLaw)
