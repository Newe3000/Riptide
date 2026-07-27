#include "riptide_control/task_space_impedance.hpp"

#include <algorithm>
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
}  // namespace

bool TaskSpaceImpedance::on_configure(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & /*logging*/,
  std::shared_ptr<riptide::IDynamicsModel> model)
{
  model_ = std::move(model);

  const auto kp_pos = get_or_declare<std::vector<double>>(params, "impedance.kp_pos", {400.0, 400.0, 400.0});
  const auto kp_ori = get_or_declare<std::vector<double>>(params, "impedance.kp_ori", {40.0, 40.0, 40.0});
  const auto kd_pos = get_or_declare<std::vector<double>>(params, "impedance.kd_pos", {40.0, 40.0, 40.0});
  const auto kd_ori = get_or_declare<std::vector<double>>(params, "impedance.kd_ori", {8.0, 8.0, 8.0});
  kp_ << kp_pos[0], kp_pos[1], kp_pos[2], kp_ori[0], kp_ori[1], kp_ori[2];
  kd_ << kd_pos[0], kd_pos[1], kd_pos[2], kd_ori[0], kd_ori[1], kd_ori[2];

  null_kp_ = get_or_declare<double>(params, "impedance.null_kp", 5.0);
  null_kd_ = get_or_declare<double>(params, "impedance.null_kd", 1.0);
  jacobian_damping_ = get_or_declare<double>(params, "impedance.jacobian_damping", 1e-3);

  q_rest_ = to_vec(get_or_declare<std::vector<double>>(
    params, "impedance.q_rest", {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}));
  max_effort_ = to_vec(get_or_declare<std::vector<double>>(
    params, "impedance.max_effort", {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0}));
  return true;
}

Eigen::VectorXd TaskSpaceImpedance::compute(
  const riptide::RobotState & state,
  const riptide::EndEffectorTarget & target,
  double /*dt*/)
{
  model_->update(state);

  const Eigen::Isometry3d X = model_->framePose("ee");   // world EE pose
  const Eigen::MatrixXd J = model_->jacobian("ee");       // 6 x n (world)
  const Eigen::Index n = J.cols();

  // Task-space pose error (world).
  Eigen::Matrix<double, 6, 1> x_err;
  x_err.head<3>() = target.pose.translation() - X.translation();
  const Eigen::Matrix3d R_err = target.pose.rotation() * X.rotation().transpose();
  const Eigen::AngleAxisd aa(R_err);
  x_err.tail<3>() = aa.angle() * aa.axis();

  // Damp the arm-induced EE velocity. Base motion is rejected through the
  // position error (x_err) rather than a base-velocity feedforward: on a light
  // free-floating base the arm's reaction forces move the base, so aggressive
  // feedforward is counter-productive (it excites the base more than it helps).
  const Eigen::Matrix<double, 6, 1> v_ee = J * state.dq;

  const Eigen::Matrix<double, 6, 1> wrench =
    kp_.cwiseProduct(x_err) - kd_.cwiseProduct(v_ee);

  Eigen::VectorXd tau = J.transpose() * wrench;

  // Posture task in the nullspace (resolves the 7-DoF redundancy).
  const Eigen::MatrixXd JJt =
    J * J.transpose() + jacobian_damping_ * Eigen::MatrixXd::Identity(6, 6);
  const Eigen::MatrixXd J_pinv = J.transpose() * JJt.inverse();       // n x 6
  const Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n, n) - J_pinv * J;
  Eigen::VectorXd tau_posture = Eigen::VectorXd::Zero(n);
  if (q_rest_.size() == n)
  {
    tau_posture = null_kp_ * (q_rest_ - state.q) - null_kd_ * state.dq;
  }
  tau += N * tau_posture;

  // Compensate Coriolis/gravity (gravity is ~0 underwater; harmless otherwise).
  tau += model_->nonlinear();

  // Clamp to per-joint torque limits.
  for (Eigen::Index i = 0; i < n; ++i)
  {
    const double lim = (i < max_effort_.size()) ? max_effort_[i] : 1e9;
    tau[i] = std::clamp(tau[i], -lim, lim);
  }
  return tau;
}

void TaskSpaceImpedance::reset() {}

}  // namespace riptide_control

PLUGINLIB_EXPORT_CLASS(riptide_control::TaskSpaceImpedance, riptide::IControlLaw)
