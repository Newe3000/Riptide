#include "riptide_control/task_space_impedance.hpp"

#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "riptide_control/operational_space.hpp"

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

  // Inertia-shaped Cartesian impedance (Ott 2008): the desired task acceleration
  //   a_des = Kp * x_err - Kd * (J dq)
  // is realized through the operational-space inertia, so the EE presents a
  // decoupled second-order impedance (x_ddot + Kd x_dot + Kp x_err = 0) rather
  // than a pose-dependent one. Base motion is rejected through x_err; only the
  // arm-induced EE velocity (J dq) is damped (a base-velocity feedforward was
  // tried and reverted -- it excites the light free base via arm reaction).
  const Eigen::Matrix<double, 6, 1> x_err =
    task_pose_error(model_->framePose("ee"), target.pose);
  const Eigen::Matrix<double, 6, 1> v_ee = model_->jacobian("ee") * state.dq;
  const Eigen::Matrix<double, 6, 1> a_des =
    kp_.cwiseProduct(x_err) - kd_.cwiseProduct(v_ee);

  return operational_space_torque(
    *model_, a_des, state.q, state.dq, q_rest_, null_kp_, null_kd_,
    jacobian_damping_, max_effort_);
}

void TaskSpaceImpedance::reset() {}

}  // namespace riptide_control

PLUGINLIB_EXPORT_CLASS(riptide_control::TaskSpaceImpedance, riptide::IControlLaw)
