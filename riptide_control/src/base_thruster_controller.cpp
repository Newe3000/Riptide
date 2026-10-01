#include "riptide_control/base_thruster_controller.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "riptide_geometry/allocation.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace riptide_control
{
namespace
{
// Canonical floating-base sensor interfaces (must match riptide_mujoco).
const std::vector<std::string> kBaseIfaceNames = {
  "position.x", "position.y", "position.z",
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "linear_velocity.x", "linear_velocity.y", "linear_velocity.z",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z"};
}  // namespace

controller_interface::CallbackReturn BaseThrusterController::on_init()
{
  auto_declare<std::string>("base_sensor", "auv_base");
  auto_declare<std::string>("thruster_gpio", "thrusters");
  auto_declare<std::vector<std::string>>("thrusters", {});
  auto_declare<std::vector<double>>("thruster_positions", {});  // flat 3N
  auto_declare<std::vector<double>>("thruster_axes", {});       // flat 3N
  auto_declare<double>("max_thrust", 200.0);

  auto_declare<double>("kp_x", 0.0);
  auto_declare<double>("kd_x", 0.0);
  auto_declare<double>("kp_y", 0.0);
  auto_declare<double>("kd_y", 0.0);
  auto_declare<double>("kp_z", 0.0);
  auto_declare<double>("kd_z", 0.0);
  auto_declare<double>("kp_roll", 0.0);
  auto_declare<double>("kd_roll", 0.0);
  auto_declare<double>("kp_pitch", 0.0);
  auto_declare<double>("kd_pitch", 0.0);
  auto_declare<double>("kp_yaw", 0.0);
  auto_declare<double>("kd_yaw", 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BaseThrusterController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  base_sensor_ = node->get_parameter("base_sensor").as_string();
  thruster_gpio_ = node->get_parameter("thruster_gpio").as_string();
  thruster_names_ = node->get_parameter("thrusters").as_string_array();
  base_iface_names_ = kBaseIfaceNames;
  max_thrust_ = node->get_parameter("max_thrust").as_double();

  kp_x_ = node->get_parameter("kp_x").as_double();
  kd_x_ = node->get_parameter("kd_x").as_double();
  kp_y_ = node->get_parameter("kp_y").as_double();
  kd_y_ = node->get_parameter("kd_y").as_double();
  kp_z_ = node->get_parameter("kp_z").as_double();
  kd_z_ = node->get_parameter("kd_z").as_double();
  kp_roll_ = node->get_parameter("kp_roll").as_double();
  kd_roll_ = node->get_parameter("kd_roll").as_double();
  kp_pitch_ = node->get_parameter("kp_pitch").as_double();
  kd_pitch_ = node->get_parameter("kd_pitch").as_double();
  kp_yaw_ = node->get_parameter("kp_yaw").as_double();
  kd_yaw_ = node->get_parameter("kd_yaw").as_double();

  const auto pos = node->get_parameter("thruster_positions").as_double_array();
  const auto axes = node->get_parameter("thruster_axes").as_double_array();
  const std::size_t nt = thruster_names_.size();
  if (nt == 0 || pos.size() != 3 * nt || axes.size() != 3 * nt)
  {
    RCLCPP_ERROR(node->get_logger(),
      "Need 'thrusters' (N) with 'thruster_positions'/'thruster_axes' of size 3N "
      "(got N=%zu, pos=%zu, axes=%zu).", nt, pos.size(), axes.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Build the 6 x N thruster allocation and its regularized pseudo-inverse in the
  // eigen-only riptide_geometry leaf (Tikhonov reg keeps weakly-actuated DOFs
  // well-posed). Column i is thruster i's unit wrench [axis; r x axis].
  std::vector<Eigen::Vector3d> positions(nt), thruster_axes(nt);
  for (std::size_t i = 0; i < nt; ++i)
  {
    positions[i] = Eigen::Vector3d(pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]);
    thruster_axes[i] = Eigen::Vector3d(axes[3 * i], axes[3 * i + 1], axes[3 * i + 2]);
  }
  const Eigen::MatrixXd A = riptide_geometry::allocation_matrix(positions, thruster_axes);
  alloc_pinv_ = riptide_geometry::regularized_pinv(A);   // N x 6, reg = 1e-6

  RCLCPP_INFO(node->get_logger(),
    "BaseThrusterController configured: %zu thrusters, sensor '%s'.",
    nt, base_sensor_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BaseThrusterController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & t : thruster_names_)
  {
    cfg.names.push_back(thruster_gpio_ + "/" + t);
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
BaseThrusterController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & n : base_iface_names_)
  {
    cfg.names.push_back(base_sensor_ + "/" + n);
  }
  return cfg;
}

controller_interface::CallbackReturn BaseThrusterController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto find = [](const auto & vec, const std::string & prefix, const std::string & iface,
                 std::size_t & out) -> bool {
    for (std::size_t k = 0; k < vec.size(); ++k)
    {
      if (vec[k].get_prefix_name() == prefix && vec[k].get_interface_name() == iface)
      {
        out = k;
        return true;
      }
    }
    return false;
  };

  cmd_idx_.assign(thruster_names_.size(), 0);
  for (std::size_t i = 0; i < thruster_names_.size(); ++i)
  {
    if (!find(command_interfaces_, thruster_gpio_, thruster_names_[i], cmd_idx_[i]))
    {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Missing thruster command interface '%s/%s'.",
        thruster_gpio_.c_str(), thruster_names_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  base_idx_.assign(base_iface_names_.size(), 0);
  for (std::size_t k = 0; k < base_iface_names_.size(); ++k)
  {
    if (!find(state_interfaces_, base_sensor_, base_iface_names_[k], base_idx_[k]))
    {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Missing base sensor interface '%s/%s'.",
        base_sensor_.c_str(), base_iface_names_[k].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Capture the hold pose on the first finite update() (see EeStabilizationController:
  // at activation the hardware's first read() may not have run, so state is NaN).
  capture_pending_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BaseThrusterController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (std::size_t i = 0; i < cmd_idx_.size(); ++i)
  {
    (void)command_interfaces_[cmd_idx_[i]].set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool BaseThrusterController::read_base(
  Eigen::Vector3d & p, Eigen::Matrix3d & R,
  Eigen::Vector3d & v_world, Eigen::Vector3d & w_body) const
{
  auto v = [&](std::size_t k) {
    return state_interfaces_[base_idx_[k]].get_optional().value_or(0.0);
  };
  p = Eigen::Vector3d(v(0), v(1), v(2));
  Eigen::Quaterniond q(v(6), v(3), v(4), v(5));  // (w, x, y, z)
  v_world = Eigen::Vector3d(v(7), v(8), v(9));
  w_body = Eigen::Vector3d(v(10), v(11), v(12));
  if (!p.allFinite() || !v_world.allFinite() || !w_body.allFinite() ||
      !std::isfinite(q.w()) || q.norm() < 1e-6)
  {
    return false;
  }
  R = q.normalized().toRotationMatrix();
  return true;
}

controller_interface::return_type BaseThrusterController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto hold_zero = [&]() {
    for (std::size_t i = 0; i < cmd_idx_.size(); ++i)
    {
      (void)command_interfaces_[cmd_idx_[i]].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  };

  Eigen::Vector3d p, v_world, w_body;
  Eigen::Matrix3d R;
  if (!read_base(p, R, v_world, w_body)) { return hold_zero(); }

  const double yaw = std::atan2(R(1, 0), R(0, 0));   // heading about world z
  if (capture_pending_)
  {
    target_pos_ = p;
    target_yaw_ = yaw;
    capture_pending_ = false;
    RCLCPP_INFO(get_node()->get_logger(),
      "Station-keeping base at x=%.3f, y=%.3f, z=%.3f, yaw=%.3f.",
      p.x(), p.y(), p.z(), yaw);
  }

  // Desired restoring force in the world frame (x, y, z), rotated into the body
  // frame the thrusters act in.
  const Eigen::Vector3d f_world(
    kp_x_ * (target_pos_.x() - p.x()) - kd_x_ * v_world.x(),
    kp_y_ * (target_pos_.y() - p.y()) - kd_y_ * v_world.y(),
    kp_z_ * (target_pos_.z() - p.z()) - kd_z_ * v_world.z());
  const Eigen::Vector3d f_body = R.transpose() * f_world;

  // Leveling: drive the body z-axis toward world up. tilt = (R*z) x up gives the
  // world-frame correction axis; expressed in the body frame it is the roll/pitch
  // error. Angular velocity is already body-frame (from the free-joint sensor).
  const Eigen::Vector3d z_body_in_world = R.col(2);
  const Eigen::Vector3d tilt_body =
    R.transpose() * z_body_in_world.cross(Eigen::Vector3d::UnitZ());
  const double m_x = kp_roll_ * tilt_body.x() - kd_roll_ * w_body.x();
  const double m_y = kp_pitch_ * tilt_body.y() - kd_pitch_ * w_body.y();

  // Yaw: hold the captured heading. For a near-level hull body-z ~ world-z, so
  // the heading error maps directly onto the body-frame yaw torque.
  double yaw_err = target_yaw_ - yaw;
  yaw_err = std::atan2(std::sin(yaw_err), std::cos(yaw_err));   // wrap to [-pi, pi]
  const double m_z = kp_yaw_ * yaw_err - kd_yaw_ * w_body.z();

  Eigen::VectorXd wrench(6);
  wrench << f_body.x(), f_body.y(), f_body.z(), m_x, m_y, m_z;
  Eigen::VectorXd u = alloc_pinv_ * wrench;   // per-thruster force
  if (!u.allFinite()) { return hold_zero(); }

  for (std::size_t i = 0; i < cmd_idx_.size() && i < static_cast<std::size_t>(u.size()); ++i)
  {
    const double f = std::clamp(u[i], -max_thrust_, max_thrust_);
    (void)command_interfaces_[cmd_idx_[i]].set_value(f);
  }
  return controller_interface::return_type::OK;
}

}  // namespace riptide_control

PLUGINLIB_EXPORT_CLASS(
  riptide_control::BaseThrusterController, controller_interface::ControllerInterface)
