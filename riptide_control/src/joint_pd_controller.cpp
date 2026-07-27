#include "riptide_control/joint_pd_controller.hpp"

#include <algorithm>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace riptide_control
{
namespace
{
// Expand a parameter to size n: empty -> all `fallback`, size 1 -> broadcast,
// size n -> as-is. Returns false on any other (mismatched) size.
bool fit(std::vector<double> & v, std::size_t n, double fallback)
{
  if (v.empty()) { v.assign(n, fallback); return true; }
  if (v.size() == 1) { v.assign(n, v[0]); return true; }
  return v.size() == n;
}
}  // namespace

controller_interface::CallbackReturn JointPdController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::vector<double>>("kp", {});
  auto_declare<std::vector<double>>("kd", {});
  auto_declare<std::vector<double>>("hold_position", {});
  auto_declare<std::vector<double>>("max_effort", {});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPdController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  if (joints_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  const std::size_t n = joints_.size();

  kp_ = get_node()->get_parameter("kp").as_double_array();
  kd_ = get_node()->get_parameter("kd").as_double_array();
  hold_ = get_node()->get_parameter("hold_position").as_double_array();
  max_effort_ = get_node()->get_parameter("max_effort").as_double_array();

  if (!fit(kp_, n, 0.0) || !fit(kd_, n, 0.0) ||
      !fit(max_effort_, n, std::numeric_limits<double>::infinity()))
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "kp/kd/max_effort must be empty, scalar, or match joints size (%zu).", n);
    return controller_interface::CallbackReturn::ERROR;
  }

  // Empty hold_position -> hold whatever pose the arm is in at activation.
  capture_hold_on_activate_ = hold_.empty();
  if (!capture_hold_on_activate_ && hold_.size() != n)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "hold_position must be empty or match joints size (%zu).", n);
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(),
    "JointPdController configured for %zu joints.", n);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointPdController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_)
  {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
JointPdController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_)
  {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return cfg;
}

controller_interface::CallbackReturn JointPdController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::size_t n = joints_.size();
  cmd_effort_idx_.assign(n, 0);
  state_pos_idx_.assign(n, 0);
  state_vel_idx_.assign(n, 0);

  // Resolve claimed interfaces to joints by (prefix, interface) name.
  auto find_state = [&](const std::string & joint, const std::string & iface,
                        std::size_t & out) -> bool {
    for (std::size_t k = 0; k < state_interfaces_.size(); ++k)
    {
      if (state_interfaces_[k].get_prefix_name() == joint &&
          state_interfaces_[k].get_interface_name() == iface)
      {
        out = k;
        return true;
      }
    }
    return false;
  };
  auto find_cmd = [&](const std::string & joint, const std::string & iface,
                      std::size_t & out) -> bool {
    for (std::size_t k = 0; k < command_interfaces_.size(); ++k)
    {
      if (command_interfaces_[k].get_prefix_name() == joint &&
          command_interfaces_[k].get_interface_name() == iface)
      {
        out = k;
        return true;
      }
    }
    return false;
  };

  for (std::size_t i = 0; i < n; ++i)
  {
    if (!find_state(joints_[i], hardware_interface::HW_IF_POSITION, state_pos_idx_[i]) ||
        !find_state(joints_[i], hardware_interface::HW_IF_VELOCITY, state_vel_idx_[i]) ||
        !find_cmd(joints_[i], hardware_interface::HW_IF_EFFORT, cmd_effort_idx_[i]))
    {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Missing claimed interface(s) for joint '%s'.", joints_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  if (capture_hold_on_activate_)
  {
    hold_.assign(n, 0.0);
    for (std::size_t i = 0; i < n; ++i)
    {
      hold_[i] = state_interfaces_[state_pos_idx_[i]].get_optional().value_or(0.0);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPdController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Command zero torque so the arm is not left with a stale hold command.
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    (void)command_interfaces_[cmd_effort_idx_[i]].set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointPdController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    const double q = state_interfaces_[state_pos_idx_[i]].get_optional().value_or(0.0);
    const double dq = state_interfaces_[state_vel_idx_[i]].get_optional().value_or(0.0);

    double tau = kp_[i] * (hold_[i] - q) - kd_[i] * dq;
    tau = std::clamp(tau, -max_effort_[i], max_effort_[i]);

    (void)command_interfaces_[cmd_effort_idx_[i]].set_value(tau);
  }
  return controller_interface::return_type::OK;
}

}  // namespace riptide_control

PLUGINLIB_EXPORT_CLASS(
  riptide_control::JointPdController, controller_interface::ControllerInterface)
