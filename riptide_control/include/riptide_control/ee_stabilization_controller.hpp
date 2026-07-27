#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "riptide_control/control_law_interface.hpp"
#include "riptide_dynamics/dynamics_model_interface.hpp"
#include "riptide_dynamics/robot_state.hpp"

namespace riptide_control
{

/// Hosts an IControlLaw plugin: reads the arm joints + floating-base sensor,
/// assembles a RobotState, and writes the control law's joint torques. The
/// control law + dynamics model are swappable, so the ROS plumbing here is
/// shared across every control approach (PID, impedance, LQR, MPC).
class EeStabilizationController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  riptide::RobotState read_state() const;

  std::vector<std::string> joints_;
  std::string base_sensor_;
  std::vector<std::string> base_iface_names_;   // 13, canonical order
  riptide::EndEffectorTarget target_;

  std::shared_ptr<riptide::IDynamicsModel> model_;
  std::unique_ptr<pluginlib::ClassLoader<riptide::IControlLaw>> loader_;
  std::shared_ptr<riptide::IControlLaw> control_law_;

  // Indices into command_interfaces_ / state_interfaces_.
  std::vector<std::size_t> cmd_effort_idx_;
  std::vector<std::size_t> state_pos_idx_;
  std::vector<std::size_t> state_vel_idx_;
  std::vector<std::size_t> base_idx_;           // 13
  bool has_base_{false};
};

}  // namespace riptide_control
