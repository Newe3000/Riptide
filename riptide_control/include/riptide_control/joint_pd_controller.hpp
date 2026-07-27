#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace riptide_control
{

/// Joint-space PD (effort) controller: tau_i = kp_i*(q*_i - q_i) - kd_i*dq_i,
/// clamped to per-joint torque limits.
///
/// This is the simplest thing that visibly demonstrates torque control through
/// the full ros2_control stack: with it, the arm holds its target pose against
/// gravity; without it, the arm collapses. It is also the seed of the Phase 2
/// baseline loop — later replaced/augmented by the task-space IControlLaw
/// plugins (PID, impedance, LQR, MPC).
class JointPdController : public controller_interface::ControllerInterface
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
  std::vector<std::string> joints_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  std::vector<double> hold_;         ///< target positions (rad)
  std::vector<double> max_effort_;   ///< per-joint torque clamp (Nm)
  bool capture_hold_on_activate_{false};

  // Indices into command_interfaces_ / state_interfaces_, per joint.
  std::vector<std::size_t> cmd_effort_idx_;
  std::vector<std::size_t> state_pos_idx_;
  std::vector<std::size_t> state_vel_idx_;
};

}  // namespace riptide_control
