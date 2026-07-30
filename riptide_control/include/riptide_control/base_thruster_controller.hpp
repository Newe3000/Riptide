#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace riptide_control
{

/// Base dynamic-positioning controller (Phase 4b). Reads the floating-base
/// sensor and drives the hull thrusters to station-keep the AUV, so the arm's
/// EE controller no longer fights an unbounded base drift.
///
/// It controls the 4 DOFs the thruster layout can produce:
///   * surge (Fx, forward/back) and heave (Fz, up/down)  -> PD on position,
///   * roll (Mx) and pitch (My)                          -> PD leveling to flat.
/// Sway (y) and yaw are left uncontrolled (no thrusters for them). A desired
/// body wrench [Fx, Fz, Mx, My] is mapped to per-thruster forces through the
/// pseudo-inverse of the allocation matrix, which is rebuilt from the thruster
/// geometry (positions + axes) so the layout can change from config alone.
class BaseThrusterController : public controller_interface::ControllerInterface
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
  std::string base_sensor_;
  std::string thruster_gpio_;
  std::vector<std::string> thruster_names_;
  std::vector<std::string> base_iface_names_;   // 13, canonical order

  // Allocation: u (per-thruster force) = alloc_pinv_ * W, W = [Fx, Fz, Mx, My].
  Eigen::MatrixXd alloc_pinv_;                  // N x 4
  double max_thrust_{200.0};

  // Station-keeping PD gains.
  double kp_x_{0.0}, kd_x_{0.0}, kp_z_{0.0}, kd_z_{0.0};
  double kp_roll_{0.0}, kd_roll_{0.0}, kp_pitch_{0.0}, kd_pitch_{0.0};

  // Hold target (world). x and z are held; y/yaw are not actuated.
  Eigen::Vector3d target_pos_{Eigen::Vector3d::Zero()};
  bool capture_pending_{false};

  // Claimed-interface indices.
  std::vector<std::size_t> cmd_idx_;            // per thruster
  std::vector<std::size_t> base_idx_;           // 13

  bool read_base(
    Eigen::Vector3d & p, Eigen::Matrix3d & R,
    Eigen::Vector3d & v_world, Eigen::Vector3d & w_body) const;
};

}  // namespace riptide_control
