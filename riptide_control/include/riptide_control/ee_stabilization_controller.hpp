#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "riptide_msgs/msg/control_debug.hpp"

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

  // ControlDebug publisher (for logging / the evaluation benchmark).
  rclcpp::Publisher<riptide_msgs::msg::ControlDebug>::SharedPtr debug_pub_;
  std::string control_law_name_;
  std::uint64_t cycle_{0};

  // Live desired-pose command channel (e.g. the teleop GUI). A PoseStamped on
  // /riptide/ee_target (world frame) overrides the held target in real time; the
  // current target is republished (latched) on /riptide/ee_target/current so a
  // late-joining teleop can seed itself without snapping the EE.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_current_pub_;
  std::mutex target_cmd_mutex_;
  Eigen::Isometry3d target_cmd_pose_{Eigen::Isometry3d::Identity()};
  std::atomic<bool> have_target_cmd_{false};
  void target_callback(const geometry_msgs::msg::PoseStamped & msg);
  void publish_current_target();

  // Joint-limit avoidance: a repulsive torque that switches on only within
  // `jla_buffer_` of a limit and grows toward it, applied on top of any law.
  bool jla_enabled_{true};
  double jla_buffer_{0.2};    ///< activation distance from the limit [rad]
  double jla_gain_{40.0};     ///< repulsive strength at the limit [Nm]
  double jla_damping_{2.0};   ///< damping of motion into the limit [Nm.s/rad]
  Eigen::VectorXd q_lower_, q_upper_;   ///< joint position limits (controller order)
  Eigen::VectorXd max_effort_;          ///< final per-joint torque clamp [Nm]

  Eigen::VectorXd jointLimitAvoidance(
    const Eigen::VectorXd & q, const Eigen::VectorXd & dq) const;

  // The EE hold target is captured on the first update() cycle whose measured
  // state is finite, not in on_activate(): at activation the hardware's first
  // read() may not have run yet, so the state interfaces can still read NaN,
  // which would propagate into the target and every torque.
  bool capture_pending_{false};
};

}  // namespace riptide_control
