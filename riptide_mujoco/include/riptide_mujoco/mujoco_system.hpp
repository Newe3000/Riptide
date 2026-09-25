#pragma once

#include <array>
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "riptide_msgs/msg/disturbance_command.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace riptide_mujoco
{

/// ros2_control SystemInterface that embeds MuJoCo and runs the physics step in
/// the controller_manager update loop.
///
/// Actuated joints are matched to MuJoCo joints/actuators by name (R2). The
/// floating base is exposed as a ros2_control "sensor" (pose + twist state
/// interfaces) so controllers can read it. An internal node subscribes to
/// disturbance-wrench commands (applied to mjData.xfrc_applied), echoes them as
/// ground truth, and publishes the base TF + odometry for visualization.
class MujocoSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ~MujocoSystem() override;

private:
  void disturbance_callback(const riptide_msgs::msg::DisturbanceCommand & msg);
  void current_callback(const geometry_msgs::msg::Vector3Stamped & msg);
  void publish_base_state(const rclcpp::Time & time);

  // MuJoCo model + state (owned).
  mjModel * m_{nullptr};
  mjData * d_{nullptr};

  // Per-ros2_control-joint indices into MuJoCo arrays (parallel to info_.joints).
  std::vector<int> qpos_adr_;
  std::vector<int> dof_adr_;
  std::vector<int> act_id_;

  // Actuated-joint interface storage (parallel to info_.joints).
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> eff_;
  std::vector<double> eff_cmd_;
  std::vector<double> home_;

  // Hull thrusters exposed via a ros2_control <gpio> (Phase 4b). Each command
  // interface is mapped by name to a MuJoCo force actuator; the state interface
  // reports the applied force. Parallel arrays.
  std::vector<std::string> thr_gpio_;    ///< owning gpio name (interface prefix)
  std::vector<std::string> thr_name_;    ///< actuator / interface name
  std::vector<int> thr_act_id_;          ///< MuJoCo actuator id
  std::vector<double> thr_cmd_;          ///< commanded force [N]
  std::vector<double> thr_force_;        ///< applied force (state) [N]

  // Floating base exposed as a sensor: pose (3 + quat) + twist (6) = 13 values.
  bool has_base_{false};
  std::string base_sensor_name_;
  int base_qpos_adr_{-1};   ///< start index of the free joint in qpos (7 vals)
  int base_dof_adr_{-1};    ///< start index of the free joint in qvel (6 vals)
  std::vector<std::string> base_iface_names_;
  std::vector<double> base_state_;

  // ROS side (internal node spun on its own thread).
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;
  std::thread spin_thread_;
  std::atomic<bool> spinning_{false};

  rclcpp::Subscription<riptide_msgs::msg::DisturbanceCommand>::SharedPtr dist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_sub_;
  rclcpp::Publisher<riptide_msgs::msg::DisturbanceCommand>::SharedPtr dist_gt_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex dist_mutex_;
  std::map<std::string, std::array<double, 6>> disturbances_;  ///< body -> wrench
  std::array<double, 3> current_vel_{{0.0, 0.0, 0.0}};  ///< world-frame flow vel [m/s]

  std::string world_frame_{"world"};
  std::string base_frame_{"auv_base_link"};
  std::uint64_t cycle_{0};
};

}  // namespace riptide_mujoco
