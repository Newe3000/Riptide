#pragma once

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace riptide_mujoco
{

/// ros2_control SystemInterface that embeds MuJoCo and runs the physics step in
/// the controller_manager update loop. It occupies the exact seam the Phase 1
/// mock_components/GenericSystem occupied, so nothing above it changes.
///
/// Mapping is by name: each ros2_control <joint> is matched to the MuJoCo joint
/// and actuator of the same name (the MJCF is authored with fer_joint1..7). This
/// keeps the extensibility guarantee (R2): adding an actuator/sensor is a
/// declarative change in the MJCF + URDF, not a code change here.
class MujocoSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

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
  // MuJoCo model + state (owned).
  mjModel * m_{nullptr};
  mjData * d_{nullptr};

  // Per-ros2_control-joint indices into MuJoCo arrays (parallel to info_.joints).
  std::vector<int> qpos_adr_;   ///< index into d_->qpos
  std::vector<int> dof_adr_;    ///< index into d_->qvel / qfrc
  std::vector<int> act_id_;     ///< index into d_->ctrl / actuator_force

  // ros2_control interface storage (parallel to info_.joints).
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> eff_;      ///< measured actuator force
  std::vector<double> eff_cmd_;  ///< commanded joint torque

  // Optional home configuration applied on activation (radians), size = n joints.
  std::vector<double> home_;
};

}  // namespace riptide_mujoco
