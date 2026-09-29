#pragma once

#include <memory>

#include <Eigen/Dense>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "riptide_dynamics/dynamics_model_interface.hpp"
#include "riptide_dynamics/robot_state.hpp"

namespace riptide
{

/// Strategy interface for a control approach. Implementations (PidControlLaw,
/// ImpedanceControlLaw, LqrControlLaw, MpcControlLaw) are pluginlib plugins
/// hosted by the single EeStabilizationController. compute() is a pure
/// RobotState -> torque mapping with no ros2_control/MuJoCo dependency, so each
/// law is unit-testable in isolation and hot-swappable at runtime.
class IControlLaw
{
public:
  virtual ~IControlLaw() = default;

  /// One-time setup: read parameters (host-agnostic: works for both
  /// rclcpp::Node and LifecycleNode via their interfaces), capture the model.
  virtual bool on_configure(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging,
    std::shared_ptr<IDynamicsModel> model) = 0;

  /// Real-time safe: no heap allocation, no logging, no locks. Returns the
  /// joint torque vector to write to the effort command interfaces.
  virtual Eigen::VectorXd compute(
    const RobotState & state,
    const EndEffectorTarget & target,
    double dt) = 0;

  /// Clear internal state (integrators, warm-starts) on (re)activation.
  virtual void reset() = 0;
};

}  // namespace riptide
