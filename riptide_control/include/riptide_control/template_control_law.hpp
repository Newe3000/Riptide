#pragma once

#include <memory>

#include <Eigen/Dense>

#include "riptide_control/control_law_interface.hpp"
#include "riptide_dynamics/dynamics_model_interface.hpp"

namespace riptide_control
{

/// Blank IControlLaw to copy when adding a new control approach.
///
/// A control law is a pure  (RobotState, EndEffectorTarget) -> joint torque
/// mapping; the host EeStabilizationController does all the ros2_control I/O,
/// base-thruster control, joint-limit avoidance, torque clamping and logging.
///
/// To make your own: copy this pair of files, rename the class, register it in
/// control_law_plugins.xml + CMakeLists.txt, add a config/law_<name>.yaml
/// selector, then run with control_law:=<name>. See compute() for every input
/// you can read and where to put your control math.
class TemplateControlLaw : public riptide::IControlLaw
{
public:
  bool on_configure(
    riptide::ParamSource & params,
    const riptide::Logger & log,
    std::shared_ptr<riptide::IDynamicsModel> model) override;
  Eigen::VectorXd compute(
    const riptide::RobotState & state,
    const riptide::EndEffectorTarget & target,
    double dt) override;
  void reset() override;

private:
  // The whole-body dynamics oracle (M, C+g, J, EE pose, hydro, joint limits).
  std::shared_ptr<riptide::IDynamicsModel> model_;

  // Example parameters (delete/replace with your own). Read in on_configure().
  Eigen::Matrix<double, 6, 1> kp_;   ///< task stiffness [pos(3); ori(3)]
  Eigen::Matrix<double, 6, 1> kd_;   ///< task damping
  double null_kp_{5.0};
  double null_kd_{1.0};
  double jacobian_damping_{1e-3};
  Eigen::VectorXd q_rest_;           ///< nullspace posture target
  Eigen::VectorXd max_effort_;       ///< per-joint torque clamp [Nm]
};

}  // namespace riptide_control
