// The full migration payoff: a closed-loop task-space controller running with ZERO ROS.
// Everything here resolves from Conan -- Pinocchio (via riptide_dynamics) loads the
// vendored fer URDF, and TaskSpaceImpedance (from riptide_control_core) is driven
// through the ROS-free riptide::IControlLaw interface. We integrate the rigid-body
// forward dynamics ourselves (no MuJoCo, no ros2_control) and show the end-effector
// converge to a scripted Cartesian target.
#include "riptide_control/control_law_interface.hpp"
#include "riptide_control/task_space_impedance.hpp"
#include "riptide_dynamics/pinocchio_model.hpp"
#include "riptide_dynamics/robot_state.hpp"

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace
{
// In-memory ParamSource: every control-law parameter takes its default (no ROS node).
class DefaultParams : public riptide::ParamSource
{
public:
  double declare(const std::string &, double def) override { return def; }
  int declare(const std::string &, int def) override { return def; }
  std::vector<double> declare(const std::string &, const std::vector<double> & def) override
  {
    return def;
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  const std::string urdf = (argc > 1) ? argv[1] : "fer_arm.urdf";

  // --- dynamics core (Conan Pinocchio loads the URDF; no ~/.local/pinocchio) ---
  const std::vector<std::string> joints = {
    "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
    "fer_joint5", "fer_joint6", "fer_joint7"};
  const std::vector<std::string> locked = {"fer_finger_joint1", "fer_finger_joint2"};
  auto model = std::make_shared<riptide::PinocchioModel>(urdf, "fer_hand_tcp", joints, locked);
  const int n = static_cast<int>(joints.size());

  // --- control-law core, configured with no ROS (DefaultParams + no-op logger) ---
  DefaultParams params;
  const riptide::Logger log = [](const std::string &) {};
  riptide_control::TaskSpaceImpedance law;
  law.on_configure(params, log, model);

  // --- initial state + a scripted EE target offset from the rest pose ---
  riptide::RobotState state;
  state.q = (Eigen::VectorXd(n) << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785).finished();
  state.dq = Eigen::VectorXd::Zero(n);

  model.get()->update(state);
  const Eigen::Isometry3d X0 = model->framePose("ee");
  riptide::EndEffectorTarget target;
  target.pose = X0;
  target.pose.translation() += Eigen::Vector3d(0.03, 0.0, 0.02);   // move EE 3cm +x, 2cm +z

  auto ee_err = [&](const riptide::RobotState & s) {
    model->update(s);
    return (target.pose.translation() - model->framePose("ee").translation()).norm();
  };
  const double err0 = ee_err(state);

  // --- closed loop: tau from the law, integrate rigid-body forward dynamics ---
  const double dt = 0.004;      // 250 Hz, same as the ROS controller
  const int steps = 1500;       // 6 s
  for (int k = 0; k < steps; ++k)
  {
    model->update(state);
    const Eigen::VectorXd tau = law.compute(state, target, dt);
    // tau already includes nonlinear() (C(q,dq)dq + g); remove it to get the net
    // task+posture wrench, then forward-integrate: ddq = M^-1 (tau - nle).
    const Eigen::VectorXd ddq = model->massMatrix().ldlt().solve(tau - model->nonlinear());
    state.dq += ddq * dt;
    state.dq *= 0.999;                    // light numerical damping for the explicit integrator
    state.q  += state.dq * dt;
  }
  const double errN = ee_err(state);

  std::printf("control_loop: EE error  start=%.4f m  end=%.4f m  (target offset=0.036 m)\n",
              err0, errN);

  const bool finite = state.q.allFinite() && state.dq.allFinite();
  const bool converged = errN < 0.5 * err0 && errN < 0.02;   // moved most of the way, settled
  std::printf("control_loop: %s\n", (finite && converged) ? "PASS" : "FAIL");
  return (finite && converged) ? 0 : 1;
}
