// Ports the control-law golden test to run with ZERO ROS: a mock dynamics model,
// an in-memory ParamSource (defaults only), and the Step-1 golden fixture. Proves
// the de-ROS'd laws in riptide_control_core reproduce the recorded torques.
#include "riptide_control/task_space_impedance.hpp"
#include "riptide_control/task_space_lqr.hpp"
#include "riptide_control/task_space_mpc.hpp"
#include "riptide_control/template_control_law.hpp"
#include "riptide_dynamics/dynamics_model_interface.hpp"
#include "riptide_dynamics/robot_state.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr int kN = 7;

// Identical to the unit-test mock: M=I, nle=0, J=[I6|0], EE at (0.4,0,1.0).
class MockModel : public riptide::IDynamicsModel
{
public:
  MockModel()
  {
    M_ = Eigen::MatrixXd::Identity(kN, kN);
    nle_ = Eigen::VectorXd::Zero(kN);
    J_ = Eigen::MatrixXd::Zero(6, kN);
    J_.leftCols<6>() = Eigen::MatrixXd::Identity(6, 6);
    ee_ = Eigen::Isometry3d::Identity();
    ee_.translation() = Eigen::Vector3d(0.4, 0.0, 1.0);
  }
  void update(const riptide::RobotState &) override {}
  const Eigen::MatrixXd & massMatrix() const override { return M_; }
  const Eigen::VectorXd & nonlinear() const override { return nle_; }
  Eigen::MatrixXd jacobian(const std::string &) const override { return J_; }
  Eigen::Isometry3d framePose(const std::string &) const override { return ee_; }
  Eigen::VectorXd hydroForces(const riptide::RobotState &) const override
  {
    return Eigen::VectorXd::Zero(kN);
  }
private:
  Eigen::MatrixXd M_, J_;
  Eigen::VectorXd nle_;
  Eigen::Isometry3d ee_;
};

// In-memory ParamSource: returns the default for every parameter (no overrides),
// matching the golden harness run on a fresh node.
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

riptide::RobotState rest_state()
{
  riptide::RobotState s;
  s.q = (Eigen::VectorXd(kN) << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785).finished();
  s.dq = Eigen::VectorXd::Zero(kN);
  return s;
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string fixture = (argc > 1) ? argv[1] : "control_law_torques.csv";
  auto model = std::make_shared<MockModel>();
  DefaultParams params;
  const riptide::Logger log = [](const std::string &) {};

  const auto state = rest_state();
  riptide::EndEffectorTarget target;
  target.pose = Eigen::Isometry3d::Identity();
  target.pose.translation() = Eigen::Vector3d(0.5, 0.1, 1.05);
  const double dt = 0.004;

  std::map<std::string, Eigen::VectorXd> got;
  { riptide_control::TaskSpaceImpedance l; l.on_configure(params, log, model); got["impedance"] = l.compute(state, target, dt); }
  { riptide_control::TaskSpaceLqr l; l.on_configure(params, log, model); got["lqr"] = l.compute(state, target, dt); }
  { riptide_control::TaskSpaceMpc l; l.on_configure(params, log, model); l.reset(); got["mpc"] = l.compute(state, target, dt); }
  { riptide_control::TemplateControlLaw l; l.on_configure(params, log, model); got["template"] = l.compute(state, target, dt); }

  std::ifstream in(fixture);
  if (!in.good()) { std::printf("cannot open fixture: %s\n", fixture.c_str()); return 2; }
  std::map<std::string, Eigen::VectorXd> expected;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') { continue; }
    std::stringstream ss(line);
    std::string name, cell;
    std::getline(ss, name, ',');
    std::vector<double> v;
    while (std::getline(ss, cell, ',')) { v.push_back(std::stod(cell)); }
    Eigen::VectorXd e(static_cast<Eigen::Index>(v.size()));
    for (std::size_t i = 0; i < v.size(); ++i) { e[static_cast<Eigen::Index>(i)] = v[i]; }
    expected[name] = e;
  }

  const double tol = 1e-9;
  int bad = 0;
  for (const auto & [name, tau] : got) {
    if (!expected.count(name)) { std::printf("fixture missing law: %s\n", name.c_str()); ++bad; continue; }
    for (int i = 0; i < tau.size(); ++i) {
      if (std::abs(tau[i] - expected[name][i]) > tol) {
        std::printf("MISMATCH %s tau[%d]: got %.12f exp %.12f\n", name.c_str(), i, tau[i], expected[name][i]);
        ++bad;
      }
    }
  }
  std::printf("riptide_control_core golden: %s (%zu laws vs fixture)\n",
              bad == 0 ? "REPRODUCED" : "FAILED", got.size());
  return bad == 0 ? 0 : 1;
}
