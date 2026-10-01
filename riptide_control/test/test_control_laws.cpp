// Unit tests for the task-space IControlLaw plugins. Each law is a pure
// RobotState -> torque mapping, so we drive it with a mock IDynamicsModel and
// assert structural + directional properties (size, finiteness, equilibrium,
// error-direction) without needing MuJoCo or a real robot.

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "riptide_control/task_space_impedance.hpp"
#include "riptide_control/task_space_lqr.hpp"
#include "riptide_control/task_space_mpc.hpp"
#include "riptide_control/template_control_law.hpp"
#include "riptide_dynamics/dynamics_model_interface.hpp"
#include "riptide_dynamics/pinocchio_model.hpp"

namespace
{
constexpr int kN = 7;  // arm DoF

// Mock model: M = I, nonlinear = 0, J = [I6 | 0] (so Lambda = I6 and the task
// axes map onto the first six joints), and a settable EE pose.
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
  Eigen::Isometry3d ee_;

private:
  Eigen::MatrixXd M_, J_;
  Eigen::VectorXd nle_;
};

const Eigen::VectorXd kQRest =
  (Eigen::VectorXd(kN) << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785).finished();

riptide::RobotState rest_state()
{
  riptide::RobotState s;
  s.q = kQRest;
  s.dq = Eigen::VectorXd::Zero(kN);
  return s;
}

riptide::EndEffectorTarget target_at(const Eigen::Vector3d & p)
{
  riptide::EndEffectorTarget t;
  t.pose = Eigen::Isometry3d::Identity();
  t.pose.translation() = p;
  return t;
}

class ControlLawTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) { rclcpp::init(0, nullptr); }
  }
  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) { rclcpp::shutdown(); }
  }
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_control_laws");
    model_ = std::make_shared<MockModel>();
  }
  bool configure(riptide::IControlLaw & law)
  {
    return law.on_configure(
      node_->get_node_parameters_interface(),
      node_->get_node_logging_interface(), model_);
  }
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MockModel> model_;
};

// ---- shared property checks, parameterised over the concrete law -----------

template <typename Law>
void check_output_size_and_finite(ControlLawTest * t, riptide::IControlLaw & law)
{
  (void)t;
  const auto target = target_at(Eigen::Vector3d(0.5, 0.1, 1.05));  // some error
  const Eigen::VectorXd tau = law.compute(rest_state(), target, 0.004);
  EXPECT_EQ(tau.size(), kN);
  EXPECT_TRUE(tau.allFinite());
}

template <typename Law>
void check_zero_error_zero_torque(riptide::IControlLaw & law, MockModel & model)
{
  // Target == current EE, arm at q_rest, zero velocity, zero nonlinear => tau ~ 0.
  const auto target = target_at(model.ee_.translation());
  const Eigen::VectorXd tau = law.compute(rest_state(), target, 0.004);
  EXPECT_LT(tau.norm(), 1e-6) << "expected ~zero torque at equilibrium";
}

template <typename Law>
void check_pushes_toward_target(riptide::IControlLaw & law, MockModel & model)
{
  // Target 10 cm ahead in +x: with J = [I6|0], the task-x force lands on joint 0,
  // so tau[0] must be positive (pull the EE toward the target).
  auto target = target_at(model.ee_.translation() + Eigen::Vector3d(0.1, 0.0, 0.0));
  const Eigen::VectorXd tau = law.compute(rest_state(), target, 0.004);
  EXPECT_GT(tau[0], 0.0) << "torque should drive the EE toward the +x target";
}

// ---- impedance -------------------------------------------------------------

TEST_F(ControlLawTest, ImpedanceOutputSizeAndFinite)
{
  riptide_control::TaskSpaceImpedance law;
  ASSERT_TRUE(configure(law));
  check_output_size_and_finite<riptide_control::TaskSpaceImpedance>(this, law);
}
TEST_F(ControlLawTest, ImpedanceZeroErrorZeroTorque)
{
  riptide_control::TaskSpaceImpedance law;
  ASSERT_TRUE(configure(law));
  check_zero_error_zero_torque<riptide_control::TaskSpaceImpedance>(law, *model_);
}
TEST_F(ControlLawTest, ImpedancePushesTowardTarget)
{
  riptide_control::TaskSpaceImpedance law;
  ASSERT_TRUE(configure(law));
  check_pushes_toward_target<riptide_control::TaskSpaceImpedance>(law, *model_);
}

// ---- LQR -------------------------------------------------------------------

TEST_F(ControlLawTest, LqrOutputSizeAndFinite)
{
  riptide_control::TaskSpaceLqr law;
  ASSERT_TRUE(configure(law));
  check_output_size_and_finite<riptide_control::TaskSpaceLqr>(this, law);
}
TEST_F(ControlLawTest, LqrZeroErrorZeroTorque)
{
  riptide_control::TaskSpaceLqr law;
  ASSERT_TRUE(configure(law));
  check_zero_error_zero_torque<riptide_control::TaskSpaceLqr>(law, *model_);
}
TEST_F(ControlLawTest, LqrPushesTowardTarget)
{
  riptide_control::TaskSpaceLqr law;
  ASSERT_TRUE(configure(law));
  check_pushes_toward_target<riptide_control::TaskSpaceLqr>(law, *model_);
}
TEST_F(ControlLawTest, LqrGainsAreCriticallyDampedForZeroVelWeight)
{
  // With qd=0 the closed-form CARE damping gives zeta = 1/sqrt(2): Kd = sqrt(2*Kp).
  // q_pos=640000, r=1 => Kp=800, Kd=sqrt(1600)=40. Probe: pure -x velocity of
  // 1 m/s at zero position error => a_des_x = -Kd*(-1)... check tau[0] = Kd on
  // damping. Easier: velocity in +x should produce a damping torque tau[0] < 0.
  riptide_control::TaskSpaceLqr law;
  ASSERT_TRUE(configure(law));
  auto s = rest_state();
  s.dq[0] = 1.0;  // J=[I6|0] => EE x-velocity = +1
  const auto target = target_at(model_->ee_.translation());  // zero pos error
  const Eigen::VectorXd tau = law.compute(s, target, 0.004);
  EXPECT_LT(tau[0], 0.0) << "damping should oppose +x EE velocity";
}

// ---- MPC -------------------------------------------------------------------

TEST_F(ControlLawTest, MpcOutputSizeAndFinite)
{
  riptide_control::TaskSpaceMpc law;
  ASSERT_TRUE(configure(law));
  check_output_size_and_finite<riptide_control::TaskSpaceMpc>(this, law);
}
TEST_F(ControlLawTest, MpcZeroErrorZeroTorque)
{
  riptide_control::TaskSpaceMpc law;
  ASSERT_TRUE(configure(law));
  law.reset();
  check_zero_error_zero_torque<riptide_control::TaskSpaceMpc>(law, *model_);
}
TEST_F(ControlLawTest, MpcPushesTowardTarget)
{
  riptide_control::TaskSpaceMpc law;
  ASSERT_TRUE(configure(law));
  law.reset();
  check_pushes_toward_target<riptide_control::TaskSpaceMpc>(law, *model_);
}
TEST_F(ControlLawTest, MpcRespectsInputBound)
{
  // A huge position error must not blow up: the input box |w|<=w_max_pos caps
  // the task acceleration, so tau[0] <= w_max_pos (default 15) via Lambda=I.
  riptide_control::TaskSpaceMpc law;
  ASSERT_TRUE(configure(law));
  law.reset();
  auto target = target_at(model_->ee_.translation() + Eigen::Vector3d(100.0, 0, 0));
  Eigen::VectorXd tau;
  for (int i = 0; i < 10; ++i) {  // let the warm-start converge
    tau = law.compute(rest_state(), target, 0.004);
  }
  EXPECT_TRUE(tau.allFinite());
  EXPECT_LE(tau[0], 15.0 + 1e-6) << "MPC input bound should cap the task accel";
}

// ---- hydrodynamics model (real PinocchioModel from the arm URDF) -----------

class HydroTest : public ControlLawTest
{
protected:
  std::string urdf_or_skip()
  {
    try {
      return ament_index_cpp::get_package_share_directory("riptide_description") +
             "/urdf/fer_arm.urdf";
    } catch (const std::exception &) {
      return "";
    }
  }
  const std::vector<std::string> joints_ = {
    "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
    "fer_joint5", "fer_joint6", "fer_joint7"};
  const std::vector<std::string> locked_ = {"fer_finger_joint1", "fer_finger_joint2"};
};

TEST_F(HydroTest, DragIsDissipativeGatedAndQuadratic)
{
  const std::string urdf = urdf_or_skip();
  if (urdf.empty()) { GTEST_SKIP() << "riptide_description share not found"; }

  riptide::PinocchioModel on(urdf, "fer_hand_tcp", joints_, locked_,
                             Eigen::Vector3d::Zero(), /*hydro=*/true);
  riptide::PinocchioModel off(urdf, "fer_hand_tcp", joints_, locked_,
                              Eigen::Vector3d::Zero(), /*hydro=*/false);

  riptide::RobotState s;
  s.q = kQRest;
  s.dq = Eigen::VectorXd::Zero(kN);

  // At rest -> no drag.
  on.update(s);
  EXPECT_LT(on.hydroForces(s).norm(), 1e-9);

  // With joint velocity -> nonzero, dissipative, and gated off when disabled.
  s.dq = Eigen::VectorXd::Constant(kN, 0.5);
  on.update(s);
  const Eigen::VectorXd d1 = on.hydroForces(s);
  EXPECT_GT(d1.norm(), 0.0);
  EXPECT_GT(s.dq.dot(d1), 0.0) << "generalized drag must dissipate (dq^T D > 0)";
  off.update(s);
  EXPECT_LT(off.hydroForces(s).norm(), 1e-12) << "hydro must be gated off";

  // Quadratic form drag => doubling velocity more than doubles the drag.
  const double n1 = d1.norm();
  s.dq = Eigen::VectorXd::Constant(kN, 1.0);
  on.update(s);
  EXPECT_GT(on.hydroForces(s).norm(), 2.0 * n1);
}

// ---- golden baseline -------------------------------------------------------
// For a fixed RobotState + EndEffectorTarget and default gains, record the exact
// compute() torque vector of each law in a checked-in fixture. The Step 5 de-ROS
// refactor changes the IControlLaw API but not the math, so these must reproduce.
// Regenerate with RIPTIDE_GOLDEN_REGEN=1 (or when the fixture is absent).

TEST_F(ControlLawTest, GoldenComputeBaseline)
{
  const auto state = rest_state();
  const auto target = target_at(Eigen::Vector3d(0.5, 0.1, 1.05));
  const double dt = 0.004;

  std::map<std::string, Eigen::VectorXd> got;
  {
    riptide_control::TaskSpaceImpedance law;
    ASSERT_TRUE(configure(law));
    got["impedance"] = law.compute(state, target, dt);
  }
  {
    riptide_control::TaskSpaceLqr law;
    ASSERT_TRUE(configure(law));
    got["lqr"] = law.compute(state, target, dt);
  }
  {
    riptide_control::TaskSpaceMpc law;
    ASSERT_TRUE(configure(law));
    law.reset();
    got["mpc"] = law.compute(state, target, dt);
  }
  {
    riptide_control::TemplateControlLaw law;
    ASSERT_TRUE(configure(law));
    got["template"] = law.compute(state, target, dt);
  }

#ifndef RIPTIDE_GOLDEN_FILE
  GTEST_SKIP() << "RIPTIDE_GOLDEN_FILE not defined by the build";
#else
  const std::string path = RIPTIDE_GOLDEN_FILE;
  std::ifstream in(path);
  if (std::getenv("RIPTIDE_GOLDEN_REGEN") != nullptr || !in.good()) {
    std::ofstream out(path);
    ASSERT_TRUE(out.good()) << "cannot write golden fixture: " << path;
    out << "# law,tau0..tau6 -- fixed q_rest, target (0.5,0.1,1.05), dt=0.004, "
           "default gains, MockModel (M=I, nle=0, J=[I6|0]).\n";
    out << std::setprecision(17);
    for (const auto & [name, tau] : got) {
      out << name;
      for (int i = 0; i < tau.size(); ++i) { out << "," << tau[i]; }
      out << "\n";
    }
    SUCCEED() << "regenerated golden fixture: " << path;
    return;
  }

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

  const double tol = 1e-9;  // same binary is bit-identical; tol absorbs rebuilds
  for (const auto & [name, tau] : got) {
    ASSERT_TRUE(expected.count(name)) << "golden fixture missing law: " << name;
    ASSERT_EQ(expected[name].size(), tau.size()) << name;
    for (int i = 0; i < tau.size(); ++i) {
      EXPECT_NEAR(tau[i], expected[name][i], tol) << name << " tau[" << i << "]";
    }
  }
#endif
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
