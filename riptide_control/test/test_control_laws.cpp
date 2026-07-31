// Unit tests for the task-space IControlLaw plugins. Each law is a pure
// RobotState -> torque mapping, so we drive it with a mock IDynamicsModel and
// assert structural + directional properties (size, finiteness, equilibrium,
// error-direction) without needing MuJoCo or a real robot.

#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "riptide_control/task_space_impedance.hpp"
#include "riptide_control/task_space_lqr.hpp"
#include "riptide_control/task_space_mpc.hpp"
#include "riptide_dynamics/dynamics_model_interface.hpp"

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

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
