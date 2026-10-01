// Exercises the Conan-built riptide_dynamics (-> Conan pinocchio 3.8.0 + boost
// 1.89, hidden-visibility) inside a process that has already initialized apt
// rclcpp. If the two toolchains' Eigen/Boost/libstdc++ coexist, this runs clean;
// an ODR/symbol clash would crash here.
#include <rclcpp/rclcpp.hpp>

#include "riptide_dynamics/pinocchio_model.hpp"

#include <cstdio>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("abi_canary");

  const std::string urdf = (argc > 1) ? argv[1] : "fer_arm.urdf";
  const std::vector<std::string> joints = {
    "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
    "fer_joint5", "fer_joint6", "fer_joint7"};
  const std::vector<std::string> locked = {"fer_finger_joint1", "fer_finger_joint2"};

  riptide::PinocchioModel model(urdf, "fer_hand_tcp", joints, locked);
  riptide::RobotState s;
  s.q = (Eigen::VectorXd(7) << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785).finished();
  s.dq = Eigen::VectorXd::Zero(7);
  model.update(s);                       // runs Conan pinocchio + boost + eigen
  const Eigen::MatrixXd M = model.massMatrix();
  const Eigen::MatrixXd J = model.jacobian("ee");

  RCLCPP_INFO(node->get_logger(),
    "ABI canary OK: apt rclcpp + Conan riptide_dynamics coexist. M=%ldx%ld |J|=%.4f",
    static_cast<long>(M.rows()), static_cast<long>(M.cols()), J.norm());
  rclcpp::shutdown();
  return (M.allFinite() && J.allFinite()) ? 0 : 1;
}
