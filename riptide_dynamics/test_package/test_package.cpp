// Proves riptide_dynamics runs with NO ROS: build the fer arm model from URDF and
// compute the mass matrix, EE Jacobian, and EE pose through the Conan-built lib
// (which pulls Conan pinocchio/3.8.0 + eigen).
#include "riptide_dynamics/pinocchio_model.hpp"

#include <cstdio>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  const std::string urdf = (argc > 1) ? argv[1] : "fer_arm.urdf";
  const std::vector<std::string> joints = {
    "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
    "fer_joint5", "fer_joint6", "fer_joint7"};
  const std::vector<std::string> locked = {"fer_finger_joint1", "fer_finger_joint2"};

  riptide::PinocchioModel model(urdf, "fer_hand_tcp", joints, locked);

  riptide::RobotState s;
  s.q = (Eigen::VectorXd(7) << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785).finished();
  s.dq = Eigen::VectorXd::Zero(7);
  model.update(s);

  const Eigen::MatrixXd M = model.massMatrix();
  const Eigen::MatrixXd J = model.jacobian("ee");
  const Eigen::Isometry3d X = model.framePose("ee");

  std::printf("riptide_dynamics ok: M=%ldx%ld |J|=%.4f ee=[%.3f %.3f %.3f]\n",
              static_cast<long>(M.rows()), static_cast<long>(M.cols()), J.norm(),
              X.translation().x(), X.translation().y(), X.translation().z());
  return (M.rows() == 7 && M.cols() == 7 && M.allFinite() && J.allFinite()) ? 0 : 1;
}
