// Mirrors the exact Pinocchio calls riptide_dynamics::PinocchioModel makes, so a
// clean compile+run against pinocchio/3.8.0 proves the 3.8.0 API covers Riptide's
// needs (the local from-source build is 4.1.0 -- this is the version-delta spike).
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/model.hpp>

#include <cstdio>
#include <string>

int main(int argc, char ** argv)
{
  const std::string urdf = (argc > 1) ? argv[1] : "fer_arm.urdf";
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf, model);          // parsers/urdf.hpp
  model.gravity.linear(Eigen::Vector3d::Zero());     // neutral-buoyancy match

  pinocchio::Data data(model);
  Eigen::VectorXd q = pinocchio::neutral(model);     // joint-configuration.hpp
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

  pinocchio::forwardKinematics(model, data, q, v);   // kinematics.hpp
  pinocchio::updateFramePlacements(model, data);     // frames.hpp
  pinocchio::computeJointJacobians(model, data, q);  // jacobian.hpp
  pinocchio::crba(model, data, q);                   // crba.hpp -> M
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, q, v);    // rnea.hpp -> C(q,dq)dq+g

  const std::string ee = "fer_hand_tcp";
  if (!model.existFrame(ee)) { std::printf("frame '%s' missing\n", ee.c_str()); return 1; }
  const auto fid = model.getFrameId(ee);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv);
  pinocchio::getFrameJacobian(model, data, fid, pinocchio::LOCAL_WORLD_ALIGNED, J);

  std::printf("pinocchio ok: nq=%d nv=%d M(0,0)=%.4f |nle|=%.4f |J_ee|=%.4f\n",
              model.nq, model.nv, data.M(0, 0), data.nle.norm(), J.norm());
  return 0;
}
