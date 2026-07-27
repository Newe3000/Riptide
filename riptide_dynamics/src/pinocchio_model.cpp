#include "riptide_dynamics/pinocchio_model.hpp"

#include <stdexcept>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace riptide
{

PinocchioModel::PinocchioModel(
  const std::string & urdf_path,
  const std::string & ee_frame,
  const std::vector<std::string> & joint_order,
  const std::vector<std::string> & locked_joints)
{
  pinocchio::Model full;
  pinocchio::urdf::buildModel(urdf_path, full);  // fixed base

  // Lock the requested joints (e.g. the gripper fingers) at the neutral config.
  std::vector<pinocchio::JointIndex> lock;
  for (const auto & name : locked_joints)
  {
    if (full.existJointName(name)) { lock.push_back(full.getJointId(name)); }
  }
  const Eigen::VectorXd q_ref = pinocchio::neutral(full);
  if (lock.empty())
  {
    model_ = full;
  }
  else
  {
    pinocchio::buildReducedModel(full, lock, q_ref, model_);
  }
  data_ = pinocchio::Data(model_);

  if (!model_.existFrame(ee_frame))
  {
    throw std::runtime_error("PinocchioModel: EE frame '" + ee_frame + "' not found.");
  }
  ee_id_ = model_.getFrameId(ee_frame);

  // Resolve the controller's joint order to pinocchio q/v indices.
  q_index_.resize(joint_order.size());
  v_index_.resize(joint_order.size());
  for (std::size_t i = 0; i < joint_order.size(); ++i)
  {
    if (!model_.existJointName(joint_order[i]))
    {
      throw std::runtime_error("PinocchioModel: joint '" + joint_order[i] + "' not in model.");
    }
    const auto jid = model_.getJointId(joint_order[i]);
    q_index_[i] = model_.idx_qs[jid];
    v_index_[i] = model_.idx_vs[jid];
  }

  M_ = Eigen::MatrixXd::Zero(model_.nv, model_.nv);
  nle_ = Eigen::VectorXd::Zero(model_.nv);
  J_world_ = Eigen::MatrixXd::Zero(6, model_.nv);
}

void PinocchioModel::update(const RobotState & state)
{
  Eigen::VectorXd q = pinocchio::neutral(model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  const std::size_t n = q_index_.size();
  for (std::size_t i = 0; i < n && i < static_cast<std::size_t>(state.q.size()); ++i)
  {
    q[q_index_[i]] = state.q[i];
    v[v_index_[i]] = state.dq[i];
  }

  pinocchio::forwardKinematics(model_, data_, q, v);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeJointJacobians(model_, data_, q);

  Eigen::MatrixXd J_local = Eigen::MatrixXd::Zero(6, model_.nv);
  pinocchio::getFrameJacobian(
    model_, data_, ee_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_local);

  pinocchio::crba(model_, data_, q);
  data_.M.triangularView<Eigen::StrictlyLower>() =
    data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  M_ = data_.M;

  pinocchio::nonLinearEffects(model_, data_, q, v);
  nle_ = data_.nle;

  // Compose the fixed-base EE quantities with the measured base pose (world).
  const pinocchio::SE3 & oMf = data_.oMf[ee_id_];   // EE in base frame
  const Eigen::Matrix3d R_wb = state.base_pose.rotation();
  ee_world_.linear() = R_wb * oMf.rotation();
  ee_world_.translation() = state.base_pose.translation() + R_wb * oMf.translation();

  // LOCAL_WORLD_ALIGNED is expressed in a base-aligned world frame; rotate the
  // linear and angular blocks by R_wb to get the true world Jacobian.
  J_world_.topRows(3) = R_wb * J_local.topRows(3);
  J_world_.bottomRows(3) = R_wb * J_local.bottomRows(3);
}

Eigen::MatrixXd PinocchioModel::jacobian(const std::string & /*frame*/) const
{
  return J_world_;
}

Eigen::Isometry3d PinocchioModel::framePose(const std::string & /*frame*/) const
{
  return ee_world_;
}

Eigen::VectorXd PinocchioModel::hydroForces(const RobotState & /*state*/) const
{
  return Eigen::VectorXd::Zero(model_.nv);
}

}  // namespace riptide
