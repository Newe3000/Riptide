#include "riptide_dynamics/pinocchio_model.hpp"

#include <cmath>
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
  const std::vector<std::string> & locked_joints,
  const Eigen::Vector3d & base_to_arm,
  bool hydro, double fluid_density, double fluid_viscosity, double drag_coefficient)
: mount_t_(base_to_arm),
  hydro_(hydro), rho_(fluid_density), mu_(fluid_viscosity), cd_(drag_coefficient)
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
  // Match the plant: the AUV floats in neutral buoyancy, modelled as zero
  // gravity in the MJCF. Pinocchio defaults to -9.81 m/s^2, so leaving it would
  // make nonLinearEffects() return a spurious gravity-compensation torque that
  // the control laws add with nothing in the plant to cancel it (the arm drifts
  // "up"). Zero it so nonlinear() = C(q,dq)dq only, consistent with the sim.
  model_.gravity.linear(Eigen::Vector3d::Zero());

  data_ = pinocchio::Data(model_);

  if (!model_.existFrame(ee_frame))
  {
    throw std::runtime_error("PinocchioModel: EE frame '" + ee_frame + "' not found.");
  }
  ee_id_ = model_.getFrameId(ee_frame);

  // Resolve the controller's joint order to pinocchio q/v indices, and pull the
  // position limits (from the URDF) in that same order.
  q_index_.resize(joint_order.size());
  v_index_.resize(joint_order.size());
  q_lower_.resize(joint_order.size());
  q_upper_.resize(joint_order.size());
  for (std::size_t i = 0; i < joint_order.size(); ++i)
  {
    if (!model_.existJointName(joint_order[i]))
    {
      throw std::runtime_error("PinocchioModel: joint '" + joint_order[i] + "' not in model.");
    }
    const auto jid = model_.getJointId(joint_order[i]);
    q_index_[i] = model_.idx_qs[jid];
    v_index_[i] = model_.idx_vs[jid];
    q_lower_[i] = model_.lowerPositionLimit[q_index_[i]];
    q_upper_[i] = model_.upperPositionLimit[q_index_[i]];
  }

  M_ = Eigen::MatrixXd::Zero(model_.nv, model_.nv);
  nle_ = Eigen::VectorXd::Zero(model_.nv);
  J_world_ = Eigen::MatrixXd::Zero(6, model_.nv);
  hydro_force_ = Eigen::VectorXd::Zero(model_.nv);
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

  // Hydrodynamic drag the arm moves through (closes the mismatch with MuJoCo's
  // fluid model). Fold it into nonlinear() so every control law compensates it.
  if (hydro_)
  {
    hydro_force_ = computeHydroDrag();
    nle_ += hydro_force_;
  }
  else
  {
    hydro_force_.setZero();
  }

  // Compose the fixed-base EE quantities with the measured base pose (world).
  // oMf is the EE in the ARM ROOT (link0) frame; the arm root is mounted at
  // base_pose ∘ mount_t_ (link0 sits on top of the hull), so add the mount
  // offset before rotating into world. The mount is a pure, constant translation
  // (no rotation), so the Jacobian mapping is unchanged.
  const pinocchio::SE3 & oMf = data_.oMf[ee_id_];   // EE in arm-root frame
  const Eigen::Matrix3d R_wb = state.base_pose.rotation();
  ee_world_.linear() = R_wb * oMf.rotation();
  ee_world_.translation() =
    state.base_pose.translation() + R_wb * (mount_t_ + oMf.translation());

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
  return hydro_force_;   // as computed by the last update()
}

Eigen::VectorXd PinocchioModel::computeHydroDrag() const
{
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(model_.nv);
  Eigen::MatrixXd Ji(6, model_.nv);
  for (pinocchio::JointIndex i = 1;
       i < static_cast<pinocchio::JointIndex>(model_.njoints); ++i)
  {
    const pinocchio::Inertia & Y = model_.inertias[i];
    const double m = Y.mass();
    if (m < 1e-9) { continue; }

    // Equivalent uniform box half-sizes from the link's (diagonal) inertia:
    //   I_x = m/3 (hy^2 + hz^2), ...  =>  hx^2 = 1.5*sum(I)/m - 3 I_x/m.
    const Eigen::Matrix3d Ibar = Y.inertia().matrix();
    const Eigen::Vector3d Id(Ibar(0, 0), Ibar(1, 1), Ibar(2, 2));
    const double S = 1.5 * Id.sum() / m;
    Eigen::Vector3d h;
    for (int k = 0; k < 3; ++k) { h[k] = std::sqrt(std::max(S - 3.0 * Id[k] / m, 0.0)); }

    // Link spatial velocity in the local (joint) frame.
    const Eigen::Vector3d vl = data_.v[i].linear();
    const Eigen::Vector3d vw = data_.v[i].angular();

    // Drag as the dissipative D(dq)dq term in the LHS convention
    //   M ddq + C dq + g + D(dq)dq = tau,
    // i.e. aligned WITH the link velocity, so adding it to tau cancels the fluid
    // force f = -D(dq)dq the plant applies. Quadratic (form) + linear (viscous)
    // on the box faces for translation; a Stokes-like viscous term for rotation.
    Eigen::Matrix<double, 6, 1> w;
    for (int k = 0; k < 3; ++k)
    {
      const int a = (k + 1) % 3, b = (k + 2) % 3;
      const double area = 4.0 * h[a] * h[b];                 // face _|_ axis k
      const double c_quad = 0.5 * rho_ * cd_ * area;
      const double c_visc = 3.0 * M_PI * mu_ * (h[a] + h[b]);
      w[k] = (c_quad * std::abs(vl[k]) + c_visc) * vl[k];
    }
    const double r = (h[0] + h[1] + h[2]) / 3.0;
    const double c_rot = 8.0 * M_PI * mu_ * r * r * r;        // Stokes rotational
    for (int k = 0; k < 3; ++k) { w[3 + k] = c_rot * vw[k]; }

    // Map the local-frame drag wrench to joint torques: tau += J_i,local^T w.
    Ji.setZero();
    pinocchio::getJointJacobian(model_, data_, i, pinocchio::LOCAL, Ji);
    tau += Ji.transpose() * w;
  }
  return tau;
}

}  // namespace riptide
