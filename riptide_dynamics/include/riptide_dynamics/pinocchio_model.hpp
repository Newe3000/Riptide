#pragma once

#include <string>
#include <vector>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "riptide_dynamics/dynamics_model_interface.hpp"

namespace riptide
{

/// Pinocchio-backed IDynamicsModel for the (fixed-base) arm, composed with the
/// measured floating-base pose to expose the end-effector in the WORLD frame.
///
/// The arm is loaded from a URDF; any extra joints (e.g. the gripper fingers)
/// are locked so the model is exactly the actuated DoF the controller drives.
/// update() sets q/dq from RobotState, runs FK + Jacobian + M + nonlinear, then
/// rotates the EE pose/Jacobian into the world frame using state.base_pose.
class PinocchioModel : public IDynamicsModel
{
public:
  /// \param urdf_path   arm URDF (fixed base).
  /// \param ee_frame    end-effector frame name (e.g. "fer_hand_tcp").
  /// \param joint_order actuated joint names, in the order the controller uses.
  /// \param locked_joints joints to freeze (e.g. the fingers).
  PinocchioModel(
    const std::string & urdf_path,
    const std::string & ee_frame,
    const std::vector<std::string> & joint_order,
    const std::vector<std::string> & locked_joints);

  void update(const RobotState & state) override;

  const Eigen::MatrixXd & massMatrix() const override { return M_; }
  const Eigen::VectorXd & nonlinear() const override { return nle_; }
  Eigen::MatrixXd jacobian(const std::string & frame) const override;
  Eigen::Isometry3d framePose(const std::string & frame) const override;
  Eigen::VectorXd hydroForces(const RobotState & state) const override;

  std::size_t nJoints() const { return static_cast<std::size_t>(model_.nv); }

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex ee_id_{0};

  // Maps controller joint index -> pinocchio q / v index.
  std::vector<int> q_index_;
  std::vector<int> v_index_;

  // Latest results (world frame for EE quantities).
  Eigen::MatrixXd M_;        ///< joint-space inertia (nv x nv)
  Eigen::VectorXd nle_;      ///< Coriolis + gravity (nv)
  Eigen::MatrixXd J_world_;  ///< EE geometric Jacobian in world (6 x nv)
  Eigen::Isometry3d ee_world_{Eigen::Isometry3d::Identity()};
};

}  // namespace riptide
