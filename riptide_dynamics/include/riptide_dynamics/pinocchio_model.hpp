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
  /// \param base_to_arm constant translation from the measured base link
  ///        (auv_base_link) to the arm root (link0) — the mount offset. Without
  ///        it the composed EE pose is wrong by this amount (the arm sits on top
  ///        of the hull, not at its centre).
  /// \param hydro if true, model per-link hydrodynamic drag (Fossen box-approx)
  ///        and fold it into nonlinear() so the control laws compensate the
  ///        water the arm moves through — closing the model–plant mismatch with
  ///        MuJoCo's fluid model. \param fluid_density, \param fluid_viscosity
  ///        match the MJCF <option>; \param drag_coefficient is the blunt-body Cd.
  PinocchioModel(
    const std::string & urdf_path,
    const std::string & ee_frame,
    const std::vector<std::string> & joint_order,
    const std::vector<std::string> & locked_joints,
    const Eigen::Vector3d & base_to_arm = Eigen::Vector3d::Zero(),
    bool hydro = false,
    double fluid_density = 1000.0,
    double fluid_viscosity = 0.0009,
    double drag_coefficient = 1.0);

  void update(const RobotState & state) override;

  const Eigen::MatrixXd & massMatrix() const override { return M_; }
  /// C(q,dq)dq + g(q), plus the hydrodynamic drag D(dq) when hydro is enabled,
  /// so the control laws (which add nonlinear() as feedforward) compensate it.
  const Eigen::VectorXd & nonlinear() const override { return nle_; }
  Eigen::MatrixXd jacobian(const std::string & frame) const override;
  Eigen::Isometry3d framePose(const std::string & frame) const override;
  /// Generalized hydrodynamic drag D(dq) alone (for introspection / tests).
  Eigen::VectorXd hydroForces(const RobotState & state) const override;

  std::size_t nJoints() const { return static_cast<std::size_t>(model_.nv); }

  /// Joint position limits (from the URDF), in the controller's joint order.
  const Eigen::VectorXd & lowerLimits() const { return q_lower_; }
  const Eigen::VectorXd & upperLimits() const { return q_upper_; }

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex ee_id_{0};

  // Maps controller joint index -> pinocchio q / v index.
  std::vector<int> q_index_;
  std::vector<int> v_index_;

  // Joint position limits in controller order (from the URDF).
  Eigen::VectorXd q_lower_, q_upper_;

  // Constant mount offset: base link (auv_base_link) -> arm root (link0).
  Eigen::Vector3d mount_t_{Eigen::Vector3d::Zero()};

  // Hydrodynamics (per-link drag; see closeModelPlantMismatch in the notes).
  bool hydro_{false};
  double rho_{1000.0};   ///< fluid density [kg/m^3]
  double mu_{0.0009};    ///< dynamic viscosity [Pa.s]
  double cd_{1.0};       ///< blunt-body quadratic-drag coefficient
  Eigen::VectorXd hydro_force_;   ///< last generalized drag D(dq) (nv)

  /// Per-link quadratic + viscous drag mapped to joint space (uses current data_).
  Eigen::VectorXd computeHydroDrag() const;

  // Latest results (world frame for EE quantities).
  Eigen::MatrixXd M_;        ///< joint-space inertia (nv x nv)
  Eigen::VectorXd nle_;      ///< Coriolis + gravity (nv)
  Eigen::MatrixXd J_world_;  ///< EE geometric Jacobian in world (6 x nv)
  Eigen::Isometry3d ee_world_{Eigen::Isometry3d::Identity()};
};

}  // namespace riptide
