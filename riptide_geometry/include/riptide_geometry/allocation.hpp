#pragma once

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace riptide_geometry
{

/// Build the 6xN wrench allocation matrix: column i is thruster i's unit wrench
/// [axis; r x axis] -- rows (Fx,Fy,Fz,Mx,My,Mz) -- for N thrusters at body-frame
/// positions r_i with unit force axes axis_i.
inline Eigen::MatrixXd allocation_matrix(
  const std::vector<Eigen::Vector3d> & positions,
  const std::vector<Eigen::Vector3d> & axes)
{
  const Eigen::Index n = static_cast<Eigen::Index>(positions.size());
  Eigen::MatrixXd A(6, n);
  for (Eigen::Index i = 0; i < n; ++i)
  {
    const Eigen::Vector3d & r = positions[static_cast<std::size_t>(i)];
    const Eigen::Vector3d & a = axes[static_cast<std::size_t>(i)];
    const Eigen::Vector3d m = r.cross(a);
    A(0, i) = a.x(); A(1, i) = a.y(); A(2, i) = a.z();
    A(3, i) = m.x(); A(4, i) = m.y(); A(5, i) = m.z();
  }
  return A;
}

/// Regularized right pseudo-inverse A^T (A A^T + reg I)^-1 (maps a desired 6-D
/// wrench to per-thruster forces). The Tikhonov term keeps the inverse well-posed
/// when a DOF is weakly actuated.
inline Eigen::MatrixXd regularized_pinv(const Eigen::MatrixXd & A, double reg = 1e-6)
{
  const Eigen::Index m = A.rows();
  const Eigen::MatrixXd AAt = A * A.transpose();
  return A.transpose() * (AAt + reg * Eigen::MatrixXd::Identity(m, m)).inverse();
}

}  // namespace riptide_geometry
