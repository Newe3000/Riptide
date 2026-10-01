// Exercises the eigen-only geometry leaf with no ROS: build a fully-actuated
// 6-thruster allocation, check the pseudo-inverse reproduces a commanded wrench,
// and check a known SE(3) pose error.
#include "riptide_geometry/allocation.hpp"
#include "riptide_geometry/spatial.hpp"

#include <cstdio>
#include <vector>

int main()
{
  // Surge + 4 vertical + 2 lateral (the Riptide layout) -> full 6-DOF.
  std::vector<Eigen::Vector3d> pos = {
    {-0.3, 0, 0}, {0.22, 0.22, 0.3}, {0.22, -0.22, 0.3},
    {-0.22, 0.22, 0.3}, {-0.22, -0.22, 0.3}, {0.3, 0, 0}, {-0.3, 0, 0}};
  std::vector<Eigen::Vector3d> axs = {
    {1, 0, 0}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 1, 0}, {0, 1, 0}};

  const Eigen::MatrixXd A = riptide_geometry::allocation_matrix(pos, axs);
  const Eigen::MatrixXd Ainv = riptide_geometry::regularized_pinv(A);
  Eigen::Matrix<double, 6, 1> w;
  w << 10, 5, 20, 0, 0, 3;                 // a feasible 6-D wrench
  const Eigen::VectorXd u = Ainv * w;      // per-thruster forces
  const double wrench_err = (A * u - w).norm();

  Eigen::Isometry3d X = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d Xd = Eigen::Isometry3d::Identity();
  Xd.translation() = Eigen::Vector3d(0.1, 0.0, 0.0);
  const double pos_err = riptide_geometry::task_pose_error(X, Xd).head<3>().norm();

  std::printf("riptide_geometry ok: A=%ldx%ld wrench_reproduce_err=%.2e pose_err=%.3f\n",
              static_cast<long>(A.rows()), static_cast<long>(A.cols()), wrench_err, pos_err);
  // The regularized pinv trades exactness for conditioning, so the reproduction
  // residual is O(reg * |w|) (~2e-5 here), not machine zero.
  return (wrench_err < 1e-3 && std::abs(pos_err - 0.1) < 1e-9) ? 0 : 1;
}
