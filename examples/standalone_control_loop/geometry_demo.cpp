// The frictionless proof: the eigen-only riptide_geometry leaf, consumed from Conan
// with ZERO ROS and ZERO Pinocchio. Builds a 6-thruster allocation, checks the
// regularized pseudo-inverse reproduces a commanded wrench, and checks an SE(3)
// task-pose error -- the lowest-barrier way to adopt a Riptide core.
#include "riptide_geometry/allocation.hpp"
#include "riptide_geometry/spatial.hpp"

#include <cstdio>
#include <vector>

int main()
{
  // A fully-actuated 7-thruster layout: surge + 4 vertical + 2 lateral at +/-x. The
  // two lateral thrusters make Fy and Mz independently commandable (a single one
  // couples them), so the pseudo-inverse reproduces any 6-D wrench.
  const std::vector<Eigen::Vector3d> pos = {
    {-0.3, 0, 0}, {0.22, 0.22, 0.3}, {0.22, -0.22, 0.3},
    {-0.22, 0.22, 0.3}, {-0.22, -0.22, 0.3}, {0.3, 0, 0}, {-0.3, 0, 0}};
  const std::vector<Eigen::Vector3d> axs = {
    {1, 0, 0}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 1, 0}, {0, 1, 0}};

  const Eigen::MatrixXd A = riptide_geometry::allocation_matrix(pos, axs);
  const Eigen::MatrixXd Ainv = riptide_geometry::regularized_pinv(A);

  Eigen::Matrix<double, 6, 1> w;
  w << 10, 5, 20, 0, 0, 3;                       // a feasible 6-D wrench
  const Eigen::VectorXd u = Ainv * w;            // per-thruster forces
  const double reproduce_err = (A * u - w).norm();

  Eigen::Isometry3d X = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d Xd = Eigen::Isometry3d::Identity();
  Xd.translation() = Eigen::Vector3d(0.1, 0.0, 0.0);
  const double pos_err = riptide_geometry::task_pose_error(X, Xd).head<3>().norm();

  std::printf("geometry_demo: A=%ldx%ld  wrench_reproduce_err=%.2e  pose_err=%.3f\n",
              static_cast<long>(A.rows()), static_cast<long>(A.cols()), reproduce_err, pos_err);

  const bool ok = reproduce_err < 1e-3 && std::abs(pos_err - 0.1) < 1e-9;
  std::printf("geometry_demo: %s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
