#include <Eigen/Dense>
#include <cstdio>
int main() {
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity() * 2.0;
  Eigen::Vector3d b(1, 2, 3);
  Eigen::Vector3d x = A.ldlt().solve(b);   // exercises a real Eigen op
  std::printf("eigen ok: x = [%.3f %.3f %.3f]\n", x[0], x[1], x[2]);
  return (x - Eigen::Vector3d(0.5, 1.0, 1.5)).norm() < 1e-12 ? 0 : 1;
}
