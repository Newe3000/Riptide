#include "riptide_control/task_space_mpc.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace riptide_control
{
namespace
{
using ParamsIface = rclcpp::node_interfaces::NodeParametersInterface;

template <typename T>
T get_or_declare(const ParamsIface::SharedPtr & params, const std::string & name, const T & def)
{
  if (!params->has_parameter(name))
  {
    params->declare_parameter(name, rclcpp::ParameterValue(def));
  }
  return params->get_parameter(name).get_value<T>();
}

Eigen::VectorXd to_vec(const std::vector<double> & v)
{
  return Eigen::Map<const Eigen::VectorXd>(v.data(), static_cast<Eigen::Index>(v.size()));
}

// Discrete algebraic Riccati equation for the scalar-input double integrator,
// solved by iterating the Riccati recursion to a fixed point (2x2, converges
// fast). Used as the MPC terminal cost so the scheme is stabilizing.
Eigen::Matrix2d dare(const Eigen::Matrix2d & A, const Eigen::Vector2d & B,
                     const Eigen::Matrix2d & Q, double r)
{
  Eigen::Matrix2d P = Q;
  for (int it = 0; it < 5000; ++it)
  {
    const double s = r + B.dot(P * B);                       // scalar
    const Eigen::RowVector2d K = (B.transpose() * P * A) / s; // 1x2
    const Eigen::Matrix2d Pn = A.transpose() * P * A
                             - A.transpose() * P * B * K + Q;
    if ((Pn - P).cwiseAbs().maxCoeff() < 1e-12) { return Pn; }
    P = Pn;
  }
  return P;
}
}  // namespace

bool TaskSpaceMpc::on_configure(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging,
  std::shared_ptr<riptide::IDynamicsModel> model)
{
  model_ = std::move(model);

  N_ = get_or_declare<int>(params, "mpc.horizon", 20);
  T_ = get_or_declare<double>(params, "mpc.sample_time", 0.004);
  max_iter_ = get_or_declare<int>(params, "mpc.max_iter", 40);

  const auto q_pos = get_or_declare<std::vector<double>>(params, "mpc.q_pos", {640000.0, 640000.0, 640000.0});
  const auto q_ori = get_or_declare<std::vector<double>>(params, "mpc.q_ori", {3600.0, 3600.0, 3600.0});
  const auto qd_pos = get_or_declare<std::vector<double>>(params, "mpc.qd_pos", {0.0, 0.0, 0.0});
  const auto qd_ori = get_or_declare<std::vector<double>>(params, "mpc.qd_ori", {0.0, 0.0, 0.0});
  const auto r_pos = get_or_declare<std::vector<double>>(params, "mpc.r_pos", {1.0, 1.0, 1.0});
  const auto r_ori = get_or_declare<std::vector<double>>(params, "mpc.r_ori", {1.0, 1.0, 1.0});
  const auto wmax_pos = get_or_declare<std::vector<double>>(params, "mpc.w_max_pos", {15.0, 15.0, 15.0});
  const auto wmax_ori = get_or_declare<std::vector<double>>(params, "mpc.w_max_ori", {40.0, 40.0, 40.0});

  const std::array<double, 6> qp{q_pos[0], q_pos[1], q_pos[2], q_ori[0], q_ori[1], q_ori[2]};
  const std::array<double, 6> qv{qd_pos[0], qd_pos[1], qd_pos[2], qd_ori[0], qd_ori[1], qd_ori[2]};
  const std::array<double, 6> rr{r_pos[0], r_pos[1], r_pos[2], r_ori[0], r_ori[1], r_ori[2]};
  const std::array<double, 6> wm{wmax_pos[0], wmax_pos[1], wmax_pos[2], wmax_ori[0], wmax_ori[1], wmax_ori[2]};

  // Double integrator: A=[[1,T],[0,1]], B=[T^2/2; T].
  Eigen::Matrix2d A;
  A << 1.0, T_, 0.0, 1.0;
  Eigen::Vector2d B(0.5 * T_ * T_, T_);

  const int N = N_;
  for (int a = 0; a < 6; ++a)
  {
    Axis & ax = axis_[a];
    Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
    Q(0, 0) = qp[a];
    Q(1, 1) = qv[a];
    const double r = rr[a];
    const Eigen::Matrix2d P = dare(A, B, Q, r);   // terminal cost

    // Condense: stack z_k = Sx z0 + Su U for k=1..N.
    Eigen::MatrixXd Sx(2 * N, 2);
    Eigen::MatrixXd Su = Eigen::MatrixXd::Zero(2 * N, N);
    Eigen::Matrix2d Apow = A;                     // A^k, starts at A^1
    // Precompute A^{i} B for i>=0.
    std::vector<Eigen::Vector2d> AiB(N);
    AiB[0] = B;
    for (int i = 1; i < N; ++i) { AiB[i] = A * AiB[i - 1]; }
    for (int k = 1; k <= N; ++k)
    {
      Sx.block(2 * (k - 1), 0, 2, 2) = Apow;
      for (int j = 0; j < k; ++j)
      {
        Su.block(2 * (k - 1), j, 2, 1) = AiB[k - 1 - j];
      }
      Apow = A * Apow;
    }

    // Block-diagonal weights: Q for z_1..z_{N-1}, P for the terminal z_N.
    Eigen::MatrixXd Qblk = Eigen::MatrixXd::Zero(2 * N, 2 * N);
    for (int k = 1; k <= N; ++k)
    {
      Qblk.block(2 * (k - 1), 2 * (k - 1), 2, 2) = (k == N) ? P : Q;
    }
    const Eigen::MatrixXd Rblk = r * Eigen::MatrixXd::Identity(N, N);

    // J(U) = 1/2 U^T H U + (G z0)^T U + const.
    ax.H = 2.0 * (Su.transpose() * Qblk * Su + Rblk);
    ax.H = 0.5 * (ax.H + ax.H.transpose());       // symmetrize
    ax.G = 2.0 * Su.transpose() * Qblk * Sx;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(ax.H);
    ax.L = es.eigenvalues().maxCoeff();
    ax.mu = std::max(es.eigenvalues().minCoeff(), 1e-9);
    ax.w_max = wm[a];
    ax.warm = Eigen::VectorXd::Zero(N);
    ax.g = Eigen::VectorXd::Zero(N);
    ax.y = Eigen::VectorXd::Zero(N);
    ax.u_new = Eigen::VectorXd::Zero(N);
    ax.grad = Eigen::VectorXd::Zero(N);
  }

  null_kp_ = get_or_declare<double>(params, "mpc.null_kp", 5.0);
  null_kd_ = get_or_declare<double>(params, "mpc.null_kd", 1.0);
  jacobian_damping_ = get_or_declare<double>(params, "mpc.jacobian_damping", 1e-3);
  q_rest_ = to_vec(get_or_declare<std::vector<double>>(
    params, "mpc.q_rest", {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}));
  max_effort_ = to_vec(get_or_declare<std::vector<double>>(
    params, "mpc.max_effort", {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0}));

  RCLCPP_INFO(logging->get_logger(),
    "TaskSpaceMpc configured: horizon N=%d, T=%.4f s, %d QP iters/axis, "
    "input bounds |w|<=[%.0f pos, %.0f ori].", N_, T_, max_iter_, wm[0], wm[3]);
  return true;
}

// Warm-started projected fast-gradient (Nesterov) for the box-constrained QP
// min 1/2 U^T H U + g^T U  s.t.  |U_i| <= w_max. Returns the first input U[0].
double TaskSpaceMpc::solve_axis(Axis & ax, double e, double e_dot)
{
  const Eigen::Vector2d z0(e, e_dot);
  ax.g.noalias() = ax.G * z0;

  const double beta = (std::sqrt(ax.L) - std::sqrt(ax.mu)) /
                      (std::sqrt(ax.L) + std::sqrt(ax.mu));
  Eigen::VectorXd & U = ax.warm;   // warm-start from the previous cycle
  ax.y = U;
  for (int it = 0; it < max_iter_; ++it)
  {
    ax.grad.noalias() = ax.H * ax.y + ax.g;
    ax.u_new = (ax.y - ax.grad / ax.L).cwiseMax(-ax.w_max).cwiseMin(ax.w_max);
    ax.y = ax.u_new + beta * (ax.u_new - U);
    U = ax.u_new;
  }
  const double w0 = U[0];

  // Shift the warm-start for the next cycle (drop the applied move, repeat last).
  const int N = static_cast<int>(U.size());
  for (int i = 0; i < N - 1; ++i) { U[i] = U[i + 1]; }
  return w0;
}

Eigen::VectorXd TaskSpaceMpc::compute(
  const riptide::RobotState & state,
  const riptide::EndEffectorTarget & target,
  double /*dt*/)
{
  model_->update(state);

  const Eigen::Isometry3d X = model_->framePose("ee");
  const Eigen::MatrixXd J = model_->jacobian("ee");
  const Eigen::MatrixXd & M = model_->massMatrix();
  const Eigen::Index n = J.cols();

  // Task error x_err = x_d - x (world); e = x - x_d = -x_err.
  Eigen::Matrix<double, 6, 1> x_err;
  x_err.head<3>() = target.pose.translation() - X.translation();
  const Eigen::Matrix3d R_err = target.pose.rotation() * X.rotation().transpose();
  const Eigen::AngleAxisd aa(R_err);
  x_err.tail<3>() = aa.angle() * aa.axis();

  const Eigen::Matrix<double, 6, 1> v_ee = J * state.dq;   // e_dot per axis

  // Per-axis receding-horizon solve -> commanded task acceleration w.
  Eigen::Matrix<double, 6, 1> w;
  for (int a = 0; a < 6; ++a)
  {
    w[a] = solve_axis(axis_[a], -x_err[a], v_ee[a]);
  }

  // Operational-space mapping (identical to TaskSpaceLqr): tau = J^T (Lambda w)
  // + nonlinear + dynamically-consistent nullspace posture.
  const Eigen::MatrixXd Minv = M.inverse();
  const Eigen::MatrixXd Lambda =
    (J * Minv * J.transpose() + jacobian_damping_ * Eigen::MatrixXd::Identity(6, 6)).inverse();

  Eigen::VectorXd tau = J.transpose() * (Lambda * w);

  const Eigen::MatrixXd Jbar_T = Lambda * J * Minv;                    // 6 x n
  const Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n, n) - J.transpose() * Jbar_T;
  if (q_rest_.size() == n)
  {
    const Eigen::VectorXd tau_posture = null_kp_ * (q_rest_ - state.q) - null_kd_ * state.dq;
    tau += N * tau_posture;
  }

  tau += model_->nonlinear();

  for (Eigen::Index i = 0; i < n; ++i)
  {
    const double lim = (i < max_effort_.size()) ? max_effort_[i] : 1e9;
    tau[i] = std::clamp(tau[i], -lim, lim);
  }
  return tau;
}

void TaskSpaceMpc::reset()
{
  for (auto & ax : axis_) { ax.warm.setZero(); }
}

}  // namespace riptide_control

PLUGINLIB_EXPORT_CLASS(riptide_control::TaskSpaceMpc, riptide::IControlLaw)
