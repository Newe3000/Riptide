#include "riptide_control/ee_stabilization_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <Eigen/Geometry>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "riptide_dynamics/pinocchio_model.hpp"

namespace riptide_control
{
namespace
{
// Canonical floating-base sensor interfaces (must match riptide_mujoco).
const std::vector<std::string> kBaseIfaceNames = {
  "position.x", "position.y", "position.z",
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "linear_velocity.x", "linear_velocity.y", "linear_velocity.z",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z"};

bool is_finite(const riptide::RobotState & s)
{
  return s.q.allFinite() && s.dq.allFinite() &&
         s.base_pose.matrix().allFinite() && s.base_twist.allFinite();
}
}  // namespace

controller_interface::CallbackReturn EeStabilizationController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("ee_frame", "fer_hand_tcp");
  auto_declare<std::string>("arm_urdf_package", "riptide_description");
  auto_declare<std::string>("arm_urdf_relpath", "urdf/fer_arm.urdf");
  auto_declare<std::vector<std::string>>(
    "locked_joints", {"fer_finger_joint1", "fer_finger_joint2"});
  auto_declare<std::string>("base_sensor", "auv_base");
  auto_declare<std::string>("control_law", "riptide_control/TaskSpaceImpedance");
  auto_declare<std::vector<double>>("target_position", {0.5, 0.0, 0.6});
  auto_declare<std::vector<double>>("target_orientation", {0.0, 0.0, 0.0, 1.0});  // xyzw (identity)
  // Joint-limit avoidance (repulsive torque near the limits; on top of any law).
  auto_declare<bool>("joint_limit_avoidance", true);
  auto_declare<double>("limit_buffer", 0.2);    // rad from a limit before it engages
  auto_declare<double>("limit_gain", 40.0);     // Nm at the limit
  auto_declare<double>("limit_damping", 2.0);   // Nm.s/rad, damps motion into a limit
  auto_declare<std::vector<double>>("max_effort", {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0});
  // Mount offset: measured base link (auv_base_link) -> arm root (link0). The
  // arm sits on top of the 0.6 m hull, so its root is +0.3 m in z.
  auto_declare<std::vector<double>>("base_to_arm_offset", {0.0, 0.0, 0.3});
  // Hydrodynamic drag compensation. OFF by default: cancelling the arm's drag is
  // ANTI-DAMPING (it removes the beneficial damping the water provides), and with
  // an over-estimating model it destabilizes under fast motion (measured: EE RMS
  // 41 cm -> 145 cm at a 120 N current). The mismatch is favorable; the hydro
  // model is kept for study / future MPC prediction. Fluid params match the MJCF.
  auto_declare<bool>("hydro_compensation", false);
  auto_declare<double>("fluid_density", 1000.0);
  auto_declare<double>("fluid_viscosity", 0.0009);
  auto_declare<double>("drag_coefficient", 1.0);
  // Default: hold whatever EE pose the arm is in at activation (robust demo).
  auto_declare<bool>("capture_target_on_activate", true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EeStabilizationController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  base_sensor_ = node->get_parameter("base_sensor").as_string();
  base_iface_names_ = kBaseIfaceNames;
  if (joints_.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  const std::string ee_frame = node->get_parameter("ee_frame").as_string();
  const auto locked = node->get_parameter("locked_joints").as_string_array();
  std::string arm_urdf;
  try
  {
    arm_urdf = ament_index_cpp::get_package_share_directory(
                 node->get_parameter("arm_urdf_package").as_string()) +
               "/" + node->get_parameter("arm_urdf_relpath").as_string();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(node->get_logger(), "Could not resolve arm URDF path: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto off = node->get_parameter("base_to_arm_offset").as_double_array();
  const Eigen::Vector3d mount =
    (off.size() == 3) ? Eigen::Vector3d(off[0], off[1], off[2]) : Eigen::Vector3d::Zero();
  const bool hydro = node->get_parameter("hydro_compensation").as_bool();
  const double rho = node->get_parameter("fluid_density").as_double();
  const double mu = node->get_parameter("fluid_viscosity").as_double();
  const double cd = node->get_parameter("drag_coefficient").as_double();

  std::shared_ptr<riptide::PinocchioModel> pin_model;
  try
  {
    pin_model = std::make_shared<riptide::PinocchioModel>(
      arm_urdf, ee_frame, joints_, locked, mount, hydro, rho, mu, cd);
    model_ = pin_model;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to build Pinocchio model: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Joint-limit avoidance setup (limits come from the URDF via the model).
  jla_enabled_ = node->get_parameter("joint_limit_avoidance").as_bool();
  jla_buffer_ = node->get_parameter("limit_buffer").as_double();
  jla_gain_ = node->get_parameter("limit_gain").as_double();
  jla_damping_ = node->get_parameter("limit_damping").as_double();
  q_lower_ = pin_model->lowerLimits();
  q_upper_ = pin_model->upperLimits();
  const auto me = node->get_parameter("max_effort").as_double_array();
  max_effort_ = Eigen::Map<const Eigen::VectorXd>(me.data(), static_cast<Eigen::Index>(me.size()));

  // Target EE pose (world).
  const auto tp = node->get_parameter("target_position").as_double_array();
  const auto to = node->get_parameter("target_orientation").as_double_array();
  target_.pose = Eigen::Isometry3d::Identity();
  if (tp.size() == 3) { target_.pose.translation() = Eigen::Vector3d(tp[0], tp[1], tp[2]); }
  if (to.size() == 4)
  {
    Eigen::Quaterniond q(to[3], to[0], to[1], to[2]);  // (w, x, y, z)
    target_.pose.linear() = q.normalized().toRotationMatrix();
  }

  // Load the control-law plugin and configure it against this node.
  const std::string law = node->get_parameter("control_law").as_string();
  try
  {
    loader_ = std::make_unique<pluginlib::ClassLoader<riptide::IControlLaw>>(
      "riptide_control", "riptide::IControlLaw");
    control_law_ = loader_->createSharedInstance(law);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to load control law '%s': %s", law.c_str(), e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!control_law_->on_configure(
        node->get_node_parameters_interface(), node->get_node_logging_interface(), model_))
  {
    RCLCPP_ERROR(node->get_logger(), "Control law on_configure failed.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Strip the "riptide_control/" prefix for a compact label (e.g. "TaskSpaceLqr").
  const auto slash = law.find_last_of('/');
  control_law_name_ = (slash == std::string::npos) ? law : law.substr(slash + 1);
  debug_pub_ = node->create_publisher<riptide_msgs::msg::ControlDebug>(
    "/riptide/control_debug", rclcpp::SystemDefaultsQoS());

  // Live desired-pose command (teleop GUI etc.) + latched current-target echo.
  target_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/riptide/ee_target", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::PoseStamped & msg) { target_callback(msg); });
  target_current_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/riptide/ee_target/current", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(node->get_logger(),
    "EeStabilizationController configured: %zu joints, law '%s', hydro_compensation=%s.",
    joints_.size(), law.c_str(), hydro ? "on" : "off");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
EeStabilizationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_) { cfg.names.push_back(j + "/" + hardware_interface::HW_IF_EFFORT); }
  return cfg;
}

controller_interface::InterfaceConfiguration
EeStabilizationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_)
  {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & n : base_iface_names_) { cfg.names.push_back(base_sensor_ + "/" + n); }
  return cfg;
}

controller_interface::CallbackReturn EeStabilizationController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::size_t n = joints_.size();
  cmd_effort_idx_.assign(n, 0);
  state_pos_idx_.assign(n, 0);
  state_vel_idx_.assign(n, 0);
  base_idx_.assign(base_iface_names_.size(), 0);

  auto find = [](const auto & vec, const std::string & prefix, const std::string & iface,
                 std::size_t & out) -> bool {
    for (std::size_t k = 0; k < vec.size(); ++k)
    {
      if (vec[k].get_prefix_name() == prefix && vec[k].get_interface_name() == iface)
      {
        out = k;
        return true;
      }
    }
    return false;
  };

  for (std::size_t i = 0; i < n; ++i)
  {
    if (!find(state_interfaces_, joints_[i], hardware_interface::HW_IF_POSITION, state_pos_idx_[i]) ||
        !find(state_interfaces_, joints_[i], hardware_interface::HW_IF_VELOCITY, state_vel_idx_[i]) ||
        !find(command_interfaces_, joints_[i], hardware_interface::HW_IF_EFFORT, cmd_effort_idx_[i]))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing interface for joint '%s'.", joints_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  has_base_ = true;
  for (std::size_t k = 0; k < base_iface_names_.size(); ++k)
  {
    if (!find(state_interfaces_, base_sensor_, base_iface_names_[k], base_idx_[k]))
    {
      has_base_ = false;  // no floating-base sensor (e.g. mock) -> treat base as fixed
      break;
    }
  }
  if (control_law_) { control_law_->reset(); }

  // Defer capturing the EE hold target to the first update() with a finite
  // state. At activation the hardware's first read() may not have populated the
  // state interfaces yet (they read NaN), which would poison the target. Until
  // the capture happens, target_ holds the configured fallback from on_configure.
  capture_pending_ = get_node()->get_parameter("capture_target_on_activate").as_bool();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EeStabilizationController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    (void)command_interfaces_[cmd_effort_idx_[i]].set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

riptide::RobotState EeStabilizationController::read_state() const
{
  const std::size_t n = joints_.size();
  riptide::RobotState state;
  state.q.resize(n);
  state.dq.resize(n);
  // Treat a NaN interface (not yet written by the hardware's first read()) the
  // same as an absent one: fall back to 0 rather than propagating the NaN.
  auto finite_or_zero = [](double x) { return std::isfinite(x) ? x : 0.0; };
  for (std::size_t i = 0; i < n; ++i)
  {
    state.q[i] = finite_or_zero(state_interfaces_[state_pos_idx_[i]].get_optional().value_or(0.0));
    state.dq[i] = finite_or_zero(state_interfaces_[state_vel_idx_[i]].get_optional().value_or(0.0));
  }
  if (has_base_)
  {
    auto v = [&](std::size_t k) {
      return finite_or_zero(state_interfaces_[base_idx_[k]].get_optional().value_or(0.0));
    };
    state.base_pose = Eigen::Isometry3d::Identity();
    state.base_pose.translation() = Eigen::Vector3d(v(0), v(1), v(2));
    Eigen::Quaterniond q(v(6), v(3), v(4), v(5));  // (w, x, y, z)
    if (q.norm() > 1e-6) { state.base_pose.linear() = q.normalized().toRotationMatrix(); }
    state.base_twist << v(7), v(8), v(9), v(10), v(11), v(12);
  }
  return state;
}

Eigen::VectorXd EeStabilizationController::jointLimitAvoidance(
  const Eigen::VectorXd & q, const Eigen::VectorXd & dq) const
{
  const Eigen::Index n = q.size();
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(n);
  if (!jla_enabled_ || q_lower_.size() != n || q_upper_.size() != n || jla_buffer_ <= 0.0)
  {
    return tau;
  }
  for (Eigen::Index i = 0; i < n; ++i)
  {
    // Near the UPPER limit: push toward smaller q, growing quadratically inside
    // the buffer; damp velocity heading further into the limit.
    const double pen_u = q[i] - (q_upper_[i] - jla_buffer_);
    if (pen_u > 0.0)
    {
      const double r = pen_u / jla_buffer_;                 // 0 at buffer edge, 1 at limit
      tau[i] -= jla_gain_ * r * r;
      if (dq[i] > 0.0) { tau[i] -= jla_damping_ * dq[i] * std::min(r, 1.0); }
    }
    // Near the LOWER limit: push toward larger q.
    const double pen_l = (q_lower_[i] + jla_buffer_) - q[i];
    if (pen_l > 0.0)
    {
      const double r = pen_l / jla_buffer_;
      tau[i] += jla_gain_ * r * r;
      if (dq[i] < 0.0) { tau[i] -= jla_damping_ * dq[i] * std::min(r, 1.0); }
    }
  }
  return tau;
}

controller_interface::return_type EeStabilizationController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const riptide::RobotState state = read_state();

  auto hold_zero = [&]() {
    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
      (void)command_interfaces_[cmd_effort_idx_[i]].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  };

  // Wait for a finite measurement before doing anything: on the first cycles the
  // hardware may not have written the state interfaces yet.
  if (!is_finite(state)) { return hold_zero(); }

  // A live pose command (teleop GUI) overrides the held/captured target. Once
  // one arrives, the initial auto-capture is abandoned in favour of the command.
  if (have_target_cmd_.load())
  {
    std::lock_guard<std::mutex> lock(target_cmd_mutex_);
    target_.pose = target_cmd_pose_;
    capture_pending_ = false;
  }

  // Deferred target capture (see on_activate): grab the EE pose from the first
  // finite state. Only commit it if the resulting pose is itself finite.
  if (capture_pending_)
  {
    model_->update(state);
    const Eigen::Isometry3d ee = model_->framePose("ee");
    if (ee.matrix().allFinite())
    {
      target_.pose = ee;
      capture_pending_ = false;
      const Eigen::Vector3d p = target_.pose.translation();
      RCLCPP_INFO(get_node()->get_logger(),
        "Holding captured EE target at world [%.3f, %.3f, %.3f].", p.x(), p.y(), p.z());
    }
    else
    {
      return hold_zero();  // model not ready yet; try again next cycle
    }
  }

  // Echo the current target (latched) at ~10 Hz so a teleop GUI can seed itself
  // to the live pose and avoid snapping the EE when it takes control.
  if (cycle_ % 25 == 0) { publish_current_target(); }

  const auto t_start = std::chrono::steady_clock::now();
  Eigen::VectorXd tau = control_law_->compute(state, target_, period.seconds());
  const double solve_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();

  if (!tau.allFinite())
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Control law produced a non-finite torque; commanding zero this cycle.");
    return hold_zero();
  }

  // Joint-limit avoidance on top of any control law, then a final torque clamp.
  tau += jointLimitAvoidance(state.q, state.dq);
  for (Eigen::Index i = 0; i < tau.size() && i < max_effort_.size(); ++i)
  {
    tau[i] = std::clamp(tau[i], -max_effort_[i], max_effort_[i]);
  }

  for (std::size_t i = 0; i < joints_.size() && i < static_cast<std::size_t>(tau.size()); ++i)
  {
    (void)command_interfaces_[cmd_effort_idx_[i]].set_value(tau[i]);
  }

  // Publish ControlDebug at ~50 Hz (every 5th cycle at 250 Hz) for evaluation.
  if (debug_pub_ && (cycle_++ % 5 == 0))
  {
    const Eigen::Isometry3d X = model_->framePose("ee");   // model was updated in compute()
    Eigen::Matrix<double, 6, 1> err;
    err.head<3>() = target_.pose.translation() - X.translation();
    const Eigen::Matrix3d R_err = target_.pose.rotation() * X.rotation().transpose();
    const Eigen::AngleAxisd aa(R_err);
    err.tail<3>() = aa.angle() * aa.axis();

    riptide_msgs::msg::ControlDebug msg;
    msg.header.stamp = get_node()->now();
    msg.control_law = control_law_name_;
    msg.solve_time_ms = solve_ms;
    msg.joint_names = joints_;
    msg.tau.assign(tau.data(), tau.data() + tau.size());
    msg.q.assign(state.q.data(), state.q.data() + state.q.size());
    msg.dq.assign(state.dq.data(), state.dq.data() + state.dq.size());
    msg.ee_pose_error.assign(err.data(), err.data() + 6);
    debug_pub_->publish(msg);
  }
  return controller_interface::return_type::OK;
}

void EeStabilizationController::target_callback(const geometry_msgs::msg::PoseStamped & msg)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() =
    Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  Eigen::Quaterniond q(
    msg.pose.orientation.w, msg.pose.orientation.x,
    msg.pose.orientation.y, msg.pose.orientation.z);
  if (!pose.translation().allFinite() || q.norm() < 1e-6) { return; }  // ignore junk
  pose.linear() = q.normalized().toRotationMatrix();
  {
    std::lock_guard<std::mutex> lock(target_cmd_mutex_);
    target_cmd_pose_ = pose;
  }
  have_target_cmd_.store(true);
}

void EeStabilizationController::publish_current_target()
{
  if (!target_current_pub_) { return; }
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = get_node()->now();
  msg.header.frame_id = "world";
  const Eigen::Vector3d p = target_.pose.translation();
  const Eigen::Quaterniond q(target_.pose.rotation());
  msg.pose.position.x = p.x();
  msg.pose.position.y = p.y();
  msg.pose.position.z = p.z();
  msg.pose.orientation.w = q.w();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  target_current_pub_->publish(msg);
}

}  // namespace riptide_control

PLUGINLIB_EXPORT_CLASS(
  riptide_control::EeStabilizationController, controller_interface::ControllerInterface)
