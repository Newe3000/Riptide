#include "riptide_control/ee_stabilization_controller.hpp"

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
  auto_declare<std::vector<double>>("target_orientation", {1.0, 0.0, 0.0, 0.0});  // xyzw
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

  try
  {
    model_ = std::make_shared<riptide::PinocchioModel>(arm_urdf, ee_frame, joints_, locked);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to build Pinocchio model: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

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

  RCLCPP_INFO(node->get_logger(),
    "EeStabilizationController configured: %zu joints, law '%s'.", joints_.size(), law.c_str());
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

  // Capture the current EE world pose as the hold target, if requested.
  if (get_node()->get_parameter("capture_target_on_activate").as_bool())
  {
    const riptide::RobotState s = read_state();
    model_->update(s);
    target_.pose = model_->framePose("ee");
    const Eigen::Vector3d p = target_.pose.translation();
    RCLCPP_INFO(get_node()->get_logger(),
      "Holding captured EE target at world [%.3f, %.3f, %.3f].", p.x(), p.y(), p.z());
  }
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
  for (std::size_t i = 0; i < n; ++i)
  {
    state.q[i] = state_interfaces_[state_pos_idx_[i]].get_optional().value_or(0.0);
    state.dq[i] = state_interfaces_[state_vel_idx_[i]].get_optional().value_or(0.0);
  }
  if (has_base_)
  {
    auto v = [&](std::size_t k) { return state_interfaces_[base_idx_[k]].get_optional().value_or(0.0); };
    state.base_pose = Eigen::Isometry3d::Identity();
    state.base_pose.translation() = Eigen::Vector3d(v(0), v(1), v(2));
    Eigen::Quaterniond q(v(6), v(3), v(4), v(5));  // (w, x, y, z)
    if (q.norm() > 1e-6) { state.base_pose.linear() = q.normalized().toRotationMatrix(); }
    state.base_twist << v(7), v(8), v(9), v(10), v(11), v(12);
  }
  return state;
}

controller_interface::return_type EeStabilizationController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const riptide::RobotState state = read_state();
  const Eigen::VectorXd tau = control_law_->compute(state, target_, period.seconds());
  for (std::size_t i = 0; i < joints_.size() && i < static_cast<std::size_t>(tau.size()); ++i)
  {
    (void)command_interfaces_[cmd_effort_idx_[i]].set_value(tau[i]);
  }
  return controller_interface::return_type::OK;
}

}  // namespace riptide_control

PLUGINLIB_EXPORT_CLASS(
  riptide_control::EeStabilizationController, controller_interface::ControllerInterface)
