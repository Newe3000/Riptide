#include "riptide_mujoco/mujoco_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace riptide_mujoco
{
namespace
{
constexpr char kLogger[] = "MujocoSystem";

// Default Franka "ready" pose (radians); order matches fer_joint1..7.
const std::vector<double> kFrankaHome = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};

// Canonical floating-base state interfaces (order == fill order in read()).
const std::vector<std::string> kBaseIfaceNames = {
  "position.x", "position.y", "position.z",
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "linear_velocity.x", "linear_velocity.y", "linear_velocity.z",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z"};

std::string param_or(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & fallback)
{
  auto it = info.hardware_parameters.find(key);
  return (it != info.hardware_parameters.end() && !it->second.empty()) ? it->second
                                                                       : fallback;
}
}  // namespace

hardware_interface::CallbackReturn MujocoSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::string mjcf_path = param_or(info_, "mjcf_model", "");
  if (mjcf_path.empty())
  {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger),
      "Missing required hardware parameter 'mjcf_model' (path to the MJCF).");
    return hardware_interface::CallbackReturn::ERROR;
  }

  char error[1024] = "";
  m_ = mj_loadXML(mjcf_path.c_str(), nullptr, error, sizeof(error));
  if (m_ == nullptr)
  {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger),
      "Failed to load MJCF '%s': %s", mjcf_path.c_str(), error);
    return hardware_interface::CallbackReturn::ERROR;
  }
  d_ = mj_makeData(m_);

  // --- actuated joints ----------------------------------------------------
  const std::size_t n = info_.joints.size();
  qpos_adr_.assign(n, -1);
  dof_adr_.assign(n, -1);
  act_id_.assign(n, -1);
  pos_.assign(n, 0.0);
  vel_.assign(n, 0.0);
  eff_.assign(n, 0.0);
  eff_cmd_.assign(n, 0.0);
  home_.assign(n, 0.0);

  for (std::size_t i = 0; i < n; ++i)
  {
    const std::string & jname = info_.joints[i].name;
    const int jid = mj_name2id(m_, mjOBJ_JOINT, jname.c_str());
    const int aid = mj_name2id(m_, mjOBJ_ACTUATOR, jname.c_str());
    if (jid < 0 || aid < 0)
    {
      RCLCPP_FATAL(rclcpp::get_logger(kLogger),
        "Joint/actuator '%s' from the URDF is not present in the MJCF.", jname.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    qpos_adr_[i] = m_->jnt_qposadr[jid];
    dof_adr_[i] = m_->jnt_dofadr[jid];
    act_id_[i] = aid;
    home_[i] = (i < kFrankaHome.size()) ? kFrankaHome[i] : 0.0;
  }

  // --- floating base (optional) -------------------------------------------
  base_frame_ = param_or(info_, "base_link_frame", "auv_base_link");
  world_frame_ = param_or(info_, "world_frame", "world");
  const std::string base_joint = param_or(info_, "floating_base_joint", "auv_freejoint");
  const int bjid = mj_name2id(m_, mjOBJ_JOINT, base_joint.c_str());
  if (bjid >= 0 && m_->jnt_type[bjid] == mjJNT_FREE)
  {
    has_base_ = true;
    base_qpos_adr_ = m_->jnt_qposadr[bjid];
    base_dof_adr_ = m_->jnt_dofadr[bjid];
    base_sensor_name_ = info_.sensors.empty() ? "auv_base" : info_.sensors.front().name;
    base_iface_names_ = kBaseIfaceNames;
    base_state_.assign(base_iface_names_.size(), 0.0);
    RCLCPP_INFO(rclcpp::get_logger(kLogger),
      "Floating base '%s' exposed as sensor '%s' (%zu state interfaces).",
      base_joint.c_str(), base_sensor_name_.c_str(), base_iface_names_.size());
  }

  // --- hull thrusters exposed via <gpio> (optional) -----------------------
  // Each gpio command interface names a MuJoCo force actuator (thr_*).
  for (const auto & gpio : info_.gpios)
  {
    for (const auto & ci : gpio.command_interfaces)
    {
      const int aid = mj_name2id(m_, mjOBJ_ACTUATOR, ci.name.c_str());
      if (aid < 0)
      {
        RCLCPP_FATAL(rclcpp::get_logger(kLogger),
          "Thruster actuator '%s' (gpio '%s') is not present in the MJCF.",
          ci.name.c_str(), gpio.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      thr_gpio_.push_back(gpio.name);
      thr_name_.push_back(ci.name);
      thr_act_id_.push_back(aid);
    }
  }
  thr_cmd_.assign(thr_act_id_.size(), 0.0);
  thr_force_.assign(thr_act_id_.size(), 0.0);

  RCLCPP_INFO(rclcpp::get_logger(kLogger),
    "Loaded MJCF '%s': %ld dof, %ld actuators; mapped %zu joints, %zu thrusters.",
    mjcf_path.c_str(),
    static_cast<long>(m_->nv), static_cast<long>(m_->nu), n, thr_act_id_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Internal node spun on its own thread: disturbance I/O + base viz.
  rclcpp::NodeOptions opts;
  node_ = std::make_shared<rclcpp::Node>("riptide_mujoco_bridge", opts);

  dist_sub_ = node_->create_subscription<riptide_msgs::msg::DisturbanceCommand>(
    "/riptide/disturbance", rclcpp::QoS(10),
    [this](const riptide_msgs::msg::DisturbanceCommand & msg) { disturbance_callback(msg); });
  dist_gt_pub_ = node_->create_publisher<riptide_msgs::msg::DisturbanceCommand>(
    "/riptide/disturbance/ground_truth", rclcpp::QoS(10));
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
    "/riptide/odom", rclcpp::QoS(10));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spinning_ = true;
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "MuJoCo bridge node started.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    const std::string & jn = info_.joints[i].name;
    ifaces.emplace_back(jn, hardware_interface::HW_IF_POSITION, &pos_[i]);
    ifaces.emplace_back(jn, hardware_interface::HW_IF_VELOCITY, &vel_[i]);
    ifaces.emplace_back(jn, hardware_interface::HW_IF_EFFORT, &eff_[i]);
  }
  if (has_base_)
  {
    for (std::size_t k = 0; k < base_iface_names_.size(); ++k)
    {
      ifaces.emplace_back(base_sensor_name_, base_iface_names_[k], &base_state_[k]);
    }
  }
  for (std::size_t k = 0; k < thr_act_id_.size(); ++k)
  {
    ifaces.emplace_back(thr_gpio_[k], thr_name_[k], &thr_force_[k]);
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface> MujocoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    ifaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &eff_cmd_[i]);
  }
  for (std::size_t k = 0; k < thr_act_id_.size(); ++k)
  {
    ifaces.emplace_back(thr_gpio_[k], thr_name_[k], &thr_cmd_[k]);
  }
  return ifaces;
}

hardware_interface::CallbackReturn MujocoSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  mj_resetData(m_, d_);   // restores the spawn pose (incl. base at its MJCF pos)
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    d_->qpos[qpos_adr_[i]] = home_[i];
    eff_cmd_[i] = 0.0;
  }
  std::fill(thr_cmd_.begin(), thr_cmd_.end(), 0.0);
  mj_forward(m_, d_);
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "MuJoCo system activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (spinning_.exchange(false))
  {
    if (executor_) { executor_->cancel(); }
    if (spin_thread_.joinable()) { spin_thread_.join(); }
  }
  if (d_) { mj_deleteData(d_); d_ = nullptr; }
  if (m_) { mj_deleteModel(m_); m_ = nullptr; }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    pos_[i] = d_->qpos[qpos_adr_[i]];
    vel_[i] = d_->qvel[dof_adr_[i]];
    eff_[i] = d_->actuator_force[act_id_[i]];
  }
  for (std::size_t k = 0; k < thr_act_id_.size(); ++k)
  {
    thr_force_[k] = d_->actuator_force[thr_act_id_[k]];
  }

  if (has_base_)
  {
    const int q = base_qpos_adr_;
    const int v = base_dof_adr_;
    // MuJoCo free joint: qpos = [x y z, qw qx qy qz]; qvel = [v(world), w(local)].
    base_state_[0] = d_->qpos[q + 0];
    base_state_[1] = d_->qpos[q + 1];
    base_state_[2] = d_->qpos[q + 2];
    base_state_[3] = d_->qpos[q + 4];  // qx
    base_state_[4] = d_->qpos[q + 5];  // qy
    base_state_[5] = d_->qpos[q + 6];  // qz
    base_state_[6] = d_->qpos[q + 3];  // qw
    for (int k = 0; k < 6; ++k) { base_state_[7 + k] = d_->qvel[v + k]; }

    // Publish base viz at ~50 Hz (assuming a 500 Hz control loop). Stamp with
    // the node clock so it matches robot_state_publisher's TF timestamps.
    (void)time;
    if (node_ && (cycle_ % 10 == 0)) { publish_base_state(node_->now()); }
  }
  ++cycle_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Apply disturbance wrenches (world frame) to the target bodies.
  {
    std::lock_guard<std::mutex> lock(dist_mutex_);
    for (const auto & [body, w] : disturbances_)
    {
      const int bid = mj_name2id(m_, mjOBJ_BODY, body.c_str());
      if (bid < 0) { continue; }
      for (int k = 0; k < 6; ++k) { d_->xfrc_applied[6 * bid + k] = w[k]; }
    }
  }

  // Joint torque commands.
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    const double cmd = eff_cmd_[i];
    d_->ctrl[act_id_[i]] = std::isnan(cmd) ? 0.0 : cmd;
  }

  // Hull thruster force commands.
  for (std::size_t k = 0; k < thr_act_id_.size(); ++k)
  {
    const double cmd = thr_cmd_[k];
    d_->ctrl[thr_act_id_[k]] = std::isnan(cmd) ? 0.0 : cmd;
  }

  // Advance sim by the control period (>=1 MuJoCo step). Clamp the step count
  // so a scheduling gap / large first period cannot fast-forward the physics.
  const double dt = period.seconds();
  int steps = 1;
  if (dt > 0.0 && m_->opt.timestep > 0.0)
  {
    steps = std::clamp(static_cast<int>(std::lround(dt / m_->opt.timestep)), 1, 8);
  }
  for (int s = 0; s < steps; ++s) { mj_step(m_, d_); }
  return hardware_interface::return_type::OK;
}

void MujocoSystem::disturbance_callback(const riptide_msgs::msg::DisturbanceCommand & msg)
{
  const std::string body = msg.body.empty() ? base_frame_ : msg.body;
  // Map the URDF/base link name to the MuJoCo body if needed.
  const std::string mj_body = (body == base_frame_) ? "auv_base" : body;
  std::array<double, 6> w{
    msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z};
  {
    std::lock_guard<std::mutex> lock(dist_mutex_);
    disturbances_[mj_body] = w;
  }
  // Echo what is applied as ground truth.
  if (dist_gt_pub_) { dist_gt_pub_->publish(msg); }
}

void MujocoSystem::publish_base_state(const rclcpp::Time & time)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = time;
  tf.header.frame_id = world_frame_;
  tf.child_frame_id = base_frame_;
  tf.transform.translation.x = base_state_[0];
  tf.transform.translation.y = base_state_[1];
  tf.transform.translation.z = base_state_[2];
  tf.transform.rotation.x = base_state_[3];
  tf.transform.rotation.y = base_state_[4];
  tf.transform.rotation.z = base_state_[5];
  tf.transform.rotation.w = base_state_[6];
  tf_broadcaster_->sendTransform(tf);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = world_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = base_state_[0];
  odom.pose.pose.position.y = base_state_[1];
  odom.pose.pose.position.z = base_state_[2];
  odom.pose.pose.orientation.x = base_state_[3];
  odom.pose.pose.orientation.y = base_state_[4];
  odom.pose.pose.orientation.z = base_state_[5];
  odom.pose.pose.orientation.w = base_state_[6];
  odom.twist.twist.linear.x = base_state_[7];
  odom.twist.twist.linear.y = base_state_[8];
  odom.twist.twist.linear.z = base_state_[9];
  odom.twist.twist.angular.x = base_state_[10];
  odom.twist.twist.angular.y = base_state_[11];
  odom.twist.twist.angular.z = base_state_[12];
  odom_pub_->publish(odom);
}

MujocoSystem::~MujocoSystem()
{
  if (spinning_.exchange(false))
  {
    if (executor_) { executor_->cancel(); }
    if (spin_thread_.joinable()) { spin_thread_.join(); }
  }
  if (d_) { mj_deleteData(d_); d_ = nullptr; }
  if (m_) { mj_deleteModel(m_); m_ = nullptr; }
}

}  // namespace riptide_mujoco

PLUGINLIB_EXPORT_CLASS(riptide_mujoco::MujocoSystem, hardware_interface::SystemInterface)
