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

// Default Franka "ready" pose (radians), used if the joint has no defined home.
// Order matches fer_joint1..7; extra joints fall back to 0.
const std::vector<double> kFrankaHome = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
}  // namespace

hardware_interface::CallbackReturn MujocoSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto it = info_.hardware_parameters.find("mjcf_model");
  if (it == info_.hardware_parameters.end() || it->second.empty())
  {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger),
      "Missing required hardware parameter 'mjcf_model' (path to the MJCF).");
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::string mjcf_path = it->second;

  char error[1024] = "";
  m_ = mj_loadXML(mjcf_path.c_str(), nullptr, error, sizeof(error));
  if (m_ == nullptr)
  {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger),
      "Failed to load MJCF '%s': %s", mjcf_path.c_str(), error);
    return hardware_interface::CallbackReturn::ERROR;
  }
  d_ = mj_makeData(m_);

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
    if (jid < 0)
    {
      RCLCPP_FATAL(rclcpp::get_logger(kLogger),
        "Joint '%s' from the URDF is not present in the MJCF.", jname.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    qpos_adr_[i] = m_->jnt_qposadr[jid];
    dof_adr_[i] = m_->jnt_dofadr[jid];

    // Actuator carries the same name as the joint it drives (see the MJCF).
    const int aid = mj_name2id(m_, mjOBJ_ACTUATOR, jname.c_str());
    if (aid < 0)
    {
      RCLCPP_FATAL(rclcpp::get_logger(kLogger),
        "No actuator named '%s' in the MJCF for the effort command interface.",
        jname.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    act_id_[i] = aid;

    home_[i] = (i < kFrankaHome.size()) ? kFrankaHome[i] : 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger(kLogger),
    "Loaded MJCF '%s': %ld dof, %ld actuators; mapped %zu ros2_control joints.",
    mjcf_path.c_str(),
    static_cast<long>(m_->nv), static_cast<long>(m_->nu), n);

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
  return ifaces;
}

hardware_interface::CallbackReturn MujocoSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  mj_resetData(m_, d_);

  // Start from a non-singular home pose.
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    d_->qpos[qpos_adr_[i]] = home_[i];
    eff_cmd_[i] = 0.0;
  }
  mj_forward(m_, d_);

  // Seed the state interfaces so the first control cycle reads valid values.
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
  if (d_) { mj_deleteData(d_); d_ = nullptr; }
  if (m_) { mj_deleteModel(m_); m_ = nullptr; }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    pos_[i] = d_->qpos[qpos_adr_[i]];
    vel_[i] = d_->qvel[dof_adr_[i]];
    eff_[i] = d_->actuator_force[act_id_[i]];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    const double cmd = eff_cmd_[i];
    // Unclaimed command interfaces read as NaN; treat those as zero torque.
    d_->ctrl[act_id_[i]] = std::isnan(cmd) ? 0.0 : cmd;
  }

  // Advance sim by the control period (>=1 MuJoCo step), keeping sim ~ wall time.
  const double dt = period.seconds();
  int steps = 1;
  if (dt > 0.0 && m_->opt.timestep > 0.0)
  {
    steps = std::max(1, static_cast<int>(std::lround(dt / m_->opt.timestep)));
  }
  for (int s = 0; s < steps; ++s)
  {
    mj_step(m_, d_);
  }
  return hardware_interface::return_type::OK;
}

MujocoSystem::~MujocoSystem()
{
  if (d_) { mj_deleteData(d_); d_ = nullptr; }
  if (m_) { mj_deleteModel(m_); m_ = nullptr; }
}

}  // namespace riptide_mujoco

PLUGINLIB_EXPORT_CLASS(riptide_mujoco::MujocoSystem, hardware_interface::SystemInterface)
