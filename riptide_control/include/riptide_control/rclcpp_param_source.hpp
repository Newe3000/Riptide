#pragma once

#include <string>
#include <utility>
#include <vector>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "riptide_control/param_source.hpp"

namespace riptide_control
{

/// ROS adapter: implements the framework-neutral riptide::ParamSource on top of a
/// ros2 NodeParametersInterface. declare() forwards to declare_parameter the first
/// time (so a yaml/CLI override wins) and then reads via get_parameter, preserving
/// ros2's parameter declaration + override semantics for the control laws.
class RclcppParamSource : public riptide::ParamSource
{
public:
  explicit RclcppParamSource(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params)
  : params_(std::move(params)) {}

  double declare(const std::string & name, double def) override { return get(name, def); }
  int declare(const std::string & name, int def) override { return get(name, def); }
  std::vector<double> declare(
    const std::string & name, const std::vector<double> & def) override
  {
    return get(name, def);
  }

private:
  template <typename T>
  T get(const std::string & name, const T & def)
  {
    if (!params_->has_parameter(name))
    {
      params_->declare_parameter(name, rclcpp::ParameterValue(def));
    }
    return params_->get_parameter(name).get_value<T>();
  }

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params_;
};

}  // namespace riptide_control
