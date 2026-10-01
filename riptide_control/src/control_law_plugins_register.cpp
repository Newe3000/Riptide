// Registers the IControlLaw implementations as pluginlib plugins for this ROS
// package. The law classes themselves live in the ROS-agnostic riptide_control_core
// Conan package (their .cpp carry NO pluginlib dependency); this single L3 TU is
// the one place that binds them to pluginlib, so the core stays framework-neutral.
// The declarations are matched by control_law_plugins.xml (library path
// "riptide_control"), and the symbols resolve through libriptide_control_core.so.
#include <pluginlib/class_list_macros.hpp>

#include "riptide_control/control_law_interface.hpp"
#include "riptide_control/task_space_impedance.hpp"
#include "riptide_control/task_space_lqr.hpp"
#include "riptide_control/task_space_mpc.hpp"
#include "riptide_control/template_control_law.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_control::TaskSpaceImpedance, riptide::IControlLaw)
PLUGINLIB_EXPORT_CLASS(riptide_control::TaskSpaceLqr, riptide::IControlLaw)
PLUGINLIB_EXPORT_CLASS(riptide_control::TaskSpaceMpc, riptide::IControlLaw)
PLUGINLIB_EXPORT_CLASS(riptide_control::TemplateControlLaw, riptide::IControlLaw)
