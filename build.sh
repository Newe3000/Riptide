#!/usr/bin/env bash
# Build + source the Riptide workspace in one step.
#
# SOURCE it so the sourced environment stays in your current shell:
#
#   source build.sh                                   # build everything, then source
#   source build.sh --packages-select riptide_control # extra args pass to colcon
#   source build.sh --packages-select riptide_mujoco riptide_control
#
# Running it directly (./build.sh) still builds, but cannot set up your current
# shell — it will remind you to `source install/setup.bash` afterward.
#
# Overrides (optional): ROS_DISTRO (default jazzy). The MuJoCo/Pinocchio SDK
# paths default inside CMake (~/.mujoco/mujoco-3.10.0, ~/.local/pinocchio); set
# MUJOCO_ROOT / PINOCCHIO_ROOT before running if yours live elsewhere.

_riptide_distro="${ROS_DISTRO:-jazzy}"

# Workspace root = directory of this script (works whether sourced or executed).
if [ -n "${BASH_SOURCE[0]}" ]; then
  _riptide_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
  _riptide_ws="$(pwd)"
fi

# Detect sourced vs executed, so we can use return (persist env) vs exit.
if (return 0 2>/dev/null); then _riptide_sourced=1; else _riptide_sourced=0; fi

# 1. ROS 2 base.
if [ -f "/opt/ros/${_riptide_distro}/setup.bash" ]; then
  source "/opt/ros/${_riptide_distro}/setup.bash"
else
  echo "riptide: /opt/ros/${_riptide_distro}/setup.bash not found (set ROS_DISTRO?)" >&2
fi

# 2. Build (any extra args go straight to colcon, e.g. --packages-select ...).
echo "riptide: colcon build --symlink-install $*"
( cd "$_riptide_ws" && colcon build --symlink-install "$@" )
_riptide_rc=$?

# 3. Source the overlay — only if the build succeeded (a stale overlay is worse).
if [ "$_riptide_rc" -eq 0 ] && [ -f "$_riptide_ws/install/setup.bash" ]; then
  source "$_riptide_ws/install/setup.bash"
  echo "riptide: build OK — workspace sourced."
else
  echo "riptide: build failed (rc=$_riptide_rc) — overlay NOT sourced." >&2
fi

if [ "$_riptide_sourced" -eq 0 ]; then
  echo "riptide: TIP — run 'source build.sh' so the sourced env stays in this shell."
fi

# Return when sourced (keeps your terminal alive), exit when executed.
if [ "$_riptide_sourced" -eq 1 ]; then
  unset _riptide_ws _riptide_distro _riptide_sourced
  return "$_riptide_rc"
else
  unset _riptide_ws _riptide_distro _riptide_sourced
  exit "$_riptide_rc"
fi
