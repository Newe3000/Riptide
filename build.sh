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
# The L1 cores (riptide_dynamics / riptide_geometry / riptide_control_core) and the
# third-party libs (Pinocchio / Eigen / MuJoCo) come from Conan: this script
# (re)generates their CMakeDeps config files into conan/ros_deps and APPENDS that
# dir to CMAKE_PREFIX_PATH, so the colcon/ament packages discover them without a
# global conan_toolchain clobbering the ament prefix path. See
# docs/adr/0002-colcon-conan-seam.md.
#
# Overrides (optional): ROS_DISTRO (default jazzy).

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

# 2. Refresh the Conan dependency configs (best-effort; skipped if conan absent).
_riptide_rosdeps="$_riptide_ws/conan/ros_deps"
if command -v conan >/dev/null 2>&1; then
  echo "riptide: conan install -> conan/ros_deps (CMakeDeps only)"
  # List riptide_dynamics + riptide_geometry as DIRECT requires too: riptide_control
  # links them directly (PinocchioModel / thruster allocation). As purely transitive
  # deps of the shared-library control_core (whose .so does not itself NEED dynamics),
  # CMakeDeps would emit their LIBS as empty and they would never reach the link line.
  ( cd "$_riptide_ws" && conan install \
      --requires=riptide_control_core/0.0.1 \
      --requires=riptide_dynamics/0.0.1 --requires=riptide_geometry/0.0.1 \
      --requires=mujoco/3.10.0 \
      -g CMakeDeps -o "pinocchio/*:with_collision_support=False" \
      -pr:h conan/profiles/riptide-linux-release \
      -pr:b conan/profiles/riptide-linux-build \
      --build=missing --output-folder=conan/ros_deps >/dev/null ) \
    || echo "riptide: conan install failed — using existing conan/ros_deps if present" >&2
  # urdfdom must come from ONE provider shared with the ROS stack: the system/ROS
  # urdfdom (ABI soname .so.4.0, same as Conan's 4.0.0) provides the component
  # targets urdf::urdf needs AND backs conan pinocchio's urdf parser. Drop Conan's
  # urdfdom-config so find_package(urdfdom) always resolves to the system one;
  # pinocchio pulls no tinyxml2/console_bridge directly, so none leak. See ADR 0002.
  rm -f "$_riptide_ws"/conan/ros_deps/urdfdom-*.cmake 2>/dev/null
fi
if [ -d "$_riptide_rosdeps" ]; then
  # APPEND (ament's own prefixes still win for ROS packages); never a toolchain.
  export CMAKE_PREFIX_PATH="$_riptide_rosdeps:${CMAKE_PREFIX_PATH}"
else
  echo "riptide: conan/ros_deps not found — L3 packages will fail to find the cores." >&2
fi

# 3. Build. CMAKE_INSTALL_RPATH_USE_LINK_PATH bakes the Conan cache lib dirs into
#    the installed .so RUNPATH so libpinocchio/libmujoco resolve at runtime.
echo "riptide: colcon build --symlink-install $*"
( cd "$_riptide_ws" && colcon build --symlink-install "$@" \
    --cmake-args -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON )
_riptide_rc=$?

# 4. Source the overlay — only if the build succeeded (a stale overlay is worse).
if [ "$_riptide_rc" -eq 0 ] && [ -f "$_riptide_ws/install/setup.bash" ]; then
  source "$_riptide_ws/install/setup.bash"
  # Conan builds relocatable libs (no baked rpath), so the Conan cache lib dirs
  # (Pinocchio, MuJoCo, the first-party cores) must be on LD_LIBRARY_PATH at run
  # time. The VirtualRunEnv generated alongside the CMakeDeps configs does exactly
  # that; sourcing it here means a subsequent `ros2 launch` in this shell finds the
  # Conan libs. (Boost is static-linked into Pinocchio, so no Boost .so leaks in.)
  if [ -f "$_riptide_rosdeps/conanrun.sh" ]; then
    source "$_riptide_rosdeps/conanrun.sh"
    echo "riptide: build OK — workspace + Conan runtime env sourced."
  else
    echo "riptide: build OK — workspace sourced (Conan runtime env missing!)." >&2
  fi
else
  echo "riptide: build failed (rc=$_riptide_rc) — overlay NOT sourced." >&2
fi

if [ "$_riptide_sourced" -eq 0 ]; then
  echo "riptide: TIP — run 'source build.sh' so the sourced env stays in this shell."
fi

# Return when sourced (keeps your terminal alive), exit when executed.
if [ "$_riptide_sourced" -eq 1 ]; then
  unset _riptide_ws _riptide_distro _riptide_sourced _riptide_rosdeps
  return "$_riptide_rc"
else
  unset _riptide_ws _riptide_distro _riptide_sourced _riptide_rosdeps
  exit "$_riptide_rc"
fi
