#!/usr/bin/env bash
# Layer-boundary fitness check for the Conan migration (docs/plan).
#
# Asserts that the sources destined to become ROS-agnostic Layer-1 Conan cores
# contain no ROS-framework tokens. Prints a deterministic, sorted leak list.
#
#   exit 0  -> every Layer-1 target is clean
#   exit 1  -> at least one target leaks a forbidden token
#   exit 2  -> a target that must ALREADY be clean (riptide_dynamics) leaks
#
# Today the control-law core still lives inside riptide_control and legitimately
# leaks rclcpp/pluginlib; that is the documented baseline (see
# docs/adr/0001-conan-package-taxonomy.md) and is expected to clear after the
# Step 5 de-ROS refactor and the Step 7 extraction.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

FORBIDDEN='rclcpp|rclpy|pluginlib|PLUGINLIB_EXPORT_CLASS|controller_interface|hardware_interface|rosidl|riptide_msgs'

# Target that MUST already be clean.
DYNAMICS=(riptide_dynamics/include riptide_dynamics/src)

# Target that becomes riptide_control_core + riptide_geometry (Step 7); still
# inside riptide_control today.
CONTROL_CORE=(
  riptide_control/include/riptide_control/control_law_interface.hpp
  riptide_control/include/riptide_control/operational_space.hpp
  riptide_control/include/riptide_control/task_space_impedance.hpp
  riptide_control/include/riptide_control/task_space_lqr.hpp
  riptide_control/include/riptide_control/task_space_mpc.hpp
  riptide_control/include/riptide_control/template_control_law.hpp
  riptide_control/src/task_space_impedance.cpp
  riptide_control/src/task_space_lqr.cpp
  riptide_control/src/task_space_mpc.cpp
  riptide_control/src/template_control_law.cpp
)

# Emit sorted "path: tok1, tok2" lines for the forbidden tokens found in $@.
leaks() {
  grep -rEno "$FORBIDDEN" "$@" 2>/dev/null \
    | awk -F: '{ t[$1]=t[$1]" "$NF } END { for (f in t) {
        n=split(t[f],a," "); delete seen; out="";
        for (i=1;i<=n;i++) if(!seen[a[i]]++){ out=(out==""?a[i]:out", "a[i]) }
        print f": "out } }' \
    | sort
}

echo "=== Layer-1 boundary audit ==="
dyn="$(leaks "${DYNAMICS[@]}")"
core="$(leaks "${CONTROL_CORE[@]}")"

rc=0
echo
echo "[L1: riptide_dynamics]  (must be CLEAN)"
if [ -z "$dyn" ]; then echo "  CLEAN"; else echo "$dyn" | sed 's/^/  LEAK  /'; rc=2; fi

echo
echo "[L1: control-law core -> riptide_control_core/geometry]  (leaks expected until Step 5/7)"
if [ -z "$core" ]; then echo "  CLEAN"; else echo "$core" | sed 's/^/  leak  /'; [ $rc -eq 0 ] && rc=1; fi

echo
nfiles=$(printf '%s\n' "$core" | grep -c . )
echo "RESULT: dynamics=$([ -z "$dyn" ] && echo clean || echo LEAKING); control-core leaking files=$nfiles"
exit $rc
