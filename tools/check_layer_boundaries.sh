#!/usr/bin/env bash
# Layer-boundary fitness check for the Conan migration (docs/plan).
#
# Asserts that the sources destined to become ROS-agnostic Layer-1 Conan cores
# contain no ROS-framework tokens. Prints a deterministic, sorted leak list.
#
#   exit 0  -> every Layer-1 target is clean
#   exit 2  -> a target that must be clean leaks a forbidden token
#
# As of the Step 8 seam cutover, the control-law core has been extracted into the
# riptide_control_core Conan package and the in-tree copies under riptide_control
# are gone; the ROS layer now consumes the cores via Conan (see
# docs/adr/0002-colcon-conan-seam.md). All three L1 packages must be CLEAN.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

FORBIDDEN='rclcpp|rclpy|pluginlib|PLUGINLIB_EXPORT_CLASS|controller_interface|hardware_interface|rosidl|riptide_msgs'

# Targets that MUST already be clean: the extracted L1 Conan packages.
DYNAMICS=(riptide_dynamics/include riptide_dynamics/src)
GEOMETRY=(riptide_geometry/include)
CONTROL_CORE_PKG=(riptide_control_core/include riptide_control_core/src)

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
geom="$(leaks "${GEOMETRY[@]}")"
corepkg="$(leaks "${CONTROL_CORE_PKG[@]}")"

rc=0
report_clean() {  # $1=label  $2=leaks -> fail (rc=2) if any
  echo; echo "[L1: $1]  (must be CLEAN)"
  if [ -z "$2" ]; then echo "  CLEAN"; else echo "$2" | sed 's/^/  LEAK  /'; rc=2; fi
}
report_clean "riptide_dynamics" "$dyn"
report_clean "riptide_geometry" "$geom"
report_clean "riptide_control_core" "$corepkg"

echo
echo "RESULT: extracted L1 packages $([ -z "$dyn$geom$corepkg" ] && echo CLEAN || echo LEAKING)"
exit $rc
