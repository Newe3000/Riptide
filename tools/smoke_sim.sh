#!/usr/bin/env bash
# Headless sim smoke test: the colcon<->Conan integration guard (CI Job B, and a
# handy local check). Launches the MuJoCo-backed EE-stabilization demo under a
# steady-current disturbance with no viewer/GUI/RViz, then asserts that:
#   1. the MuJoCo hardware plugin (Conan libmujoco) loads + activates,
#   2. the ee_stabilization_controller activates (instantiates the Conan Pinocchio
#      model + loads the riptide_control_core TaskSpaceImpedance plugin),
#   3. /riptide/control_debug publishes a finite, bounded EE pose error.
# Exits 0 on success, 1 on failure. Does a from-source-free run against whatever is
# already built in install/ (run `source build.sh` first, or in CI after colcon build).
#
#   ./tools/smoke_sim.sh                 # uses the current workspace
#   RIPTIDE_SMOKE_TIMEOUT=60 ./tools/smoke_sim.sh
# NOTE: no `set -u` -- ROS 2 setup.bash / Conan env scripts reference unset vars and
# would abort sourcing under nounset.
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTRO="${ROS_DISTRO:-jazzy}"
TIMEOUT="${RIPTIDE_SMOKE_TIMEOUT:-45}"
LOG="$(mktemp -t riptide_smoke.XXXXXX.log)"

cleanup() {
  pkill -9 -f 'lib/controller_manager/ros2_control_node' 2>/dev/null || true
  pkill -9 -f 'sim.launch.py'        2>/dev/null || true
  pkill -9 -f 'disturbance_generator' 2>/dev/null || true
  pkill -9 -f 'robot_state_publisher' 2>/dev/null || true
}
trap cleanup EXIT

# --- environment: ROS base + workspace overlay + Conan runtime libs ---
source "/opt/ros/${DISTRO}/setup.bash"      2>/dev/null || { echo "smoke: no ROS ${DISTRO}"; exit 1; }
source "${WS}/install/setup.bash"           2>/dev/null || { echo "smoke: workspace not built"; exit 1; }
source "${WS}/conan/ros_deps/conanrun.sh"   2>/dev/null || { echo "smoke: conan runtime env missing (run build.sh)"; exit 1; }

echo "smoke: launching headless MuJoCo + EE-impedance + steady_current (timeout ${TIMEOUT}s)"
ros2 launch riptide_bringup sim.launch.py \
  use_mock_hardware:=false controller:=ee control_law:=impedance base_control:=true \
  water:=true disturbance:=steady_current \
  mujoco_viewer:=false ee_gui:=false rviz:=false > "$LOG" 2>&1 &

# --- wait for activation (or a crash) within the timeout ---
activated=0
for _ in $(seq 1 "$TIMEOUT"); do
  if grep -qiE 'terminate called|what\(\):|Segmentation|core dumped|undefined symbol' "$LOG"; then
    echo "smoke: FAIL — crash signature in sim:"; grep -iE 'terminate|what\(\):|Segmentation|undefined symbol' "$LOG" | head; exit 1
  fi
  if grep -q 'Configured and activated ee_stabilization_controller' "$LOG" \
     && grep -q "Successful 'activate' of hardware 'RiptideSystem'" "$LOG"; then
    activated=1; break
  fi
  sleep 1
done
if [ "$activated" -ne 1 ]; then
  echo "smoke: FAIL — controllers/hardware did not activate within ${TIMEOUT}s"; tail -30 "$LOG"; exit 1
fi
echo "smoke: hardware + ee_stabilization_controller activated"

# --- assert ControlDebug publishes a finite, bounded EE pose error ---
err="$(timeout 10 ros2 topic echo /riptide/control_debug --field ee_pose_error --once 2>/dev/null \
        | grep -oE "array\('d', \[[^]]*\]")"
if [ -z "$err" ]; then
  echo "smoke: FAIL — /riptide/control_debug did not publish"; exit 1
fi
# Parse the 6 doubles; require all finite and position-error norm < 1.0 m (held, not diverging).
norm="$(printf '%s' "$err" | grep -oE '[-0-9.eE]+' | head -3 | awk '{s+=$1*$1} END{printf "%.6f", sqrt(s)}')"
echo "smoke: control_debug EE position-error norm = ${norm} m"
ok="$(awk -v n="$norm" 'BEGIN{print (n==n && n+0<1.0) ? "1" : "0"}')"   # n==n rejects nan
if [ "$ok" != "1" ]; then
  echo "smoke: FAIL — EE error not finite/bounded (${norm})"; exit 1
fi

echo "smoke: PASS — Conan MuJoCo + Pinocchio mixed process stable, EE stabilization bounded"
exit 0
