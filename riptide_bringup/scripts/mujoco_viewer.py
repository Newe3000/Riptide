#!/usr/bin/env python3
"""Passive MuJoCo viewer that mirrors the live Riptide simulation.

Opens MuJoCo's native viewer window (in parallel with RViz) and drives it purely
from the published state -- /joint_states (the 7 arm joints) and /riptide/odom
(the floating base pose) -- so you watch the actual simulation without touching
the control loop. It is READ-ONLY: it loads its own copy of the same MJCF, sets
qpos from the topics, and calls mj_forward + viewer.sync(); it never steps physics
and cannot affect the ros2_control node's sim.

Prerequisite: the `mujoco` Python package must be importable by the ROS Python
(the system python3 that runs rclpy):

    pip install --user mujoco==3.10.0     # match the MuJoCo SDK version
"""
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

try:
    import glfw
    import mujoco
    import mujoco.viewer
except ImportError as exc:  # pragma: no cover - environment dependent
    sys.stderr.write(
        "[mujoco_viewer] the 'mujoco' Python package is required in the ROS "
        "Python environment. Install it with:\n"
        "    pip install --user mujoco==3.10.0\n"
        f"(import error: {exc})\n")
    sys.exit(1)


def _prefer_x11_backend():
    """Prefer X11 (XWayland) over native Wayland for the GLFW viewer — but only
    when this GLFW build actually supports X11.

    MuJoCo's GLFW viewer is more reliable on X11; however some GLFW builds (e.g.
    the pip `glfw` wheel) are Wayland-only, and forcing X11 there makes
    glfw.init() fail outright ('requested platform is not supported'). So we force
    X11 only if it is both supported and there is a DISPLAY, and otherwise leave
    GLFW's default (Wayland). Must run before GLFW is initialized (launch_passive
    does that). Override with RIPTIDE_VIEWER_GLFW_WAYLAND=1 to always keep Wayland.
    """
    if os.environ.get("RIPTIDE_VIEWER_GLFW_WAYLAND") == "1":
        return
    try:
        if os.environ.get("DISPLAY") and glfw.platform_supported(glfw.PLATFORM_X11):
            glfw.init_hint(glfw.PLATFORM, glfw.PLATFORM_X11)
    except Exception:
        pass  # leave GLFW's default platform


class MujocoViewer(Node):
    def __init__(self):
        super().__init__("mujoco_viewer")
        default_mjcf = os.path.join(
            get_package_share_directory("riptide_description"), "mujoco", "riptide.xml")
        self.declare_parameter("mjcf", default_mjcf)
        self.declare_parameter("base_joint", "auv_freejoint")

        mjcf = self.get_parameter("mjcf").value
        self.model = mujoco.MjModel.from_xml_path(mjcf)
        self.data = mujoco.MjData(self.model)

        # qpos start index of the base free joint (7 values: x y z qw qx qy qz).
        bname = self.get_parameter("base_joint").value
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, bname)
        self.base_qadr = int(self.model.jnt_qposadr[bid]) if bid >= 0 else -1

        # joint-name -> qpos address, so /joint_states maps by name (not index).
        self._qadr = {}
        for j in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
            if name is not None:
                self._qadr[name] = int(self.model.jnt_qposadr[j])

        self._got_joints = False
        self.create_subscription(JointState, "/joint_states", self._on_joints, 50)
        self.create_subscription(Odometry, "/riptide/odom", self._on_odom, 50)
        self.get_logger().info(f"mujoco_viewer mirroring {mjcf}")

    def _on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            adr = self._qadr.get(name)
            if adr is not None:
                self.data.qpos[adr] = pos
        if not self._got_joints:
            self._got_joints = True
            self.get_logger().info("receiving /joint_states — viewer is tracking the sim.")

    def _on_odom(self, msg):
        if self.base_qadr < 0:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        a = self.base_qadr
        self.data.qpos[a:a + 3] = [p.x, p.y, p.z]
        self.data.qpos[a + 3:a + 7] = [q.w, q.x, q.y, q.z]  # MuJoCo order (w,x,y,z)


def main():
    rclpy.init()
    node = MujocoViewer()
    period = 1.0 / 60.0
    _prefer_x11_backend()

    # Spin ROS in a background thread so subscription callbacks continuously
    # update node.data, while THIS (main) thread owns the GLFW viewer — GLFW must
    # run on the main thread. (Calling spin_once inside the render loop processed
    # too few/no callbacks, leaving the window frozen.)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def _spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass  # normal on shutdown (Ctrl-C / process kill)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    try:
        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            while rclpy.ok() and viewer.is_running():
                t0 = time.monotonic()
                mujoco.mj_forward(node.model, node.data)  # kinematics only, no step
                viewer.sync()
                dt = time.monotonic() - t0
                if dt < period:
                    time.sleep(period - dt)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
