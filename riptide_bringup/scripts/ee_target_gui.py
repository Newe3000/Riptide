#!/usr/bin/env python3
"""Teleop GUI for the desired end-effector Cartesian pose.

Publishes geometry_msgs/PoseStamped on /riptide/ee_target (world frame), which
EeStabilizationController tracks in real time. Press-and-HOLD an arrow to move
the target at a constant velocity; releasing stops it. The pose integrates and
republishes continuously (~30 Hz).

To avoid snapping the arm when you take control, the GUI seeds its initial pose
from /riptide/ee_target/current (published latched by the controller) and only
starts commanding once seeded.

Run:   ros2 run riptide_bringup ee_target_gui.py
       (or via the launch:  ... ee_gui:=true)

The ROS logic lives in EeTargetPublisher (no GUI toolkit) so it can be tested
headless; run_gui() is the thin Tkinter view on top.
"""
import math
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    )


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]


class EeTargetPublisher(Node):
    """ROS logic for the teleop, independent of any GUI toolkit (unit-testable)."""

    def __init__(self, lin_speed=0.15, ang_speed=0.6):
        super().__init__("ee_target_gui")
        self.lin_speed = lin_speed          # m/s while a translation arrow is held
        self.ang_speed = ang_speed          # rad/s while a rotation arrow is held
        self._lock = threading.Lock()
        self.pos = [0.4, 0.0, 1.1]          # world x y z (seeded from /current)
        self.rpy = [0.0, 0.0, 0.0]
        self.lin_vel = [0.0, 0.0, 0.0]      # commanded vel from held buttons
        self.ang_vel = [0.0, 0.0, 0.0]
        self._seeded = False

        self.pub = self.create_publisher(PoseStamped, "/riptide/ee_target", 10)
        seed_qos = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            PoseStamped, "/riptide/ee_target/current", self._on_current, seed_qos)

    # --- state the GUI drives ------------------------------------------------
    def set_lin_vel(self, axis, value):
        with self._lock:
            self.lin_vel[axis] = value

    def set_ang_vel(self, axis, value):
        with self._lock:
            self.ang_vel[axis] = value

    def stop(self):
        with self._lock:
            self.lin_vel = [0.0, 0.0, 0.0]
            self.ang_vel = [0.0, 0.0, 0.0]

    def snapshot(self):
        with self._lock:
            return list(self.pos), list(self.rpy), self._seeded

    # --- seed / integrate / publish -----------------------------------------
    def _on_current(self, msg):
        with self._lock:
            if self._seeded:
                return  # seed once only, before the user takes control
            self.pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.rpy = quat_to_rpy(
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w)
            self._seeded = True
        self.get_logger().info(
            "seeded EE target to current [%.3f, %.3f, %.3f]" % tuple(self.pos))

    def step(self, dt):
        """Integrate held-button velocities into the target and publish it."""
        with self._lock:
            if not self._seeded:
                return  # wait until we know the live target (avoids snapping)
            for i in range(3):
                self.pos[i] += self.lin_vel[i] * dt
                self.rpy[i] += self.ang_vel[i] * dt
            self.pos[2] = max(self.pos[2], 0.05)   # stay above the seabed
            pos, rpy = list(self.pos), list(self.rpy)
        self._publish(pos, rpy)

    def _publish(self, pos, rpy):
        qx, qy, qz, qw = rpy_to_quat(*rpy)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub.publish(msg)


def run_gui(node):
    import tkinter as tk

    root = tk.Tk()
    root.title("Riptide — EE Teleop")

    def arrow(parent, text, axis, sign, rot=False):
        b = tk.Button(parent, text=text, width=7, height=2)
        setv = node.set_ang_vel if rot else node.set_lin_vel
        speed = node.ang_speed if rot else node.lin_speed
        b.bind("<ButtonPress-1>", lambda e: setv(axis, sign * speed))
        b.bind("<ButtonRelease-1>", lambda e: setv(axis, 0.0))
        return b

    trans = tk.LabelFrame(root, text="Position (world, m) — press & hold", padx=6, pady=6)
    trans.grid(row=0, column=0, padx=8, pady=8, sticky="n")
    arrow(trans, "+X (fwd)", 0, +1).grid(row=0, column=1)
    arrow(trans, "-X (back)", 0, -1).grid(row=2, column=1)
    arrow(trans, "+Y (left)", 1, +1).grid(row=1, column=0)
    arrow(trans, "-Y (right)", 1, -1).grid(row=1, column=2)
    arrow(trans, "+Z (up)", 2, +1).grid(row=0, column=3, padx=(16, 0))
    arrow(trans, "-Z (down)", 2, -1).grid(row=2, column=3, padx=(16, 0))

    rotf = tk.LabelFrame(root, text="Orientation (rad) — press & hold", padx=6, pady=6)
    rotf.grid(row=1, column=0, padx=8, pady=4, sticky="n")
    for col, (name, ax) in enumerate([("Roll", 0), ("Pitch", 1), ("Yaw", 2)]):
        arrow(rotf, f"+{name}", ax, +1, rot=True).grid(row=0, column=col, padx=4)
        arrow(rotf, f"-{name}", ax, -1, rot=True).grid(row=1, column=col, padx=4)

    status = tk.Label(root, text="waiting for /riptide/ee_target/current …",
                      font=("TkFixedFont", 10), justify="left")
    status.grid(row=2, column=0, padx=8, pady=(4, 8), sticky="w")

    tk.Button(root, text="STOP", command=node.stop, width=10).grid(
        row=3, column=0, pady=(0, 8))

    root.protocol("WM_DELETE_WINDOW", root.quit)   # closing the window exits
    dt = 1.0 / 30.0

    def tick():
        # ROS is spun in a background thread (see main); here we only integrate
        # the held-button velocities into the target, publish, and refresh labels.
        if not rclpy.ok():        # external shutdown (Ctrl-C / launch stop) -> exit
            root.quit()
            return
        node.step(dt)
        pos, rpy, seeded = node.snapshot()
        if seeded:
            status.config(text="target  x=%+.3f  y=%+.3f  z=%+.3f\n"
                               "        R=%+.2f  P=%+.2f  Y=%+.2f" %
                               (pos[0], pos[1], pos[2], rpy[0], rpy[1], rpy[2]))
        root.after(int(dt * 1000), tick)

    root.after(int(dt * 1000), tick)
    root.mainloop()


def main():
    rclpy.init()
    node = EeTargetPublisher()
    # Spin in the background so the /current seed is received reliably while the
    # Tk main loop owns the window and drives publishing via node.step().
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def _spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass  # normal on shutdown (Ctrl-C / process kill)

    threading.Thread(target=_spin, daemon=True).start()
    try:
        run_gui(node)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
