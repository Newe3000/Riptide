#!/usr/bin/env python3
"""Publish external disturbances for the Riptide sim.

Two physically distinct disturbance channels are published every tick; the
inactive one is held at zero so switching `scenario` live cleanly resets it:

  * OCEAN CURRENT -- a flow *velocity* [m/s] on /riptide/current
    (geometry_msgs/Vector3Stamped, world frame). The MuJoCo SystemInterface
    feeds it into the fluid model as `wind`, so drag acts on EVERY submerged
    geom relative to (v_body - v_current). The current therefore pushes the
    whole structure -- hull AND every arm link -- distributed and
    velocity-dependent, not as one lumped force on the base.

  * POINT WRENCH -- riptide_msgs/DisturbanceCommand on /riptide/disturbance,
    applied to mjData.xfrc_applied on the target body. Used for a localized
    hit (e.g. a collision / bump), which really is a point force.

Scenarios (parameter `scenario`, changeable live via `ros2 param set`):
  none            no disturbance
  steady_current  constant current along `direction` at `current_speed`
  sinusoid        current = current_speed * sin(2*pi*frequency*t) along `direction`
  impulse         short POINT-WRENCH bursts of `amplitude` [N] every
                  `impulse_interval` seconds (a bump, not a flow)
  stochastic      realistic turbulent current: a mean flow (`current_speed`
                  along `direction`) plus first-order Gauss-Markov
                  (Ornstein-Uhlenbeck) turbulence on the current velocity --
                  temporally correlated, mean-reverting colored noise, the
                  standard model for slowly varying ocean currents (Fossen 2011).
                  The distributed drag turns this into fluctuating force AND
                  torque on the whole body automatically.
                  Knobs: `turbulence_std` (m/s, per-axis stationary velocity std),
                  `correlation_time` (s, the OU time const),
                  `seed` (>=0 reproducible, <0 nondeterministic).
"""
import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3Stamped
from riptide_msgs.msg import DisturbanceCommand


class DisturbanceGenerator(Node):
    def __init__(self):
        super().__init__("disturbance_generator")
        self.declare_parameter("scenario", "steady_current")
        self.declare_parameter("body", "auv_base_link")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("direction", [1.0, 0.0, 0.0])  # will be normalized
        self.declare_parameter("current_speed", 1.0)          # m/s (flow scenarios)
        self.declare_parameter("amplitude", 80.0)             # N   (impulse wrench)
        self.declare_parameter("frequency", 0.2)              # Hz (sinusoid)
        self.declare_parameter("impulse_interval", 5.0)       # s
        self.declare_parameter("impulse_duration", 0.3)       # s
        # Stochastic (turbulent current) parameters.
        self.declare_parameter("turbulence_std", 0.3)         # m/s, per-axis velocity
        self.declare_parameter("correlation_time", 2.0)       # s, OU time constant
        self.declare_parameter("seed", 42)                    # <0 => nondeterministic
        self.declare_parameter("publish_rate", 50.0)          # Hz

        self.current_pub = self.create_publisher(Vector3Stamped, "/riptide/current", 10)
        self.pub = self.create_publisher(DisturbanceCommand, "/riptide/disturbance", 10)
        rate = float(self.get_parameter("publish_rate").value)
        self._dt = 1.0 / max(rate, 1.0)
        self.t0 = self.get_clock().now()

        # Ornstein-Uhlenbeck state (zero-mean turbulent velocity fluctuation).
        seed = int(self.get_parameter("seed").value)
        self._rng = random.Random(seed if seed >= 0 else None)
        self._v_ou = [0.0, 0.0, 0.0]

        self.timer = self.create_timer(self._dt, self.tick)
        self.get_logger().info(
            f"disturbance_generator up: scenario='{self.get_parameter('scenario').value}'")

    def _step_ou(self, state, std, tau, dt):
        """Advance one first-order Gauss-Markov (OU) step, in place.

        Exact discretization of  dx = -(1/tau) x dt + q dW  whose stationary
        std is `std`:  x <- a x + std*sqrt(1-a^2) * N(0,1),  a = exp(-dt/tau).
        Correlated over ~tau seconds, mean-reverting, zero mean.
        """
        a = math.exp(-dt / max(tau, 1e-3))
        k = std * math.sqrt(max(1.0 - a * a, 0.0))
        for i in range(3):
            state[i] = a * state[i] + k * self._rng.gauss(0.0, 1.0)

    def _current_velocity(self, scenario, t, speed, d):
        """World-frame current velocity [m/s] for the flow scenarios."""
        if scenario == "steady_current":
            return [d[i] * speed for i in range(3)]
        if scenario == "sinusoid":
            f = float(self.get_parameter("frequency").value)
            mag = speed * math.sin(2.0 * math.pi * f * t)
            return [d[i] * mag for i in range(3)]
        if scenario == "stochastic":
            tau = float(self.get_parameter("correlation_time").value)
            v_std = float(self.get_parameter("turbulence_std").value)
            self._step_ou(self._v_ou, v_std, tau, self._dt)
            # Mean current along `direction` + isotropic turbulent fluctuation.
            return [d[i] * speed + self._v_ou[i] for i in range(3)]
        return [0.0, 0.0, 0.0]  # "none" / not a flow scenario

    def _impulse_force(self, t, amp, d):
        """Point-wrench force [N] for the impulse scenario (0 otherwise)."""
        if str(self.get_parameter("scenario").value) != "impulse":
            return [0.0, 0.0, 0.0]
        interval = float(self.get_parameter("impulse_interval").value)
        duration = float(self.get_parameter("impulse_duration").value)
        mag = amp if (t % interval) < duration else 0.0
        return [d[0] * mag, d[1] * mag, d[2] * mag]

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        scenario = str(self.get_parameter("scenario").value)
        speed = float(self.get_parameter("current_speed").value)
        amp = float(self.get_parameter("amplitude").value)
        d = [float(x) for x in self.get_parameter("direction").value]
        norm = math.sqrt(sum(c * c for c in d)) or 1.0
        d = [c / norm for c in d]

        stamp = self.get_clock().now().to_msg()
        frame = str(self.get_parameter("frame_id").value)

        # --- ocean current (flow velocity -> MuJoCo wind, acts everywhere) ---
        vel = self._current_velocity(scenario, t, speed, d)
        cur = Vector3Stamped()
        cur.header.stamp = stamp
        cur.header.frame_id = frame
        cur.vector.x, cur.vector.y, cur.vector.z = vel
        self.current_pub.publish(cur)

        # --- point wrench (localized hit; zero unless impulse) ---------------
        force = self._impulse_force(t, amp, d)
        msg = DisturbanceCommand()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        msg.body = str(self.get_parameter("body").value)
        msg.wrench.force.x = force[0]
        msg.wrench.force.y = force[1]
        msg.wrench.force.z = force[2]
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DisturbanceGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
