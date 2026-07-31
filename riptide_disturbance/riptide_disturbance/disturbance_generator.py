#!/usr/bin/env python3
"""Publish external-wrench disturbances for the Riptide sim (R4).

Publishes riptide_msgs/DisturbanceCommand on /riptide/disturbance, which the
MuJoCo SystemInterface applies to mjData.xfrc_applied on the target body.

Scenarios (parameter `scenario`, changeable live via `ros2 param set`):
  none            zero wrench
  steady_current  constant force along `direction`
  sinusoid        force = amplitude * sin(2*pi*frequency*t) along `direction`
  impulse         short bursts of `amplitude` every `impulse_interval` seconds
  stochastic      realistic turbulent current: a mean flow (`amplitude` along
                  `direction`) plus first-order Gauss-Markov (Ornstein-Uhlenbeck)
                  turbulence on force AND torque -- temporally correlated,
                  mean-reverting colored noise, the standard model for slowly
                  varying ocean-current / environmental loads (Fossen 2011).
                  Knobs: `turbulence_std` (N, per-axis stationary force std),
                  `torque_std` (N.m), `correlation_time` (s, the OU time const),
                  `seed` (>=0 reproducible, <0 nondeterministic).
"""
import math
import random

import rclpy
from rclpy.node import Node

from riptide_msgs.msg import DisturbanceCommand


class DisturbanceGenerator(Node):
    def __init__(self):
        super().__init__("disturbance_generator")
        self.declare_parameter("scenario", "steady_current")
        self.declare_parameter("body", "auv_base_link")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("direction", [1.0, 0.0, 0.0])  # will be normalized
        self.declare_parameter("amplitude", 80.0)             # N
        self.declare_parameter("frequency", 0.2)              # Hz (sinusoid)
        self.declare_parameter("impulse_interval", 5.0)       # s
        self.declare_parameter("impulse_duration", 0.3)       # s
        # Stochastic (turbulent current) parameters.
        self.declare_parameter("turbulence_std", 25.0)        # N, per-axis force
        self.declare_parameter("torque_std", 5.0)             # N.m, per-axis
        self.declare_parameter("correlation_time", 2.0)       # s, OU time constant
        self.declare_parameter("seed", 42)                    # <0 => nondeterministic
        self.declare_parameter("publish_rate", 50.0)          # Hz

        self.pub = self.create_publisher(DisturbanceCommand, "/riptide/disturbance", 10)
        rate = float(self.get_parameter("publish_rate").value)
        self._dt = 1.0 / max(rate, 1.0)
        self.t0 = self.get_clock().now()

        # Ornstein-Uhlenbeck state (zero-mean turbulent fluctuation, force + torque).
        seed = int(self.get_parameter("seed").value)
        self._rng = random.Random(seed if seed >= 0 else None)
        self._f_ou = [0.0, 0.0, 0.0]
        self._t_ou = [0.0, 0.0, 0.0]

        self.timer = self.create_timer(self._dt, self.tick)
        self.get_logger().info(
            f"disturbance_generator up: scenario='{self.get_parameter('scenario').value}'")

    def _magnitude(self, scenario: str, t: float, amp: float) -> float:
        if scenario == "steady_current":
            return amp
        if scenario == "sinusoid":
            f = float(self.get_parameter("frequency").value)
            return amp * math.sin(2.0 * math.pi * f * t)
        if scenario == "impulse":
            interval = float(self.get_parameter("impulse_interval").value)
            duration = float(self.get_parameter("impulse_duration").value)
            return amp if (t % interval) < duration else 0.0
        return 0.0  # "none" or unknown

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

    def _stochastic_wrench(self, amp, d):
        tau = float(self.get_parameter("correlation_time").value)
        f_std = float(self.get_parameter("turbulence_std").value)
        t_std = float(self.get_parameter("torque_std").value)
        self._step_ou(self._f_ou, f_std, tau, self._dt)
        self._step_ou(self._t_ou, t_std, tau, self._dt)
        # Mean current along `direction` + isotropic turbulent fluctuation.
        force = [d[i] * amp + self._f_ou[i] for i in range(3)]
        torque = list(self._t_ou)
        return force, torque

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        scenario = str(self.get_parameter("scenario").value)
        amp = float(self.get_parameter("amplitude").value)
        d = [float(x) for x in self.get_parameter("direction").value]
        norm = math.sqrt(sum(c * c for c in d)) or 1.0
        d = [c / norm for c in d]

        if scenario == "stochastic":
            force, torque = self._stochastic_wrench(amp, d)
        else:
            mag = self._magnitude(scenario, t, amp)
            force = [d[0] * mag, d[1] * mag, d[2] * mag]
            torque = [0.0, 0.0, 0.0]

        msg = DisturbanceCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.body = str(self.get_parameter("body").value)
        msg.wrench.force.x = force[0]
        msg.wrench.force.y = force[1]
        msg.wrench.force.z = force[2]
        msg.wrench.torque.x = torque[0]
        msg.wrench.torque.y = torque[1]
        msg.wrench.torque.z = torque[2]
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
