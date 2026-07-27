#!/usr/bin/env python3
"""Publish external-wrench disturbances for the Riptide sim (R4).

Publishes riptide_msgs/DisturbanceCommand on /riptide/disturbance, which the
MuJoCo SystemInterface applies to mjData.xfrc_applied on the target body.

Scenarios (parameter `scenario`, changeable live via `ros2 param set`):
  none            zero wrench
  steady_current  constant force along `direction`
  sinusoid        force = amplitude * sin(2*pi*frequency*t) along `direction`
  impulse         short bursts of `amplitude` every `impulse_interval` seconds
"""
import math

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
        self.declare_parameter("publish_rate", 50.0)          # Hz

        self.pub = self.create_publisher(DisturbanceCommand, "/riptide/disturbance", 10)
        rate = float(self.get_parameter("publish_rate").value)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.tick)
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

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        scenario = str(self.get_parameter("scenario").value)
        amp = float(self.get_parameter("amplitude").value)
        d = [float(x) for x in self.get_parameter("direction").value]
        norm = math.sqrt(sum(c * c for c in d)) or 1.0
        d = [c / norm for c in d]

        mag = self._magnitude(scenario, t, amp)

        msg = DisturbanceCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.body = str(self.get_parameter("body").value)
        msg.wrench.force.x = d[0] * mag
        msg.wrench.force.y = d[1] * mag
        msg.wrench.force.z = d[2] * mag
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
