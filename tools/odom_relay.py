#!/usr/bin/env python3
"""Republish the robot pose from ``/odom`` on ``/robot_odom``.

RobotSNAP's bridge publishes the robot odometry on ``/odom``; the crowd-navigation
baselines of the benchmark all subscribe to ``/robot_odom``. The relay is
deliberately blind: same message, same stamp, same frames, only the name changes.

``--alias-frame`` adds one extra identity link, for a method that names the frame
it draws in differently from the one it reads: BRNE plans and draws in
``brne_odom``, and without that link RViz2 cannot place its path next to the map.
"""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

MAP_FRAME = "map"
#: How many times the link is repeated, because a latched topic is latched per writer.
_STATIC_REPEATS = 3


class OdomRelay(Node):
    """One subscription on ``/odom``, one publisher on ``/robot_odom``, one static link."""

    def __init__(self, alias_frame: str = "") -> None:
        super().__init__("odom_relay")
        self._alias_frame = alias_frame
        self._publisher = self.create_publisher(Odometry, "/robot_odom", 10)
        self.create_subscription(Odometry, "/odom", self._relay, 10)
        if alias_frame:
            latching = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._static = self.create_publisher(TFMessage, "/tf_static", latching)
            self._publish_static_link()

    def _relay(self, message: Odometry) -> None:
        self._publisher.publish(message)

    def _publish_static_link(self) -> None:
        """``map`` -> the alias frame, identity: the two names are one frame."""
        transform = TransformStamped()
        # Zero, like every other static link: a static transform has no time, and this node's clock is
        # the wall's while the graph the link joins runs on simulated seconds.
        transform.header.frame_id = MAP_FRAME
        transform.child_frame_id = self._alias_frame
        transform.transform.rotation.w = 1.0
        message = TFMessage(transforms=[transform])
        for _ in range(_STATIC_REPEATS):
            self._static.publish(message)
            time.sleep(0.1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--alias-frame",
        default="",
        help="extra name for the odometry frame, added as an identity link (default: none)",
    )
    arguments = parser.parse_args()

    rclpy.init()
    node = OdomRelay(arguments.alias_frame)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
