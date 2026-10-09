#!/usr/bin/env python3
"""Publish the simulated clock on ``/clock``, taken from the odometry stamp.

HATEB's launch sets ``/use_sim_time`` to true, so every one of its nodes reads
the time from ``/clock`` and does nothing until that clock moves. RobotSNAP does
not emit a clock stream of its own - its ``/clock`` publisher is declared but
silent - while its odometry, laser and map messages are all stamped with the
simulator's own clock. This node echoes that stamp back as a ``Clock`` message,
which is the value the navigation stack and the transforms already use.

Timestamps therefore line up by construction: ``/tf`` carries the odometry
stamp, and ``/clock`` carries the same number, so a lookup "at the current time"
lands exactly on the last transform instead of between two of them.

The ROS 1 <-> ROS 2 bridge forwards ``/clock`` like any other topic, so a ROS 1
method that shares the graph reads this clock too.

Reading the state stream instead of the odometry would work as well; the
odometry is chosen because its rate is the one the transforms are published at.
"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

#: The stream whose stamps drive the clock. Its rate becomes the clock's rate.
SOURCE_TOPIC = "/odom"


class ClockRelay(Node):
    """Re-publish the stamp of every odometry message as a ``Clock`` message."""

    def __init__(self) -> None:
        super().__init__("clock_relay")
        self._publisher = self.create_publisher(Clock, "/clock", 10)
        self.create_subscription(Odometry, SOURCE_TOPIC, self._on_source, 10)

    def _on_source(self, message: Odometry) -> None:
        clock = Clock()
        clock.clock = message.header.stamp
        self._publisher.publish(clock)


def main() -> None:
    rclpy.init()
    node = ClockRelay()
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
