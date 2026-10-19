#!/usr/bin/env python3
"""Publish the robot frame names CoHAN-Nav2 hardcodes, as identity links.

RobotSNAP names the robot body ``chassis_link``. CoHAN-Nav2 does not take that
name everywhere:

* ``invisible_humans_detection`` writes it as ``#define FOOTPRINT_FRAME
  "base_footprint"`` in C++ - not a parameter at all;
* the HATEB planner's ROS wrapper and its visualisation use the literal
  ``base_link`` (``hateb_local_planner_ros.cpp``, ``visualization.cpp``), even
  though ``base_frame`` is a parameter elsewhere in the same planner.

Each of those logs its own version of
``"... passed to lookupTransform argument source_frame does not exist"`` and,
for TEB, then plans against a robot pose it cannot find - which shows up as a
controller that only ever asks for a few centimetres per second.

The configuration files are parameterised and already point at ``chassis_link``;
this covers the parts that are not. The names are the same physical frame, so
every link is the identity.

Unlike the ROS 1 CoHAN in ``../hateb/``, no bridge stands between this node and
the consumers, so a latched ``/tf_static`` publisher is what a static link should
be and is all that is needed.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

#: ``parent -> child`` links to declare, all identity. The left name is the one
#: RobotSNAP publishes, the right names are the ones CoHAN-Nav2 asks for.
ALIASES: tuple[tuple[str, str], ...] = (
    ("chassis_link", "base_link"),
    ("chassis_link", "base_footprint"),
)


class FramesAlias(Node):
    """Publish the identity links of :data:`ALIASES` on ``/tf_static``."""

    def __init__(self) -> None:
        super().__init__("frames_alias")
        latching = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(TFMessage, "/tf_static", latching)
        self._publish()

    def _publish(self) -> None:
        transforms = []
        for parent, child in ALIASES:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self._publisher.publish(TFMessage(transforms=transforms))
        self.get_logger().info(
            "declared: " + ", ".join(f"{p} -> {c}" for p, c in ALIASES)
        )


def main() -> None:
    rclpy.init()
    node = FramesAlias()
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
