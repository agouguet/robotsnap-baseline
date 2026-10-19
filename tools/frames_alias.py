#!/usr/bin/env python3
"""Publish the robot frame names a ROS 2 navigation stack hardcodes.

RobotSNAP names the robot body after the prefab that was loaded: the Jackal
publishes its odometry under ``chassis_link`` while the Freight, the Kuri, the
Ginger and the Bibus publish it under ``base_link``. The stacks of the benchmark
do not all take that name everywhere:

* ``invisible_humans_detection`` writes it as ``#define FOOTPRINT_FRAME
  "base_footprint"`` in C++ - not a parameter at all;
* the HATEB planner's ROS wrapper and its visualisation use the literal
  ``base_link`` (``hateb_local_planner_ros.cpp``, ``visualization.cpp``), even
  though ``base_frame`` is a parameter elsewhere in the same planner;
* nav2's own example parameters name ``base_link`` and ``base_footprint`` as
  their ``robot_base_frame``, and are shipped here unmodified.

Each of those logs its own version of
``"... passed to lookupTransform argument source_frame does not exist"`` and,
for TEB, then plans against a robot pose it cannot find - which shows up as a
controller that only ever asks for a few centimetres per second.

Two rules keep this safe for every robot type:

* the links hang off the body frame the session actually uses - the
  ``child_frame_id`` of ``/odom`` - and never off a hardcoded ``chassis_link``.
  Declaring ``chassis_link -> base_link`` while the odometry already declares
  ``odom -> base_link`` gives one frame two parents, which tf2 resolves by
  dropping one of them: the tree then holds two unconnected roots and a stack
  asking for ``odom -> base_link`` is told the frames do not exist, which is a
  costmap that stops following the robot and a planner that stops sampling;
* a link is declared only when it adds a name - ``base_link`` is not declared as
  a child of itself - so the body frame keeps exactly one parent.

The link set is latched, so a reader that connects after this node still gets it;
what changes with the robot type is only which frame the names hang off.

Unlike the ROS 1 CoHAN in ``../hateb/``, no bridge stands between this node and
the consumers, so a latched ``/tf_static`` publisher is what a static link should
be and is all that is needed.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

#: The names a stack hardcodes, in the order they are declared.
HARDCODED_NAMES: tuple[str, ...] = ("base_link", "base_footprint")


class FramesAlias(Node):
    """Publish the hardcoded frame names as identity links on ``/tf_static``."""

    def __init__(self) -> None:
        super().__init__("frames_alias")
        latching = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(TFMessage, "/tf_static", latching)
        self._body = ""
        self._declared: tuple[str, ...] = ()
        self.create_subscription(Odometry, "/odom", self._on_odometry, 20)
        self.get_logger().info(
            "waiting for /odom to learn the body frame this robot publishes under"
        )

    def _on_odometry(self, message: Odometry) -> None:
        body = str(message.child_frame_id or "").strip().lstrip("/")
        if not body or body == self._body:
            return
        self._body = body
        self._publish()

    def _links(self) -> tuple[tuple[str, str], ...]:
        """``parent -> child`` links to declare, all identity, given the body frame.

        The body frame is the one the session put its odometry under. Every
        hardcoded name becomes a child of it, and never a child of itself: a
        ``base_link -> base_link`` link would be a loop, and a second parent for
        the body frame would cut the tree in two.
        """
        return tuple(
            (self._body, name) for name in HARDCODED_NAMES if name != self._body
        )

    def _publish(self) -> None:
        if not self._body:
            return
        links = self._links()
        transforms = []
        for parent, child in links:
            transform = TransformStamped()
            # The stamp is left at zero, which is what "this link has no time" means to tf2, and what
            # the mirror's own static links carry. Stamping it from this node's clock put a wall-clock
            # instant (about 1.8e9 seconds) on a topic the rest of the graph reads in simulated
            # seconds (a few hundred): tf2 keeps a static link outside its time index so the value is
            # never *used*, but a tree whose links disagree about which clock they are on is a tree no
            # reader can be reasoned about, and it sent one diagnosis down the wrong road.
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self._publisher.publish(TFMessage(transforms=transforms))
        names = tuple(f"{p} -> {c}" for p, c in links)
        if names != self._declared:
            self._declared = names
            self.get_logger().info(
                f"body frame {self._body}: declared " + (", ".join(names) or "nothing")
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
