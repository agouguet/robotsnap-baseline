#!/usr/bin/env python3
"""Publish the RobotSNAP scene as RViz markers.

    python scene_markers.py [--robot robot_1]

RViz can draw the map, the laser scan and the robot's odometry on its own, but
the two things that make social navigation *social* travel as JSON bodies RViz
cannot read: where the people are, and where the robot is trying to go. This
node reads them and republishes what they say as one ``MarkerArray`` in the
``map`` frame:

* every agent of ``/simulation/agents``, which Unity publishes in the **robot**
  frame, moved into ``map`` by the robot pose of ``/odom`` - the same transform
  CoHAN-Nav2's ``agents_bridge.py`` applies, so the two views agree;
* the robot itself, as an arrow along its heading, taken from ``/odom`` (the one
  pose the gateway publishes that is in the map frame, since ``map -> odom`` is
  the identity);
* the goal, when the task has one, from ``/simulation/state``.

``--robot`` picks which one to draw: the primary robot answers to the bare topic
names, every other to its own ``/robot_<id>/...``.

It is deliberately not method-specific: any integration that wants to *see* the
crowd can put this in a pane. Nothing it publishes is read back by a controller.
"""

from __future__ import annotations

import argparse
import json
import math

import rclpy
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

#: Every agent, in the robot frame, as JSON. RobotSNAP writes it.
AGENTS = "simulation/agents"
#: Robot pose and body twist, in the map frame.
ODOM = "odom"
#: Whole-session snapshot, including the robot's goal, as JSON. Only the whole
#: session has one, so this one is never namespaced.
TOPIC_STATE = "/simulation/state"
#: What this node writes.
TOPIC_MARKERS = "/scene/markers"

#: The robot the bare topic names reach; mirrors ``topics.PRIMARY_ROBOT_ID``.
PRIMARY_ROBOT = "robot_1"


def robot_topic(robot: str, topic: str) -> str:
    """The topic of one robot: bare for the primary, namespaced for the rest.

    RobotSNAP publishes the primary robot under both spellings and every other
    only under its own, so asking for the bare name would silently follow
    ``robot_1`` whatever ``--robot`` says.
    """
    return f"/{topic}" if robot == PRIMARY_ROBOT else f"/{robot}/{topic}"


#: Frame every marker is stamped in. ``/tf_static`` makes ``odom`` the same
#: frame as ``map``, and CoHAN's bridge already publishes its agents here.
WORLD_FRAME = "map"

#: Distinct colours, one per agent, cycled by id so the same person keeps the
#: same colour from one tick to the next. None of them is the robot's green or
#: the goal's magenta: seen from above, an agent is a disc and a goal a dot, and
#: a shared colour would make the two impossible to tell apart.
AGENT_COLORS = (
    (1.00, 0.45, 0.15),
    (0.25, 0.70, 0.95),
    (0.95, 0.80, 0.20),
    (0.60, 0.40, 0.90),
    (0.20, 0.80, 0.75),
    (0.95, 0.40, 0.55),
    (0.70, 0.70, 0.75),
    (0.90, 0.62, 0.35),
)

#: The goal, and the colour nothing else uses.
GOAL_COLOR = (0.90, 0.25, 0.85)

#: Body of an agent, in metres: a person is about this tall and this wide.
AGENT_RADIUS = 0.25
AGENT_HEIGHT = 1.7


def _color(marker: Marker, rgb) -> None:
    marker.color.r, marker.color.g, marker.color.b = rgb
    marker.color.a = 0.9


def yaw_from_quaternion(q) -> float:
    """Planar yaw of a quaternion, in radians."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def _number(value, default: float = 0.0) -> float:
    """``float(value)`` for a real number, ``default`` for anything else.

    ``bool`` is excluded on purpose: ``True`` is an int in Python, and a flag
    read as a coordinate is a bug that shows up as a marker at (1, 0).
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return default
    return float(value)


class SceneMarkers(Node):
    """Republish what the JSON streams say as markers RViz can draw."""

    def __init__(self, robot: str) -> None:
        super().__init__("robotsnap_scene_markers")
        self._robot = robot
        self._odom: Odometry | None = None
        self._agents: String | None = None
        self._goal: tuple[float, float] | None = None

        self.create_subscription(Odometry, robot_topic(robot, ODOM), self._on_odom, 10)
        self.create_subscription(String, robot_topic(robot, AGENTS), self._on_agents, 10)
        self.create_subscription(String, TOPIC_STATE, self._on_state, 10)
        self._publisher = self.create_publisher(MarkerArray, TOPIC_MARKERS, 10)

    # -- inputs ------------------------------------------------------------

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message
        # The robot's own marker follows odometry directly, so it appears even
        # before any crowd does.
        self._republish()

    def _on_agents(self, message: String) -> None:
        self._agents = message
        self._republish()

    def _on_state(self, message: String) -> None:
        try:
            body = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(body, dict) or not body.get("robot_has_goal"):
            self._goal = None
        else:
            goal = body.get("robot_goal")
            self._goal = (
                (_number(goal.get("x")), _number(goal.get("y")))
                if isinstance(goal, dict)
                else None
            )
        self._republish()

    # -- output ------------------------------------------------------------

    def _republish(self) -> None:
        odom = self._odom
        if odom is None:
            return

        # One stamp for the whole array, taken from the pose it is built around:
        # a reader that wants the tick and the crowd at the same instant gets it.
        stamp = odom.header.stamp
        markers = MarkerArray()
        markers.markers.append(self._clear_marker())
        markers.markers.append(self._robot_marker(stamp, odom))
        if self._goal is not None:
            markers.markers.extend(self._goal_markers(stamp))
        markers.markers.extend(self._agent_markers(stamp, odom))
        self._publisher.publish(markers)

    def _clear_marker(self) -> Marker:
        """A ``DELETEALL`` for the next array to start from a blank scene.

        RViz adds and updates the markers of an array but never removes one that
        stopped being sent, so an agent that walks away would keep its cylinder
        on screen for ever - and a crowd that shrinks would leave ghosts. The
        clear is the first element of every array, which RViz applies before the
        markers that follow it, so one tick still shows one coherent scene.
        """
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        marker.action = Marker.DELETEALL
        return marker

    def _robot_marker(self, stamp, odom: Odometry) -> Marker:
        pose = odom.pose.pose
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        marker.header.stamp = stamp
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = 0.05
        marker.pose.orientation = pose.orientation
        # The arrow points along +x of the pose, so its own frame is the heading.
        marker.scale = Vector3(x=0.9, y=0.18, z=0.18)
        _color(marker, (0.20, 0.90, 0.30))
        return marker

    def _goal_markers(self, stamp) -> list[Marker]:
        """The goal: a sphere to see it, and its name to be sure what it is."""
        sphere = Marker()
        sphere.header.frame_id = WORLD_FRAME
        sphere.header.stamp = stamp
        sphere.ns = "goal"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        # Unity writes the goal's height on the ground plane; lift the sphere so
        # it reads as a plateau rather than a dot buried in the floor.
        sphere.pose.position = Point(x=self._goal[0], y=self._goal[1], z=0.4)
        sphere.pose.orientation.w = 1.0
        sphere.scale = Vector3(x=0.6, y=0.6, z=0.6)
        _color(sphere, GOAL_COLOR)

        # The label is raised *and* pushed sideways: in a top-down view height
        # separates nothing, so a label placed straight above the sphere would
        # land right on top of it.
        label = Marker()
        label.header.frame_id = WORLD_FRAME
        label.header.stamp = stamp
        label.ns = "goal"
        label.id = 1
        # TEXT_VIEW_FACING is the only text marker: it always faces the camera,
        # which is what a label wants whatever the view.
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(x=self._goal[0], y=self._goal[1] + 1.1, z=1.3)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.8
        _color(label, GOAL_COLOR)
        label.color.a = 1.0
        label.text = "goal"
        return [sphere, label]

    def _agent_markers(self, stamp, odom: Odometry) -> list[Marker]:
        payload = self._agents
        if payload is None:
            return []
        try:
            body = json.loads(payload.data)
        except (TypeError, ValueError):
            return []
        entries = body.get("agents") if isinstance(body, dict) else None
        if not isinstance(entries, list):
            return []

        pose = odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        robot_x, robot_y = pose.position.x, pose.position.y

        markers: list[Marker] = []
        for index, entry in enumerate(entries):
            if not isinstance(entry, dict):
                continue
            x, y = entry.get("x"), entry.get("y")
            if isinstance(x, bool) or isinstance(y, bool):
                continue
            if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
                continue

            # Robot frame -> map frame, the same rotation CoHAN's bridge uses.
            world_x = robot_x + cos_yaw * x - sin_yaw * y
            world_y = robot_y + sin_yaw * x + cos_yaw * y

            marker = Marker()
            marker.header.frame_id = WORLD_FRAME
            marker.header.stamp = stamp
            marker.ns = "agents"
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position = Point(
                x=world_x, y=world_y, z=AGENT_HEIGHT / 2.0
            )
            marker.pose.orientation.w = 1.0
            marker.scale = Vector3(
                x=2 * AGENT_RADIUS, y=2 * AGENT_RADIUS, z=AGENT_HEIGHT
            )
            _color(marker, AGENT_COLORS[index % len(AGENT_COLORS)])
            markers.append(marker)
        return markers


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot",
        default="robot_1",
        help="robot whose agents and goal are drawn (default: %(default)s)",
    )
    arguments = parser.parse_args(argv)

    rclpy.init()
    node = SceneMarkers(arguments.robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
