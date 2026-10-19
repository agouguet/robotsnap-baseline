#!/usr/bin/env python3
"""Republish RobotSNAP agent snapshots as a world-frame ``AgentArray``.

RobotSNAP publishes ``/simulation/agents`` as JSON with every agent expressed in
the robot frame (FLU: x forward, y left). The crowd-navigation baselines of the
benchmark - BRNE, CADRL, SARL* - all read ``/social_sim/agents``
(``agents_msgs/AgentArray``) with the agents in the world frame, so this node
applies the robot pose read on ``/odom``:

    p_world = p_robot + R(yaw) . p_agent
    v_world = R(yaw) . v_agent

It is why a method whose own code is untouched can still be driven by RobotSNAP:
the frame and the message name are adapted here, in one place, for all of them.

No odometry yet means nothing is published; an invalid JSON body, a non-list
``agents`` field or an empty list is ignored without raising.
"""

from __future__ import annotations

import json
import math

import rclpy
from agents_msgs.msg import Agent, AgentArray
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

# Below this squared displacement two positions are too close to give a heading,
# so the agent keeps an identity orientation.
_MIN_DISPLACEMENT_SQ = 1e-4

_IDENTITY_ORIENTATION = (0.0, 0.0, 0.0, 1.0)


def yaw_from_quaternion(q) -> float:
    """Planar yaw of a quaternion, in radians."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Quaternion ``(x, y, z, w)`` for a rotation about z."""
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def number(value) -> float | None:
    """``float`` for a real number, ``None`` for anything else (bools included)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def agent_id(entry: dict) -> int | None:
    """The numeric index of one agent of ``/simulation/agents``.

    RobotSNAP writes the id as a JSON *string* - ``{"id": "0", ...}`` - while this node
    used to demand a bare number, and so rejected every entry of a current session:
    ``/social_sim/agents`` stayed empty and the planner was told there was nobody
    around. Both spellings name the same human, and the message field is a ``uint64``,
    so the identifier is converted here rather than trusted to arrive as a number.
    """
    raw = entry.get("id")
    if isinstance(raw, bool) or raw is None:
        return None
    if isinstance(raw, int):
        return raw
    if isinstance(raw, float):
        return int(raw) if raw.is_integer() else None
    if isinstance(raw, str):
        try:
            return int(raw.strip())
        except ValueError:
            return None
    return None


class AgentsAdapter(Node):
    """Turn robot-frame agent snapshots into world-frame ``AgentArray`` messages."""

    def __init__(self) -> None:
        super().__init__("agents_adapter")
        self._odom: Odometry | None = None
        self._last_positions: dict[int, tuple[float, float]] = {}
        self.create_subscription(Odometry, "/odom", self._store_odom, 10)
        self.create_subscription(String, "/simulation/agents", self._on_agents, 10)
        self._publisher = self.create_publisher(AgentArray, "/social_sim/agents", 10)

    def _store_odom(self, message: Odometry) -> None:
        self._odom = message

    def _on_agents(self, message: String) -> None:
        odom = self._odom
        if odom is None:
            return
        array = self._to_array(message.data, odom)
        if array is not None:
            self._publisher.publish(array)

    def _to_array(self, payload: str, odom: Odometry) -> AgentArray | None:
        try:
            body = json.loads(payload)
        except (TypeError, ValueError):
            self.get_logger().warning("ignoring /simulation/agents: payload is not JSON")
            return None
        if not isinstance(body, dict):
            return None
        entries = body.get("agents")
        if not isinstance(entries, list) or not entries:
            return None

        pose = odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        robot_x, robot_y = pose.position.x, pose.position.y
        frame = odom.header.frame_id or "odom"

        array = AgentArray()
        array.header.stamp = odom.header.stamp
        array.header.frame_id = frame

        seen: set[int] = set()
        for entry in entries:
            agent = self._to_agent(
                entry, odom.header.stamp, frame, robot_x, robot_y, cos_yaw, sin_yaw
            )
            if agent is None:
                continue
            array.agents.append(agent)
            seen.add(agent.id)

        if not array.agents:
            return None
        # Forget the agents that left the snapshot so the heading memory stays bounded.
        self._last_positions = {
            agent_id: position
            for agent_id, position in self._last_positions.items()
            if agent_id in seen
        }
        return array

    def _to_agent(
        self,
        entry,
        stamp,
        frame: str,
        robot_x: float,
        robot_y: float,
        cos_yaw: float,
        sin_yaw: float,
    ) -> Agent | None:
        if not isinstance(entry, dict):
            return None
        identifier = agent_id(entry)
        if identifier is None:
            return None
        x, y = number(entry.get("x")), number(entry.get("y"))
        if x is None or y is None:
            return None

        world_x = robot_x + cos_yaw * x - sin_yaw * y
        world_y = robot_y + sin_yaw * x + cos_yaw * y
        vx, vy = number(entry.get("vx")) or 0.0, number(entry.get("vy")) or 0.0

        agent = Agent()
        agent.header.stamp = stamp
        agent.header.frame_id = frame
        agent.id = identifier
        agent.pose.position.x = world_x
        agent.pose.position.y = world_y
        agent.pose.position.z = number(entry.get("z")) or 0.0
        qx, qy, qz, qw = self._orientation(identifier, world_x, world_y)
        agent.pose.orientation.x = qx
        agent.pose.orientation.y = qy
        agent.pose.orientation.z = qz
        agent.pose.orientation.w = qw
        agent.velocity.linear.x = cos_yaw * vx - sin_yaw * vy
        agent.velocity.linear.y = sin_yaw * vx + cos_yaw * vy
        agent.velocity.linear.z = number(entry.get("vz")) or 0.0
        agent.visible_by_robot = entry.get("visible") is True
        return agent

    def _orientation(
        self, agent_id: int, world_x: float, world_y: float
    ) -> tuple[float, float, float, float]:
        """Heading of an agent, taken from how far it moved since the last snapshot."""
        previous = self._last_positions.get(agent_id)
        self._last_positions[agent_id] = (world_x, world_y)
        if previous is None:
            return _IDENTITY_ORIENTATION
        dx, dy = world_x - previous[0], world_y - previous[1]
        if dx * dx + dy * dy <= _MIN_DISPLACEMENT_SQ:
            return _IDENTITY_ORIENTATION
        return quaternion_from_yaw(math.atan2(dy, dx))


def main() -> None:
    rclpy.init()
    node = AgentsAdapter()
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
