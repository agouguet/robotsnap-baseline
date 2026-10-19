#!/usr/bin/env python3
"""Turn RobotSNAP's agent snapshots into the ``/tracked_agents`` CoHAN-Nav2 reads.

CoHAN-Nav2 follows the same contract as the ROS 1 CoHAN: everything the planner
knows about people arrives on one ``cohan_msgs/TrackedAgents`` message. Its own
``cohan_sim_navigation/agents_bridge.py`` builds that message from one odometry
per simulated human (``/humanN/base_pose_ground_truth``), which RobotSNAP does
not publish: RobotSNAP sends *every* agent at once, as JSON, in ``/simulation/agents``
and expressed in the robot frame (FLU: x forward, y left).

This node is the same idea against that source. It applies the robot pose read
on ``/odom`` to put each agent in the world frame, exactly as
``agents_adapter.py`` does for BRNE and HATEB:

    p_world = p_robot + R(yaw) . p_agent
    v_world = R(yaw) . v_agent

and publishes one ``TrackedAgent`` of type ``HUMAN`` per entry, each carrying a
single ``TORSO`` segment - the segment type both the planner and the invisible
human detection read.

The robot itself is deliberately not part of the message: the upstream bridge
only adds it when the node runs in a namespace, and the planner takes the robot
pose from TF.
"""

from __future__ import annotations

import json
import math

import rclpy
from cohan_msgs.msg import (
    AgentType,
    TrackedAgent,
    TrackedAgents,
    TrackedSegment,
    TrackedSegmentType,
)
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, String

#: A non-blocking signal from scenario_goal after the Nav2 goal was canceled.
EPISODE_RESET_TOPIC = "/robotsnap/episode_reset"

#: The frame every tracked agent is expressed in, as the upstream bridge does.
WORLD_FRAME = "map"
#: Speed, in m/s, above which an agent is reported as MOVING rather than STATIC.
_MOVING_SPEED = 0.05


def yaw_from_quaternion(q) -> float:
    """Planar yaw of a quaternion, in radians."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def number(value) -> float | None:
    """``float`` for a real number, ``None`` for anything else (bools included)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def agent_id(entry: dict) -> str | None:
    """The id of one agent of ``/simulation/agents``, as the session spells it.

    RobotSNAP writes the id as a JSON *string* - ``{"id": "0", ...}`` - and the rest
    of the project reads it that way: ``robotsnap.envs.world`` does
    ``str(entry.get("id", ""))`` and the HumanRoute analyser does
    ``int(entry["id"])``. A bare number is still accepted so an older session keeps
    working, but demanding one - which is what this node did - rejected every entry
    of a current one and left ``/tracked_agents`` published but always empty.
    """
    raw = entry.get("id")
    if isinstance(raw, bool) or raw is None:
        return None
    if isinstance(raw, int):
        return str(raw)
    if isinstance(raw, float):
        # A whole number that arrived in JSON without quotes is still an index, not a name.
        return str(int(raw)) if raw.is_integer() else None
    if isinstance(raw, str) and raw.strip():
        return raw.strip()
    return None


class TrackedAgentsBridge(Node):
    """Publish ``/tracked_agents`` from ``/simulation/agents`` and ``/odom``."""

    def __init__(self) -> None:
        super().__init__("agents_bridge")
        self._odom: Odometry | None = None
        self.create_subscription(Odometry, "/odom", self._store_odom, 10)
        self.create_subscription(String, "/simulation/agents", self._on_agents, 10)
        self.create_subscription(Empty, EPISODE_RESET_TOPIC, self._on_episode_reset, 10)
        self._publisher = self.create_publisher(TrackedAgents, "/tracked_agents", 10)

    def _store_odom(self, message: Odometry) -> None:
        self._odom = message

    def _on_agents(self, message: String) -> None:
        odom = self._odom
        if odom is None:
            # Without a robot pose the agents stay in a frame nobody else shares.
            return
        tracked = self._to_tracked(message.data, odom)
        if tracked is not None:
            self._publisher.publish(tracked)

    def _on_episode_reset(self, _message: Empty) -> None:
        """Forget the previous episode's pose and clear the planner's crowd."""
        self._odom = None
        tracked = TrackedAgents()
        tracked.header.frame_id = WORLD_FRAME
        tracked.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(tracked)
        self.get_logger().info(
            "episode reset: odometry forgotten and empty /tracked_agents published"
        )

    def _to_tracked(self, payload: str, odom: Odometry) -> TrackedAgents | None:
        try:
            body = json.loads(payload)
        except (TypeError, ValueError):
            self.get_logger().warning("ignoring /simulation/agents: payload is not JSON")
            return None
        if not isinstance(body, dict):
            return None
        entries = body.get("agents")
        if not isinstance(entries, list):
            return None

        pose = odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        robot_x, robot_y = pose.position.x, pose.position.y

        tracked = TrackedAgents()
        tracked.header.stamp = odom.header.stamp
        tracked.header.frame_id = WORLD_FRAME

        for entry in entries:
            agent = self._to_agent(
                entry, robot_x, robot_y, cos_yaw, sin_yaw
            )
            if agent is None:
                continue
            agent.track_id = len(tracked.agents) + 1
            tracked.agents.append(agent)

        # An empty list is a valid message - the crowd really is empty - so it is
        # published rather than dropped: the planner has to be told the agents went away.
        return tracked

    def _to_agent(
        self, entry, robot_x: float, robot_y: float, cos_yaw: float, sin_yaw: float
    ) -> TrackedAgent | None:
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
        world_vx = cos_yaw * vx - sin_yaw * vy
        world_vy = sin_yaw * vx + cos_yaw * vy

        segment = TrackedSegment()
        segment.type = TrackedSegmentType.TORSO
        segment.pose.pose.position.x = world_x
        segment.pose.pose.position.y = world_y
        segment.pose.pose.position.z = number(entry.get("z")) or 0.0
        segment.pose.pose.orientation.w = 1.0
        segment.twist.twist.linear.x = world_vx
        segment.twist.twist.linear.y = world_vy

        agent = TrackedAgent()
        agent.type = AgentType.HUMAN
        agent.name = f"human{identifier}"
        agent.state = (
            TrackedAgent.MOVING
            if math.hypot(world_vx, world_vy) > _MOVING_SPEED
            else TrackedAgent.STATIC
        )
        agent.segments.append(segment)
        return agent


def main() -> None:
    rclpy.init()
    node = TrackedAgentsBridge()
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
