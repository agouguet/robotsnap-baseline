#!/usr/bin/env python3
"""Build the observation DRL-VO's policy reads, from RobotSNAP's own streams.

This node replaces two nodes of the published ROS 1 stack: ``cnn_data_pub.py``,
which assembled the ``CNN_data`` message, and ``pure_pursuit.py``, which turned
the global plan into the sub-goal the policy steers towards. It publishes the
same payload on ``/drl_vo/observation``:

    [ pedestrian map : 12800 ][ scan history : 7200 ][ sub-goal : 2 ]

The first 12800 floats are the 2 x 80 x 80 grid of 0.25 m cells
``drl_vo_agents.velocity_map`` paints, channel-major (vx then vy). The next 7200
are ten frames of 720 ranges on the -90..+90 degree grid
``drl_vo_lidar.resample_scan`` produces, oldest first. The last two are the
sub-goal in the robot frame, in metres.

Everything it needs comes from the running session: ``/scan``, ``/odom``,
``/simulation/agents`` (already in the robot frame) and ``/simulation/state``
for the robot's goal. The goal is Unity's, so the scenario's goal, the one the
interface sets and the one ``send_goal.py`` writes all drive the same thing.
"""

from __future__ import annotations

import argparse
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, String

import drl_vo_agents
import drl_vo_lidar
import drl_vo_subgoal
from drl_vo_common import (
    TOPIC_AGENTS,
    TOPIC_OBSERVATION,
    TOPIC_ODOM,
    TOPIC_SCAN,
    TOPIC_STATE,
    json_body,
    number,
    yaw_from_quaternion,
)

#: Frames of scan history the policy reads, the ``NUM_TP`` of the published code.
HISTORY = 10
#: Rate the history is filled at, in Hz.
RATE_HZ = 20.0


class ObservationNode(Node):
    """Publish one observation per tick, once the scan history is full."""

    def __init__(self, robot: str, lookahead: float):
        super().__init__("drl_vo_observation")
        self._robot = robot
        self._lookahead = lookahead

        self._history: deque = deque(maxlen=HISTORY)
        self._scan = None
        self._pose = None
        self._goal = None
        self._agents = np.zeros(2 * drl_vo_agents.ROWS * drl_vo_agents.COLS)
        self._published = 0
        #: The episode the history belongs to. The policy reads ten frames of the lidar, so a history
        #: that survives a scenario change feeds the new episode the last ten scans of the old world -
        #: a crowd that is not there any more, in a robot frame that has jumped. The episode is read
        #: from the session clock, which is put back to zero when a scenario is applied.
        self._episode: float | None = None

        self.create_subscription(LaserScan, TOPIC_SCAN, self._on_scan, 10)
        self.create_subscription(Odometry, TOPIC_ODOM, self._on_odom, 10)
        self.create_subscription(String, TOPIC_AGENTS, self._on_agents, 10)
        self.create_subscription(String, TOPIC_STATE, self._on_state, 10)
        self._publisher = self.create_publisher(Float64MultiArray, TOPIC_OBSERVATION, 10)
        self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info(
            "waiting for " + ", ".join((TOPIC_SCAN, TOPIC_ODOM, TOPIC_AGENTS, TOPIC_STATE))
        )

    # -- inputs ------------------------------------------------------------

    def _on_scan(self, message: LaserScan) -> None:
        self._scan = drl_vo_lidar.resample_scan(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            message.range_min,
            message.range_max,
        )

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self._pose = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))

    def _on_agents(self, message: String) -> None:
        body = json_body(message.data)
        if body is not None:
            self._agents = drl_vo_agents.velocity_map(body)

    def _on_state(self, message: String) -> None:
        body = json_body(message.data)
        if body is not None:
            self._follow_episode(body)
            self._goal = self._robot_goal(body)

    def _follow_episode(self, body: dict) -> None:
        """Drop the scan history when the session starts a new episode.

        A clock that went backwards is a scenario that was applied: every stream describes a different
        world from then on, and a history of the one before is worse than no history at all - the
        policy would drive on ten frames of a crowd that has been rebuilt somewhere else.
        """
        now = number(body.get("sim_time_seconds"))
        if now is None:
            return
        if self._episode is not None and now < self._episode - 0.5:
            self._history.clear()
            self._scan = None
            self._agents = np.zeros(2 * drl_vo_agents.ROWS * drl_vo_agents.COLS)
            self.get_logger().info("new episode: scan history cleared")
        self._episode = now

    def _robot_goal(self, body: dict):
        """The configured robot's goal in the world frame, or ``None``.

        ``/simulation/state`` carries one entry per robot under ``robots``, plus
        the primary robot's goal at the top level. A named robot is looked up in
        the list first, so a fleet drives the one that was asked for.
        """
        entries = body.get("robots")
        if isinstance(entries, list):
            for entry in entries:
                if isinstance(entry, dict) and str(entry.get("id", "")) == self._robot:
                    if entry.get("has_goal") is not True:
                        return None
                    point = entry.get("goal")
                    if isinstance(point, dict):
                        return number(point.get("x")), number(point.get("y"))
                    return None

        if body.get("robot_has_goal") is not True:
            return None
        point = body.get("robot_goal")
        if isinstance(point, dict):
            return number(point.get("x")), number(point.get("y"))
        return None

    # -- one tick ----------------------------------------------------------

    def _tick(self) -> None:
        if self._scan is None:
            return
        self._history.append(self._scan)
        if len(self._history) < HISTORY:
            return

        subgoal = (0.0, 0.0)
        if self._pose is not None and self._goal is not None:
            subgoal = drl_vo_subgoal.subgoal_in_robot_frame(
                self._pose[0], self._pose[1], self._pose[2],
                self._goal[0], self._goal[1], self._lookahead,
            )

        message = Float64MultiArray()
        message.data = np.concatenate(
            (
                self._agents,
                np.concatenate(self._history),
                np.asarray(subgoal, dtype=np.float64),
            )
        ).tolist()
        self._publisher.publish(message)

        self._published += 1
        if self._published % (int(RATE_HZ) * 5) == 0:
            goal = "none" if self._goal is None else f"({self._goal[0]:+.2f}, {self._goal[1]:+.2f})"
            self.get_logger().info(
                f"observations published: {self._published} | goal {goal} | "
                f"sub-goal ({subgoal[0]:+.2f}, {subgoal[1]:+.2f}) m in robot frame"
            )


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--robot", default="robot_1", help="robot whose goal drives the policy")
    parser.add_argument(
        "--lookahead",
        type=float,
        default=drl_vo_subgoal.LOOKAHEAD_M,
        help="metres ahead of the robot the sub-goal sits",
    )
    arguments = parser.parse_args(argv)

    rclpy.init()
    node = ObservationNode(arguments.robot, arguments.lookahead)
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
