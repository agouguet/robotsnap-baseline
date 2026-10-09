#!/usr/bin/env python3
"""Run DRL-VO's policy on the observation and drive the robot with it.

This is ``drl_vo_inference.py`` together with ``cmd_vel_pub.py`` of the published
ROS 1 stack, as one ROS 2 node: the observation on ``/drl_vo/observation`` goes
through the same arithmetic (see :mod:`drl_vo_command`), the checkpoint picks an
action, and the action becomes a ``Twist`` on ``/cmd_vel`` - the topic
RobotSNAP's gateway writes back to the simulator.
"""

from __future__ import annotations

import argparse
import pathlib

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from drl_vo_command import GOAL_FLOATS, PEDESTRIAN_FLOATS, SCAN_FLOATS, command
from drl_vo_common import TOPIC_CMD_VEL, TOPIC_OBSERVATION
from drl_vo_network import DrlVoPolicy


class PolicyNode(Node):
    """Turn each observation from the builder into a velocity command."""

    def __init__(self, policy: DrlVoPolicy, command_topic: str):
        super().__init__("drl_vo_policy")
        self._policy = policy
        self._last = (0.0, 0.0)
        self._steps = 0

        self.create_subscription(Float64MultiArray, TOPIC_OBSERVATION, self._on_observation, 10)
        self._publisher = self.create_publisher(Twist, command_topic, 10)
        self.get_logger().info(f"policy ready; commands go to {command_topic}")

    def _on_observation(self, message: Float64MultiArray) -> None:
        data = np.asarray(message.data, dtype=np.float64)
        expected = PEDESTRIAN_FLOATS + SCAN_FLOATS + GOAL_FLOATS
        if data.size != expected:
            self.get_logger().warning(
                f"ignoring an observation of {data.size} floats; expected {expected}"
            )
            return

        forward, yaw = command(self._policy, data)
        self._publish(forward, yaw)

        self._steps += 1
        if self._steps % 100 == 0:
            goal = data[-GOAL_FLOATS:]
            self.get_logger().info(
                f"commands: {self._steps} | sub-goal ({goal[0]:+.2f}, {goal[1]:+.2f}) m | "
                f"forward {forward:+.3f} m/s | yaw {yaw:+.3f} rad/s"
            )

    def _publish(self, forward: float, yaw: float) -> None:
        if not (np.isfinite(forward) and np.isfinite(yaw)):
            self.get_logger().warning("the policy produced a non-finite command; holding the last one")
            forward, yaw = self._last
        self._last = (forward, yaw)

        message = Twist()
        message.linear.x = float(forward)
        message.angular.z = float(yaw)
        self._publisher.publish(message)


def main(argv=None) -> None:
    default_model = pathlib.Path(__file__).resolve().parent / "model" / "drl_vo.zip"
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--model", default=str(default_model), help="DRL-VO checkpoint")
    parser.add_argument(
        "--batchnorm",
        choices=("reference", "running"),
        default="reference",
        help=(
            "'reference' reproduces the published deployment, which normalised with the "
            "statistics of the sample it was given; 'running' uses the accumulated ones"
        ),
    )
    parser.add_argument("--cmd-vel-topic", default=TOPIC_CMD_VEL, help="where commands are published")
    arguments = parser.parse_args(argv)

    policy = DrlVoPolicy(arguments.model, batchnorm=arguments.batchnorm)
    rclpy.init()
    node = PolicyNode(policy, arguments.cmd_vel_topic)
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
