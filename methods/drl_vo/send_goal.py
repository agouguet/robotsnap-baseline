#!/usr/bin/env python3
"""Send a DRL-VO navigation goal to the running RobotSNAP session.

    python send_goal.py X Y [YAW_DEG] [--robot robot_1]

``X`` and ``Y`` are in the ROS frame the rest of the toolbar uses - the one
RViz2 and ``/simulation/state`` speak: x forward, y left, metres, on the map.
Unity is told the same point through ``set_robot_goal``, so the goal the
interface draws, the goal the metrics score against and the goal the policy
steers towards are one goal.

The conversion is the one ``robotsnap.client.set_robot_goal_ros`` performs: the
simulator's own axes are Unity's, where the ROS ``y`` is the negated simulator
``x`` and the ROS ``x`` is the simulator ``z``.
"""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#: Every command, session and crowd alike, travels in one JSON body.
TOPIC_CONTROL = "/simulation/control"
#: The answer to one command.
TOPIC_CONTROL_RESULT = "/simulation/control_result"


def to_simulator(x: float, y: float) -> tuple[float, float]:
    """``(x, y)`` of the ROS frame as the simulator's ``(x, z)``."""
    return -float(y), float(x)


class GoalSender(Node):
    def __init__(self):
        super().__init__("drl_vo_send_goal")
        self._publisher = self.create_publisher(String, TOPIC_CONTROL, 10)
        self._result = None
        self.create_subscription(String, TOPIC_CONTROL_RESULT, self._on_result, 10)

    def _on_result(self, message: String) -> None:
        self._result = message.data

    def send(self, x: float, y: float, robot: str, timeout: float) -> dict | None:
        simulator_x, simulator_z = to_simulator(x, y)
        body = {
            "command": "set_robot_goal",
            "x": simulator_x,
            "z": simulator_z,
            "robot": robot,
        }
        message = String()
        message.data = json.dumps(body)
        self._publisher.publish(message)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and self._result is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._result is None:
            return None
        try:
            return json.loads(self._result)
        except ValueError:
            return {"raw": self._result}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("x", type=float, help="goal x, in metres, map frame")
    parser.add_argument("y", type=float, help="goal y, in metres, map frame")
    parser.add_argument("yaw", type=float, nargs="?", default=0.0, help="unused; kept for symmetry")
    parser.add_argument("--robot", default="robot_1", help="robot the goal is for")
    parser.add_argument("--timeout", type=float, default=5.0, help="seconds to wait for the answer")
    arguments = parser.parse_args(argv)

    rclpy.init()
    node = GoalSender()
    try:
        answer = node.send(arguments.x, arguments.y, arguments.robot, arguments.timeout)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if answer is None:
        print(f"no answer on {TOPIC_CONTROL_RESULT}; is the bridge running with --ros2?")
        return 1
    if answer.get("ok") is False:
        print(f"refused: {answer.get('message') or answer}")
        return 1
    print(f"goal sent for {arguments.robot}: ({arguments.x:+.2f}, {arguments.y:+.2f}) m in the map frame")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
