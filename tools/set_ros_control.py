#!/usr/bin/env python3
"""Put a robot under ROS control, so ``/cmd_vel`` is what drives it.

    python set_ros_control.py [--robot robot_1] [--timeout 120]

A robot only reads ``/cmd_vel`` while its input controller is in its ROS mode,
and the mode can only be set once the robot exists. A run that starts the
simulator and a policy together hits that race - the first request arrives
before there is a robot to answer it - so this retries until the simulator
accepts, and says what it got each time. It exits 0 on acceptance, 1 after
``--timeout``.

This is the one copy every method shares: it talks to the simulator's control
channel, which is part of RobotSNAP's contract, not to anything a method owns.
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
#: Seconds between two attempts.
RETRY_SECONDS = 3.0


class ControlModeSetter(Node):
    """Ask the simulator to read a robot's commands from ROS, until it agrees."""

    def __init__(self, robot: str):
        super().__init__("robotsnap_set_ros_control")
        self._robot = robot
        self._publisher = self.create_publisher(String, TOPIC_CONTROL, 10)
        self._answer = None
        self.create_subscription(String, TOPIC_CONTROL_RESULT, self._on_answer, 10)

    def _on_answer(self, message: String) -> None:
        try:
            body = json.loads(message.data)
        except ValueError:
            return
        if isinstance(body, dict) and body.get("command") == "set_control_mode":
            self._answer = body

    def _request(self) -> None:
        message = String()
        message.data = json.dumps(
            {"command": "set_control_mode", "robot": self._robot, "mode": "ROS"}
        )
        self._publisher.publish(message)

    def apply(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._answer = None
            self._request()
            wait_until = time.monotonic() + RETRY_SECONDS
            while time.monotonic() < wait_until and self._answer is None:
                rclpy.spin_once(self, timeout_sec=0.1)

            answer = self._answer
            if answer is not None and answer.get("ok") is True:
                self.get_logger().info(answer.get("message", "control mode accepted"))
                return True
            if answer is not None:
                self.get_logger().info(f"refused for now: {answer.get('message')}")
            else:
                self.get_logger().info("no answer yet; the simulator may still be starting")
        return False


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--robot", default="robot_1", help="robot to hand to ROS")
    parser.add_argument("--timeout", type=float, default=120.0, help="seconds of retries")
    arguments = parser.parse_args(argv)

    rclpy.init()
    node = ControlModeSetter(arguments.robot)
    try:
        accepted = node.apply(arguments.timeout)
    except KeyboardInterrupt:
        accepted = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not accepted:
        print("the simulator never accepted the ROS control mode")
        return 1
    print(f"{arguments.robot} now takes its commands from /cmd_vel")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
