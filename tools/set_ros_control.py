#!/usr/bin/env python3
"""Put a robot under ROS control, so ``/cmd_vel`` is what drives it.

    python set_ros_control.py [--robot robot_1] [--timeout 120] [--once]

A robot only reads ``/cmd_vel`` while its input controller is in its ROS mode,
and the mode can only be set once the robot exists. A run that starts the
simulator and a policy together hits that race - the first request arrives
before there is a robot to answer it - so this retries until the simulator
accepts, and says what it got each time.

It then *keeps watching*, and that is the half that matters most. Stopping a
session destroys its robots - ``GameManager.ClearAgents`` returns the crowd to
its pool and destroys the robots - and applying a scenario builds new ones from
the prefab, whose input controller is in its keyboard mode. The ROS mode was
therefore a property of one episode, and every method worked for the first one
and then drove nothing: the stack published ``/cmd_vel``, the robot ignored it,
and no window said why. The mode is set again on every ``/reset_done``, which
the application publishes each time it applies a scenario - a first load, a
different scenario, or a stop and a start of the same one.

Only a scenario application re-arms it. A robot whose operator switched it back
to the keyboard by hand is left alone: the hand wins, which is the rule the rest
of the project follows.

This is the one copy every method shares: it talks to the simulator's control
channel, which is part of RobotSNAP's contract, not to anything a method owns.
"""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

#: Every command, session and crowd alike, travels in one JSON body.
TOPIC_CONTROL = "/simulation/control"
#: The answer to one command.
TOPIC_CONTROL_RESULT = "/simulation/control_result"
#: The world-ready handshake, published once per applied scenario.
TOPIC_RESET_DONE = "/reset_done"
#: Seconds between two attempts.
RETRY_SECONDS = 3.0


class ControlModeSetter(Node):
    """Ask the simulator to read a robot's commands from ROS, episode after episode."""

    def __init__(self, robot: str):
        super().__init__("robotsnap_set_ros_control")
        self._robot = robot
        self._publisher = self.create_publisher(String, TOPIC_CONTROL, 10)
        self._answer = None
        #: How many scenarios have been applied since this node started. It is
        #: not a count of anything the application keeps: it is this node's own
        #: memory of "the robots I set the mode on are gone, there are new ones".
        self._episodes = 0
        self.create_subscription(String, TOPIC_CONTROL_RESULT, self._on_answer, 10)
        self.create_subscription(Bool, TOPIC_RESET_DONE, self._on_reset_done, 10)

    def _on_reset_done(self, message: Bool) -> None:
        if message.data:
            self._episodes += 1
            self.get_logger().info(
                f"scenario applied (episode {self._episodes}); the robots are new"
            )

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

    def watch(self, timeout: float, once: bool) -> int:
        """Hold the robot under ROS control for this episode and every later one.

        ``apply`` spins the node while it retries, so a scenario applied during
        a retry is seen. The episode the attempt was *for* is read before it is
        made, and not after: an attempt answered just as a new world is applied
        is an attempt for the world that just went away, and comparing against a
        count taken afterwards would skip the new one.
        """
        applied_for = -1
        while rclpy.ok():
            if applied_for != self._episodes:
                wanted = self._episodes
                if not self.apply(timeout):
                    print(
                        f"episode {wanted}: the simulator never accepted the ROS control mode",
                        flush=True,
                    )
                    continue
                applied_for = wanted
                print(
                    f"{self._robot} now takes its commands from /cmd_vel "
                    f"(episode {wanted})",
                    flush=True,
                )
                if once:
                    return 0
                continue
            rclpy.spin_once(self, timeout_sec=0.2)
        return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--robot", default="robot_1", help="robot to hand to ROS")
    parser.add_argument("--timeout", type=float, default=120.0, help="seconds of retries")
    parser.add_argument(
        "--once",
        action="store_true",
        help="set the mode for one episode and exit, instead of watching the session",
    )
    arguments = parser.parse_args(argv)

    rclpy.init()
    node = ControlModeSetter(arguments.robot)
    try:
        return node.watch(arguments.timeout, arguments.once)
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
