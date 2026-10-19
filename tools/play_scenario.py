#!/usr/bin/env python3
"""Start the scenario RobotSNAP has loaded, so a method has a robot to drive.

    python play_scenario.py [--timeout 60] [--no-reset]

The player opens on the scenario it was started with, in standby: the world,
the map and the crowd exist, but nothing moves and there is no robot to drive.
The green button in the application starts it, and this is the same thing over
ROS - the ``reset`` and ``play`` commands of the simulator's control channel.

``reset`` comes first because a player that has just been launched has not
applied the scenario yet, and ``play`` on a session whose scenario was never
applied does nothing. The reset reapplies the scenario and the play starts it,
so a launch always begins a fresh episode; ``--no-reset`` keeps the session as
it is and only asks for play.

It exits 0 once the simulator answers that the simulation is running, and 1
after ``--timeout``. Every answer is printed, so a refusal says why.
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
RETRY_SECONDS = 5.0


class ScenarioStarter(Node):
    """Ask the simulator to reset and play, until it says the simulation runs."""

    def __init__(self, reset: bool):
        super().__init__("robotsnap_play_scenario")
        self._reset = reset
        self._publisher = self.create_publisher(String, TOPIC_CONTROL, 10)
        self._answer = None
        self.create_subscription(String, TOPIC_CONTROL_RESULT, self._on_answer, 10)

    def _on_answer(self, message: String) -> None:
        try:
            body = json.loads(message.data)
        except ValueError:
            return
        if isinstance(body, dict) and body.get("command") in {"reset", "play"}:
            self._answer = body

    def _request(self, command: str) -> None:
        message = String()
        message.data = json.dumps({"command": command})
        self._publisher.publish(message)

    def _ask(self, command: str, seconds: float) -> dict | None:
        """Send one command and wait for its own answer, or ``None`` on silence."""
        self._answer = None
        self._request(command)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and self._answer is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._answer

    def start(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._reset:
                answer = self._ask("reset", RETRY_SECONDS)
                if answer is None:
                    self.get_logger().info("no answer to reset; the player may still be starting")
                else:
                    self.get_logger().info(f"reset: {answer.get('message', answer)}")

            answer = self._ask("play", RETRY_SECONDS)
            if answer is not None and answer.get("ok") is True:
                self.get_logger().info(answer.get("message", "simulation running"))
                return True
            if answer is not None:
                self.get_logger().info(f"play refused for now: {answer.get('message')}")
            else:
                self.get_logger().info("no answer to play yet; the player may still be starting")
            time.sleep(1.0)
        return False


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--timeout", type=float, default=120.0, help="seconds of retries")
    parser.add_argument(
        "--no-reset",
        action="store_true",
        help="only ask for play, without reapplying the scenario first",
    )
    arguments = parser.parse_args(argv)

    rclpy.init()
    node = ScenarioStarter(reset=not arguments.no_reset)
    try:
        started = node.start(arguments.timeout)
    except KeyboardInterrupt:
        started = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not started:
        print("the simulator never said the simulation was running")
        return 1
    print("simulation running")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
