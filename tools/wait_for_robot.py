#!/usr/bin/env python3
"""Wait until RobotSNAP has a robot whose frames a navigation stack can use.

    python wait_for_robot.py [--robot robot_1] [--timeout 300]

A navigation stack that starts before the scenario does not fail loudly: its
servers stay ``inactive`` waiting for the transform ``<robot_base_frame> ->
odom``, the goal is refused, and the only clue is a line in a log. This gate
turns that silence into a short message and a guaranteed order - the stack is
launched once the robot is really there, and never before.

The check is the transform the stack itself waits on, ``odom -> base_link``,
resolved through ``/tf``. It is the whole condition: ``odom`` exists when
RobotSNAP has a robot, ``base_link`` when ``tools/frames_alias.py`` has run,
and the chain between them when both are true. Nothing is invented and nothing
is cached: the lookup is retried until it answers.

It exits 0 as soon as the transform resolves, and 1 after ``--timeout``.
"""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

#: The two ends of the transform a navigation stack cannot start without.
SOURCE_FRAME = "base_link"
TARGET_FRAME = "odom"
#: How often the transform is looked up again, and how often silence is broken.
POLL_SECONDS = 0.5
REPORT_SECONDS = 10.0


class RobotWaiter(Node):
    """Poll ``odom -> base_link`` until it resolves."""

    def __init__(self) -> None:
        super().__init__("robotsnap_wait_for_robot")
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

    def reachable(self) -> bool:
        """Whether the robot's frames are connected right now.

        ``Time()`` is the zero time, which tf2 reads as "the latest the buffer
        has": the question is not *when* the robot was there, only *whether* the
        tree holds it at all.
        """
        return self._buffer.can_transform(TARGET_FRAME, SOURCE_FRAME, rclpy.time.Time())

    def wait(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        next_report = time.monotonic()
        while time.monotonic() < deadline:
            if self.reachable():
                return True
            if time.monotonic() >= next_report:
                self.get_logger().info(
                    f"waiting for {TARGET_FRAME} -> {SOURCE_FRAME}: no robot in the "
                    "scenario yet (start it in the application, or run "
                    "tools/play_scenario.py)"
                )
                next_report = time.monotonic() + REPORT_SECONDS
            rclpy.spin_once(self, timeout_sec=POLL_SECONDS)
        return False


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--timeout", type=float, default=300.0, help="seconds to wait")
    # The robot name is accepted and ignored: RobotSNAP's primary robot answers to
    # the bare frames, and a second robot only changes the topic namespaces, not
    # the transform this gate is about. The flag is here so every tool of the
    # repository takes it.
    parser.add_argument("--robot", default="robot_1", help="unused; kept for symmetry")
    arguments = parser.parse_args(argv)

    rclpy.init()
    node = RobotWaiter()
    try:
        ready = node.wait(arguments.timeout)
    except KeyboardInterrupt:
        ready = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not ready:
        print(f"no {TARGET_FRAME} -> {SOURCE_FRAME} transform after {arguments.timeout:g}s")
        return 1
    print(f"robot present: {TARGET_FRAME} -> {SOURCE_FRAME} resolves")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
