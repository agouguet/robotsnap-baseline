#!/usr/bin/env python3
"""Publish one navigation goal on a topic and exit.

Usage::

    python send_goal.py [--topic /global_goal] X Y [YAW_DEG]

``X`` and ``Y`` are metres in the message frame (``map`` by default), ``YAW_DEG``
is degrees about z and defaults to 0. ``/global_goal`` is the topic the
crowd-navigation baselines of the benchmark read; pass ``--topic`` for another
name.

The goal waits for the method to appear as a subscriber before publishing, then
repeats it briefly, because DDS discovery on a fresh participant takes longer
than a single publish - and a goal sent before the method is listening is
silently lost.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped

_PUBLISH_PERIOD = 0.1
#: Seconds to wait for BRNE to show up as a subscriber before giving up.
DEFAULT_SUBSCRIBER_TIMEOUT = 20.0
#: How long to keep repeating the goal once a subscriber is there.
_PUBLISH_WINDOW = 0.5


def build_goal(x: float, y: float, yaw_deg: float, frame_id: str) -> PoseStamped:
    """A planar ``PoseStamped`` at ``(x, y)`` with a yaw about z."""
    half = math.radians(yaw_deg) * 0.5
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.orientation.z = math.sin(half)
    goal.pose.orientation.w = math.cos(half)
    return goal


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="send_goal.py",
        description="Publish one goal on a topic and exit.",
    )
    parser.add_argument(
        "--topic", default="/global_goal", help="topic to publish on (default: %(default)s)"
    )
    parser.add_argument("x", type=float, help="goal x in the message frame (metres)")
    parser.add_argument("y", type=float, help="goal y in the message frame (metres)")
    parser.add_argument(
        "yaw_deg", type=float, nargs="?", default=0.0, help="heading about z (degrees)"
    )
    parser.add_argument("--frame", default="map", help="header.frame_id (default: %(default)s)")
    parser.add_argument(
        "--wait",
        type=float,
        default=DEFAULT_SUBSCRIBER_TIMEOUT,
        help="seconds to wait for a subscriber (default: %(default)s)",
    )
    parser.add_argument(
        "--publish-window",
        type=float,
        default=_PUBLISH_WINDOW,
        help="seconds to keep repeating once a subscriber is there (default: %(default)s)",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    goal = build_goal(args.x, args.y, args.yaw_deg, args.frame)

    rclpy.init()
    node = rclpy.create_node("send_goal")
    publisher = node.create_publisher(PoseStamped, args.topic, 10)
    try:
        deadline = time.monotonic() + max(0.0, args.wait)
        while publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=_PUBLISH_PERIOD)
            if time.monotonic() >= deadline:
                print(
                    f"no subscriber on {args.topic} after {args.wait:g}s: is the method running?",
                    file=sys.stderr,
                )
                return 1

        deadline = time.monotonic() + max(0.0, args.publish_window)
        while True:
            goal.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(goal)
            rclpy.spin_once(node, timeout_sec=_PUBLISH_PERIOD)
            if time.monotonic() >= deadline:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(
        f"sent {args.topic} frame={args.frame} "
        f"x={args.x:g} y={args.y:g} yaw={args.yaw_deg:g}deg"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
