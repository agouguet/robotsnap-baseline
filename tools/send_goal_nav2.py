#!/usr/bin/env python3
"""Send one ``NavigateToPose`` goal to CoHAN-Nav2 and wait for the result.

Usage::

    python send_goal.py X Y [YAW_DEG]

CoHAN-Nav2 is a nav2 stack, so a goal is an *action*, not a message: the
planner it replaces lived behind ``move_base`` and read ``/global_goal``, while
this one answers the ``navigate_to_pose`` action offered by the ``bt_navigator``.
The message that carries the goal is still a ``PoseStamped`` in ``map``.

The node waits for the action server (the stack takes a while to activate),
sends the goal, then waits for the result so the caller sees the outcome instead
of a fire-and-forget publish.
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

#: Seconds to wait for the stack to offer the action before giving up.
SERVER_TIMEOUT = 60.0


def build_goal(x: float, y: float, yaw_deg: float, frame_id: str) -> NavigateToPose.Goal:
    """A ``NavigateToPose`` goal at ``(x, y)`` with a yaw about z."""
    half = math.radians(yaw_deg) * 0.5
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = frame_id
    goal.pose.pose.position.x = float(x)
    goal.pose.pose.position.y = float(y)
    goal.pose.pose.orientation.z = math.sin(half)
    goal.pose.pose.orientation.w = math.cos(half)
    return goal


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="send_goal.py",
        description="Send one NavigateToPose goal and report the outcome.",
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
        default=SERVER_TIMEOUT,
        help="seconds to wait for the action server (default: %(default)s)",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    rclpy.init()
    node = Node("send_goal")
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    try:
        if not client.wait_for_server(timeout_sec=max(0.0, args.wait)):
            print(
                f"no navigate_to_pose action server after {args.wait:g}s: "
                "is the CoHAN stack up?",
                file=sys.stderr,
            )
            return 1

        goal = build_goal(args.x, args.y, args.yaw_deg, args.frame)
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        print(f"goal sent: frame={args.frame} x={args.x:g} y={args.y:g} yaw={args.yaw_deg:g}deg")

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print("the goal was refused by the planner", file=sys.stderr)
            return 1

        print("goal accepted; waiting for the result")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result()
        status = result.status if result is not None else None
        print(f"result: status={status}")
        return 0 if status == 4 else 1
    except KeyboardInterrupt:
        return 1
    finally:
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
