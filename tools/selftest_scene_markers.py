#!/usr/bin/env python3
"""Check the RViz marker node without a simulation, an RViz or a Unity build.

    source /opt/ros/humble/setup.bash && python selftest_scene_markers.py

The node turns two JSON streams into markers, and the one thing it must never do
is move the crowd on a message that carried no new position for it. That is what
this drives: synthetic odometry, agent and state messages, fed through the node's
own handlers, with the crowd read back after every one. The first case is the
regression itself - a robot that turns while the crowd stands still.
"""

from __future__ import annotations

import importlib.util
import json
import math
import pathlib

import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker

HERE = pathlib.Path(__file__).resolve().parent


def load_scene_markers():
    """The tool under test, loaded from its file so no package install is needed."""
    spec = importlib.util.spec_from_file_location(
        "scene_markers", HERE / "scene_markers.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def odometry(x: float, y: float, yaw: float, stamp: float = 1.0) -> Odometry:
    """An ``/odom`` message: a planar pose, which is all the node reads."""
    message = Odometry()
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def agents(*entries) -> String:
    """One ``/simulation/agents`` snapshot, in the robot frame."""
    return String(data=json.dumps({"agents": list(entries), "frame": "robot"}))


def state(humans=(), **extra) -> String:
    """One ``/simulation/state`` snapshot, world frame, with whatever else the case needs."""
    body = {"humans": list(humans), "robot_has_goal": False}
    body.update(extra)
    return String(data=json.dumps(body))


def crowd_of(node) -> list[tuple[int, float, float]]:
    """The crowd as it will be drawn, rounded so the comparison is not noise."""
    return [
        (identifier, round(x, 9), round(y, 9))
        for identifier, x, y in node._crowd
    ]


class Reporter:
    """`ok`/`fail` lines, and a count of the failures, in the tools' own style."""

    def __init__(self) -> None:
        self.failures = 0

    def check(self, condition: bool, label: str, detail: str = "") -> None:
        if condition:
            print(f"ok   {label}")
            return
        self.failures += 1
        print(f"FAIL {label}{f' - {detail}' if detail else ''}")


def turning_does_not_move_the_crowd(node, reporter: Reporter) -> None:
    """The regression: odometry ticks between two snapshots used to rotate the crowd."""
    node._on_odom(odometry(0.0, 0.0, 0.0))
    node._on_agents(agents({"id": "0", "x": 1.0, "y": 0.0}))
    reporter.check(
        crowd_of(node) == [(0, 1.0, 0.0)],
        "an agent one metre ahead is drawn one metre ahead",
        f"got {crowd_of(node)}",
    )

    for step in range(1, 6):
        node._on_odom(odometry(0.0, 0.0, step * 0.2))
    reporter.check(
        crowd_of(node) == [(0, 1.0, 0.0)],
        "five odometry ticks, one whole radian of turn, and the crowd has not moved",
        f"got {crowd_of(node)}",
    )

    # The robot marker, unlike the crowd, must follow odometry: it is the pose.
    markers = node._agent_markers(node._odom.header.stamp)
    reporter.check(
        len(markers) == 1 and abs(markers[0].pose.position.x - 1.0) < 1e-9,
        "the frozen crowd is written out unchanged under a fresh stamp",
    )


def a_new_snapshot_is_placed_with_its_own_pose(node, reporter: Reporter) -> None:
    """A snapshot does land the crowd: frozen is not the same as deaf."""
    # The previous case left the robot at 1.0 rad.
    expected = (round(math.cos(1.0), 9), round(math.sin(1.0), 9))
    node._on_agents(agents({"id": "0", "x": 1.0, "y": 0.0}))
    got = crowd_of(node)[0][1:]
    reporter.check(
        got == expected,
        "a new snapshot is placed with the pose of the instant it was measured",
        f"expected {expected}, got {got}",
    )


def the_world_frame_humans_win(node, reporter: Reporter) -> None:
    """`/simulation/state` carries the crowd already in `map`: use it as it is."""
    node._on_state(state([{"id": "3", "x": 5.0, "y": -1.0}]))
    reporter.check(
        crowd_of(node) == [(3, 5.0, -1.0)],
        "the humans of the state snapshot are drawn where the scenario put them",
        f"got {crowd_of(node)}",
    )
    node._on_odom(odometry(9.0, 9.0, 3.0))
    reporter.check(
        crowd_of(node) == [(3, 5.0, -1.0)],
        "an odometry tick far away does not drag the world-frame crowd with it",
        f"got {crowd_of(node)}",
    )


def an_empty_state_falls_back_to_the_agents(node, reporter: Reporter) -> None:
    """A scenario that names no human still shows whatever the detector tracks."""
    node._on_state(state([]))
    x, y = math.cos(3.0), math.sin(3.0)
    expected = [(0, round(9.0 + x, 9), round(9.0 + y, 9))]
    reporter.check(
        crowd_of(node) == expected,
        "an empty human list falls back to the robot-frame agents",
        f"expected {expected}, got {crowd_of(node)}",
    )


def colours_follow_the_agents_not_the_order(node, reporter: Reporter) -> None:
    """A crowd that reorders must not look like the people were swapped."""
    one = {"id": "1", "x": 2.0, "y": 0.0}
    zero = {"id": "0", "x": 1.0, "y": 0.0}
    stamp = node._odom.header.stamp
    node._on_state(state([one, zero]))
    first = {m.id: (m.color.r, m.color.g, m.color.b) for m in node._agent_markers(stamp)}
    node._on_state(state([zero, one]))
    second = {m.id: (m.color.r, m.color.g, m.color.b) for m in node._agent_markers(stamp)}
    reporter.check(
        first == second and set(first) == {0, 1},
        "each agent keeps its own marker id and colour when the list reorders",
        f"{first} then {second}",
    )


def the_array_is_one_clear_scene(node, reporter: Reporter) -> None:
    """RViz needs the DELETEALL first, and nobody else may write the crowd's namespace."""
    node._on_odom(odometry(0.0, 0.0, 0.0))
    published: list = []
    node._publisher.publish = published.append
    node._republish()
    markers = published[0].markers
    reporter.check(
        bool(markers) and markers[0].action == Marker.DELETEALL,
        "the array opens with the DELETEALL that stops ghosts",
        f"first action {markers[0].action if markers else None}",
    )
    reporter.check(
        [m.ns for m in markers] == ["", "robot", "agents", "agents"],
        "one robot marker and the crowd, nothing else",
        f"got {[m.ns for m in markers]}",
    )


def main() -> int:
    scene_markers = load_scene_markers()
    rclpy.init()
    reporter = Reporter()
    node = scene_markers.SceneMarkers("robot_1")
    try:
        turning_does_not_move_the_crowd(node, reporter)
        a_new_snapshot_is_placed_with_its_own_pose(node, reporter)
        the_world_frame_humans_win(node, reporter)
        an_empty_state_falls_back_to_the_agents(node, reporter)
        colours_follow_the_agents_not_the_order(node, reporter)
        the_array_is_one_clear_scene(node, reporter)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if reporter.failures:
        print(f"\n{reporter.failures} check(s) failed")
        return 1
    print("\nall checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
