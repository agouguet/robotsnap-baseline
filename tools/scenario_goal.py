#!/usr/bin/env python3
"""Deliver the goal of the running scenario to a method, in that method's own form.

    python scenario_goal.py --interface action [--robot robot_1]
    python scenario_goal.py --interface topic --topic /global_goal

A scenario carries the mission: it places the robot and it says where the robot
has to go. The application reads that goal itself, but a benchmark method does
not - it listens on an action or a topic, and it has no way to learn a goal
nobody sends it. Until this node existed the goal had to be typed into the
``goal`` window by hand for every method whose own interface is not
``/simulation/state``, while DRL-VO, which reads the session directly, simply
drove off. That is the difference this node removes.

What it does is the same whatever the method is: read ``/simulation/state``,
take the goal of the robot that was asked for - the entry of ``robots`` with
that id, or the primary robot's ``robot_goal`` - and hand that goal to the
method through the interface its manifest declares:

``--interface topic``   publish a ``PoseStamped`` on ``--topic``, the form the
                        crowd-navigation baselines read (``/global_goal``).
``--interface action``  send a ``navigate_to_pose`` goal, the form a nav2 stack
                        answers (nav2+DWB, CoHAN-Nav2).

It sends once per mission and then watches. A goal that *changes* - another
scenario, an edited goal - is sent again, and so is the same goal after an
episode boundary. ``/reset_done`` is the authoritative boundary; a regression
of ``sim_time_seconds`` remains a fallback for a session that did not publish
it. Anything else would either miss the second episode or drive the robot to
the same goal for ever.

The frames line up on their own: ``/simulation/state`` writes its goal in the
same ROS frame RViz and the goal tools speak (x forward, y left, metres, on the
map), so no conversion happens here and none should.

Nothing in this file blocks. The node is spun by ``rclpy.spin``, so a callback
that slept - waiting for the method to subscribe, for the action server to
appear, for the planner to answer - would take the whole executor with it and
freeze the very stream the waits are driven by. Every wait is therefore polled
from one timer, and the two things that must be *reported* (the outcome of a
nav2 goal) are read from done-callbacks instead of being waited on.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from nav2_episode_reset import Nav2EpisodeResetCoordinator

#: Whole-session snapshot, including every robot's goal, as JSON.
TOPIC_STATE = "/simulation/state"
#: Published by the application when applying a scenario has completed.
TOPIC_RESET_DONE = "/reset_done"
#: The name a nav2 stack answers on, whatever launch file brought it up.
ACTION_NAME = "navigate_to_pose"

#: The two interfaces a method can declare. A method that reads the session
#: itself - DRL-VO - declares neither and this node is not started for it.
INTERFACE_TOPIC = "topic"
INTERFACE_ACTION = "action"

#: How long one delivery attempt waits on a method that is still starting. A
#: nav2 stack takes tens of seconds to activate, and DDS discovery a few more.
_ACTION_SERVER_TIMEOUT = 180.0
_SUBSCRIBER_TIMEOUT = 60.0

#: The delivery loop's period, and how long the goal is repeated on a topic once
#: somebody is listening.
_TICK_SECONDS = 0.1
_PUBLISH_WINDOW = 1.0
#: A clock regression this soon after /reset_done describes the same boundary.
_RESET_DONE_GUARD_SECONDS = 2.0

#: The states of one delivery in flight.
_ARMED = "armed"
_WAITING_SERVER = "waiting for the action server"
_WAITING_SUBSCRIBER = "waiting for the method to subscribe"
_PUBLISHING = "publishing"
_WAITING_ACCEPT = "waiting for the planner to accept"
_RUNNING = "running"

_GOAL_STATUS = {
    GoalStatus.STATUS_UNKNOWN: "unknown",
    GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "executing",
    GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}


def number(value) -> float | None:
    """``float`` for a real number, ``None`` for anything else (bools included)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def robot_goal(body: dict, robot: str) -> tuple[float, float] | None:
    """``(x, y)`` of one robot's *destination*, in the ROS map frame, or ``None``.

    ``/simulation/state`` carries one entry per robot under ``robots``, plus the
    primary robot's goal at the top level. A named robot is looked up in the
    list first, so a fleet drives the one that was asked for - the same rule
    ``drl_vo_observation.py`` follows, so the two agree on what a goal is.

    The destination is ``target_pose``, not ``goal``, and the difference is a
    scenario that routes its robot through a waypoint. ``goal`` is the point the
    robot is walking towards *right now*, so on such a scenario it is the
    waypoint: sending that to a method asks it to drive to the middle of the
    route and stop, which is what the corner scenario did - the robot arrived at
    the turn, the mission never completed, and the episode was filed as stopped
    while every arrival test that read ``goal`` called it a success.
    ``target_pose`` is what the scenario authored as the destination, which is
    what a method is being asked to reach.
    """
    entries = body.get("robots")
    if isinstance(entries, list):
        for entry in entries:
            if not isinstance(entry, dict) or str(entry.get("id", "")) != robot:
                continue
            target = entry.get("target_pose")
            if isinstance(target, dict):
                point = _point(target)
                if point is not None:
                    return point
            if entry.get("has_goal") is not True:
                return None
            point = entry.get("goal")
            if isinstance(point, dict):
                return _point(point)
            return None

    target = body.get("robot_target_pose")
    if isinstance(target, dict):
        point = _point(target)
        if point is not None:
            return point
    if body.get("robot_has_goal") is not True:
        return None
    point = body.get("robot_goal")
    return _point(point) if isinstance(point, dict) else None


def _point(point: dict) -> tuple[float, float] | None:
    x, y = number(point.get("x")), number(point.get("y"))
    if x is None or y is None:
        return None
    return x, y


def pose_stamped(x: float, y: float, frame_id: str) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.pose.position.x = float(x)
    message.pose.position.y = float(y)
    message.pose.orientation.w = 1.0
    return message


def navigate_to_pose(x: float, y: float, frame_id: str) -> NavigateToPose.Goal:
    goal = NavigateToPose.Goal()
    goal.pose = pose_stamped(x, y, frame_id)
    return goal


class ScenarioGoal(Node):
    """Watch the session's goal and deliver it on the interface of one method."""

    def __init__(
        self,
        interface: str,
        robot: str,
        topic: str,
        frame_id: str,
        reset_guard_clock=None,
    ):
        super().__init__("robotsnap_scenario_goal")
        self._interface = interface
        self._robot = robot
        self._frame_id = frame_id
        self._reset_guard_clock = (
            time.monotonic if reset_guard_clock is None else reset_guard_clock
        )

        #: The last goal handed over, and the run of the clock it belonged to. A
        #: mission is a goal *and* an episode: the same goal after a reset is a
        #: new episode, and the same goal without one has already been sent.
        self._sent: tuple[float, float] | None = None
        self._sent_after = 0

        #: What the session says now, and how many times its clock went back.
        self._goal: tuple[float, float] | None = None
        self._last_sim_time: float | None = None
        self._resets = 0
        self._last_reset_done_at: float | None = None

        #: The delivery in flight: the goal it carries, the episode it belongs
        #: to, where it has got to and when it got there. One at a time, because
        #: a second goal sent under a first one is a mission nobody asked for.
        self._target: tuple[float, float] | None = None
        self._target_after = 0
        self._phase = ""
        self._phase_started = 0.0
        self._listening_since: float | None = None
        self._busy = False
        self._warned = ""
        self._attempt = 0
        self._goal_handle = None
        self._pending_send = None
        self._cancelled_handle = None
        self._reset_active = False

        self._publisher = None
        self._client = None
        self._reset_coordinator = None
        if interface == INTERFACE_TOPIC:
            self._publisher = self.create_publisher(PoseStamped, topic, 10)
        else:
            self._client = ActionClient(self, NavigateToPose, ACTION_NAME)
            self._reset_coordinator = Nav2EpisodeResetCoordinator(
                self,
                self._request_goal_cancel,
                self._goal_is_pending,
                self._episode_reset_finished,
            )

        self.create_subscription(String, TOPIC_STATE, self._on_state, 10)
        self.create_subscription(Bool, TOPIC_RESET_DONE, self._on_reset_done, 10)
        self.create_timer(_TICK_SECONDS, self._tick)
        self.get_logger().info(
            f"ready: interface={interface} robot={robot} "
            + (f"topic={topic}" if interface == INTERFACE_TOPIC else f"action={ACTION_NAME}")
        )

    # -- the session -------------------------------------------------------

    def _on_reset_done(self, message: Bool) -> None:
        """Use the application's reset signal as the authoritative boundary."""
        if not bool(message.data):
            return
        self._last_reset_done_at = self._reset_guard_clock()
        self._note_episode_boundary(TOPIC_RESET_DONE)

    def _on_state(self, message: String) -> None:
        try:
            body = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(body, dict):
            return

        self._goal = robot_goal(body, self._robot)

        # A clock that went backwards is a fallback boundary. /reset_done is
        # primary, so the same physical reset must not count twice when both
        # signals arrive close together.
        now = number(body.get("sim_time_seconds"))
        if now is not None:
            if self._last_sim_time is not None and now < self._last_sim_time - 1.0:
                guard_now = self._reset_guard_clock()
                guarded = (
                    self._last_reset_done_at is not None
                    and guard_now - self._last_reset_done_at
                    < _RESET_DONE_GUARD_SECONDS
                )
                if guarded:
                    self.get_logger().info(
                        "sim_time regression follows /reset_done; "
                        "same episode already recorded"
                    )
                else:
                    self._note_episode_boundary("sim_time regression")
            self._last_sim_time = now

    def _note_episode_boundary(self, source: str) -> None:
        """Record one boundary, then start Nav2 reset when that interface needs it."""
        self._resets += 1
        self.get_logger().info(f"episode boundary from {source} (reset #{self._resets})")
        self._begin_episode_reset()

    # -- one step of the delivery ------------------------------------------

    def _tick(self) -> None:
        if self._reset_active:
            self._reset_coordinator.step()
            return
        if self._target is None:
            self._arm()
        if self._target is None:
            return

        if self._phase == _ARMED:
            self._enter_first_wait()
        elif self._phase == _WAITING_SERVER:
            if self._client.server_is_ready():
                self._send_action()
            elif self._past(_ACTION_SERVER_TIMEOUT):
                self._give_up(f"no {ACTION_NAME} action server")
        elif self._phase == _WAITING_SUBSCRIBER:
            if self._publisher.get_subscription_count() > 0:
                self._listening_since = time.monotonic()
                self._phase = _PUBLISHING
            elif self._past(_SUBSCRIBER_TIMEOUT):
                self._give_up("nobody subscribed")
        elif self._phase == _PUBLISHING:
            self._publish()

    def _past(self, seconds: float) -> bool:
        return time.monotonic() - self._phase_started > seconds

    def _arm(self) -> None:
        """Take the session's goal as the next mission, when it is a new one."""
        if self._reset_active or self._busy or self._goal is None:
            return
        if self._goal == self._sent and self._resets == self._sent_after:
            return
        if self._target == self._goal and self._target_after == self._resets:
            return
        self._target = self._goal
        self._target_after = self._resets
        self._phase = _ARMED
        self._phase_started = time.monotonic()
        self._listening_since = None
        self._warned = ""
        self.get_logger().info(
            f"goal ({self._goal[0]:+.2f}, {self._goal[1]:+.2f}) to deliver"
        )

    def _enter_first_wait(self) -> None:
        self._phase_started = time.monotonic()
        if self._interface == INTERFACE_TOPIC:
            self._phase = _WAITING_SUBSCRIBER
        elif self._client.server_is_ready():
            self._send_action()
        else:
            self._phase = _WAITING_SERVER

    def _give_up(self, reason: str) -> None:
        """Say once why a mission is not going anywhere, and re-arm on a change.

        The goal is *kept*: the method may still be starting, and the mission is
        what the scenario asked for, not something to drop because a window was
        missed. ``_abandon`` only clears the attempt, so the next tick arms it
        again and the same wait starts over.
        """
        if self._warned != reason:
            self._warned = reason
            self.get_logger().warning(f"{reason}; keeping the goal and retrying")
        self._abandon()

    def _abandon(self) -> None:
        self._target = None
        self._phase = ""

    def _delivered(self) -> None:
        """Remember what was handed over, so the same mission is not sent twice."""
        goal = self._target
        self._sent = goal
        self._sent_after = self._target_after
        self.get_logger().info(f"goal ({goal[0]:+.2f}, {goal[1]:+.2f}) delivered")

    # -- the two interfaces ------------------------------------------------

    def _publish(self) -> None:
        """Write the goal until the method is listening, then briefly on.

        DDS discovery on a fresh participant takes longer than a single
        publish, and a goal sent before the method is subscribed is silently
        lost - the same reason ``send_goal.py`` repeats what it writes. The
        window counts from the moment somebody *was* listening, because a
        publish that arrived before the subscription existed proves nothing.
        """
        message = pose_stamped(self._target[0], self._target[1], self._frame_id)
        message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(message)
        if (
            self._listening_since is not None
            and time.monotonic() - self._listening_since >= _PUBLISH_WINDOW
        ):
            self._delivered()
            self._abandon()

    def _send_action(self) -> None:
        """Send one nav2 goal; its outcome is read from a done-callback."""
        self._busy = True
        self._phase = _WAITING_ACCEPT
        self._attempt += 1
        attempt = self._attempt
        self._pending_send = self._client.send_goal_async(
            navigate_to_pose(self._target[0], self._target[1], self._frame_id)
        )
        self._pending_send.add_done_callback(
            lambda future: self._on_accepted(future, attempt)
        )

    def _on_accepted(self, future, attempt: int) -> None:
        if self._pending_send is future:
            self._pending_send = None
        try:
            handle = future.result()
        except Exception as error:
            self.get_logger().warning(f"the goal request failed: {error}")
            if self._reset_active:
                return
            self._finish()
            return
        if handle is None or not handle.accepted:
            self.get_logger().warning("the goal was refused by the planner")
            if self._reset_active:
                return
            self._finish()
            return
        if attempt != self._attempt or self._reset_active:
            # The goal belongs to the episode that just ended. Cancel it now even
            # if the coordinator's bounded cancel phase already moved on.
            self._cancel_old_goal(handle)
            return

        self._goal_handle = handle
        self._delivered()
        self._phase = _RUNNING
        self.get_logger().info("goal accepted; waiting for the result")
        handle.get_result_async().add_done_callback(
            lambda result: self._on_result(result, attempt, handle)
        )

    def _on_result(self, future, attempt: int, handle) -> None:
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().warning(f"the goal result failed: {error}")
            result = None
        status = result.status if result is not None else None
        name = _GOAL_STATUS.get(status, str(status))
        self.get_logger().info(f"result: status={name}")
        if self._goal_handle is handle:
            self._goal_handle = None
        if attempt != self._attempt or self._reset_active:
            self.get_logger().info(f"previous episode goal is {name}")
            return
        self._finish()

    def _finish(self) -> None:
        """The mission is over: let the next scenario's goal be armed."""
        self._busy = False
        self._goal_handle = None
        self._abandon()

    # -- episode reset -----------------------------------------------------

    def _begin_episode_reset(self) -> None:
        if self._interface != INTERFACE_ACTION or self._reset_active:
            return
        self._reset_active = True
        # Acceptances from the episode that just ended are stale now. They still
        # get their handles captured and canceled from the done-callback.
        self._attempt += 1
        self._target = None
        self._phase = ""
        self._busy = False
        self._reset_coordinator.start()

    def _goal_is_pending(self) -> bool:
        return self._goal_handle is not None or self._pending_send is not None

    def _request_goal_cancel(self):
        handle = self._goal_handle
        if handle is None or handle is self._cancelled_handle:
            return None
        self._cancelled_handle = handle
        try:
            future = handle.cancel_goal_async()
        except Exception as error:
            self.get_logger().warning(f"could not cancel the active goal: {error}")
            self._goal_handle = None
            return None
        future.add_done_callback(self._on_cancel_response)
        return future

    def _on_cancel_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(f"the cancellation failed: {error}")
            return
        goals = getattr(response, "goals_canceling", None)
        if goals is not None and not goals:
            self.get_logger().warning(
                "the cancellation was acknowledged but no goal is canceling"
            )

    def _cancel_old_goal(self, handle) -> None:
        """Cancel a goal accepted after the reset had already started."""
        try:
            future = handle.cancel_goal_async()
        except Exception as error:
            self.get_logger().warning(f"could not cancel the previous goal: {error}")
            return
        future.add_done_callback(self._on_cancel_response)
        handle.get_result_async().add_done_callback(self._on_old_result)
        self.get_logger().info("previous episode goal accepted late and canceled")

    def _on_old_result(self, future) -> None:
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().warning(f"the previous goal result failed: {error}")
            return
        status = result.status if result is not None else None
        name = _GOAL_STATUS.get(status, str(status))
        self.get_logger().info(f"previous episode goal result: status={name}")

    def _episode_reset_finished(self) -> None:
        self._reset_active = False
        self._goal_handle = None
        self._busy = False
        self.get_logger().info("episode reset complete; the new goal may be armed")


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="scenario_goal.py",
        description="Hand the running scenario's goal to a method, in the form it reads.",
    )
    parser.add_argument(
        "--interface",
        required=True,
        choices=[INTERFACE_TOPIC, INTERFACE_ACTION],
        help="how the method wants to be told: a PoseStamped topic, or a nav2 action",
    )
    parser.add_argument("--robot", default="robot_1", help="whose goal (default: %(default)s)")
    parser.add_argument(
        "--topic",
        default="/global_goal",
        help="topic for --interface topic (default: %(default)s)",
    )
    parser.add_argument("--frame", default="map", help="frame written on the goal")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    arguments = parse_args(argv)
    rclpy.init()
    node = ScenarioGoal(
        arguments.interface,
        arguments.robot,
        arguments.topic,
        arguments.frame,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
