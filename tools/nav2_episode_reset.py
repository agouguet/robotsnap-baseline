#!/usr/bin/env python3
"""Coordinate the non-blocking Nav2 side of an episode reset.

The coordinator owns one short state machine:

    cancel -> clear local and global costmaps -> empty agents -> finish

It never sleeps and never waits on a future directly. ``step`` is meant to be
called from the scenario goal node's timer. A missing service is retried at
``retry_interval`` until the reset's global deadline, then skipped so that a
half-initialised Nav2 stack cannot hold the next episode forever.

CoHAN is optional. If ``cohan_msgs`` is importable this module also publishes an
empty ``TrackedAgents`` message; the bridge reset signal is always published,
which also lets the CoHAN bridge clear its remembered odometry.
"""

from __future__ import annotations

import time
from typing import Any, Callable

from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import Empty

try:
    from cohan_msgs.msg import TrackedAgents as _TrackedAgents
except ImportError:
    _TrackedAgents = None


RESET_SIGNAL_TOPIC = "/robotsnap/episode_reset"
TRACKED_AGENTS_TOPIC = "/tracked_agents"
LOCAL_COSTMAP_SERVICE = "/local_costmap/clear_entirely_local_costmap"
GLOBAL_COSTMAP_SERVICE = "/global_costmap/clear_entirely_global_costmap"

STATE_IDLE = "idle"
STATE_CANCEL = "cancel"
STATE_CLEARS = "clears"
STATE_EMPTY = "empty"
STATE_FINISH = "finish"


class Nav2EpisodeResetCoordinator:
    """Drive an episode reset from timer ticks.

    ``request_cancel`` returns a future when there is a goal handle to cancel.
    It returns ``None`` if cancellation is not possible yet or there is nothing
    to cancel; ``has_pending_goal`` says which of those two applies. This small
    protocol handles the race where reset arrives while ``send_goal_async`` has
    not supplied its goal handle yet.
    """

    def __init__(
        self,
        node,
        request_cancel: Callable[[], Any | None],
        has_pending_goal: Callable[[], bool],
        on_finished: Callable[[], None],
        *,
        clock: Callable[[], float] = time.monotonic,
        timeout: float = 4.0,
        cancel_timeout: float = 1.5,
        retry_interval: float = 0.25,
        tracked_agents_type=None,
    ) -> None:
        self._node = node
        self._request_cancel = request_cancel
        self._has_pending_goal = has_pending_goal
        self._on_finished = on_finished
        self._clock = clock
        self._timeout = max(0.0, float(timeout))
        self._cancel_timeout = max(0.0, float(cancel_timeout))
        self._retry_interval = max(0.0, float(retry_interval))

        self._clients = (
            node.create_client(ClearEntireCostmap, LOCAL_COSTMAP_SERVICE),
            node.create_client(ClearEntireCostmap, GLOBAL_COSTMAP_SERVICE),
        )
        self._signal_publisher = node.create_publisher(Empty, RESET_SIGNAL_TOPIC, 10)

        tracked_type = _TrackedAgents if tracked_agents_type is None else tracked_agents_type
        self._tracked_type = tracked_type
        self._tracked_publisher = (
            node.create_publisher(tracked_type, TRACKED_AGENTS_TOPIC, 10)
            if tracked_type is not None
            else None
        )

        self._state = STATE_IDLE
        self._deadline = 0.0
        self._cancel_deadline = 0.0
        self._cancel_future = None
        self._cancel_warned = False
        self._next_attempt = [0.0, 0.0]
        self._clear_futures: list[Any | None] = [None, None]
        self._clear_done = [False, False]
        self._clear_warned = [False, False]

    @property
    def state(self) -> str:
        return self._state

    @property
    def active(self) -> bool:
        return self._state not in (STATE_IDLE, STATE_FINISH)

    def start(self) -> None:
        """Begin one reset; subsequent calls while active are ignored."""
        if self.active:
            return
        now = self._clock()
        self._deadline = now + self._timeout
        self._cancel_deadline = min(self._deadline, now + self._cancel_timeout)
        self._cancel_future = None
        self._cancel_warned = False
        self._next_attempt = [now, now]
        self._clear_futures = [None, None]
        self._clear_done = [False, False]
        self._clear_warned = [False, False]
        self._state = STATE_CANCEL
        self._node.get_logger().info("episode reset: cancelling the active Nav2 goal")

    def step(self) -> None:
        """Advance the reset by one timer tick, without blocking."""
        if self._state == STATE_CANCEL:
            self._step_cancel(self._clock())
        if self._state == STATE_CLEARS:
            self._step_clears(self._clock())
        if self._state == STATE_EMPTY:
            self._enter_empty()

    # -- cancel ------------------------------------------------------------

    def _step_cancel(self, now: float) -> None:
        if self._cancel_future is None:
            try:
                future = self._request_cancel()
            except Exception as error:
                future = None
                if not self._cancel_warned:
                    self._cancel_warned = True
                    self._node.get_logger().warning(
                        f"episode reset: could not request cancellation: {error}"
                    )
            if future is not None:
                self._cancel_future = future
                future.add_done_callback(self._on_cancel_done)
                self._node.get_logger().info("episode reset: cancel request sent")
            elif self._has_pending_goal():
                if now >= self._cancel_deadline:
                    self._node.get_logger().warning(
                        "episode reset: pending goal did not expose a handle before "
                        "the cancel timeout; continuing with clears"
                    )
                    self._enter_clears(now)
                return
            else:
                self._enter_clears(now)
                return

        if self._cancel_future is not None and self._cancel_future.done():
            self._enter_clears(now)
        elif now >= self._cancel_deadline:
            self._node.get_logger().warning(
                "episode reset: cancellation did not answer before the timeout; "
                "continuing with clears"
            )
            self._enter_clears(now)

    def _on_cancel_done(self, future) -> None:
        try:
            future.result()
        except Exception as error:
            self._node.get_logger().warning(f"episode reset: cancellation failed: {error}")

    # -- costmaps ----------------------------------------------------------

    def _enter_clears(self, now: float) -> None:
        if self._state != STATE_CANCEL:
            return
        self._state = STATE_CLEARS
        self._next_attempt = [now, now]
        self._node.get_logger().info("episode reset: clearing local and global costmaps")

    def _step_clears(self, now: float) -> None:
        for index, client in enumerate(self._clients):
            if self._clear_done[index]:
                continue

            future = self._clear_futures[index]
            if future is not None:
                if future.done():
                    self._complete_clear(index, future)
                elif now >= self._deadline:
                    self._node.get_logger().warning(
                        f"episode reset: {self._service_name(index)} did not answer "
                        "before the reset deadline; continuing"
                    )
                    self._clear_done[index] = True
                continue

            if now >= self._deadline:
                self._node.get_logger().warning(
                    f"episode reset: {self._service_name(index)} stayed unavailable; "
                    "skipping it"
                )
                self._clear_done[index] = True
                continue
            if now < self._next_attempt[index]:
                continue
            self._next_attempt[index] = now + self._retry_interval

            try:
                ready = client.service_is_ready()
            except Exception as error:
                ready = False
                self._node.get_logger().warning(
                    f"episode reset: {self._service_name(index)} readiness check failed: "
                    f"{error}"
                )
            if not ready:
                if not self._clear_warned[index]:
                    self._clear_warned[index] = True
                    self._node.get_logger().warning(
                        f"episode reset: {self._service_name(index)} is unavailable; "
                        "retrying until the reset deadline"
                    )
                continue

            try:
                future = client.call_async(ClearEntireCostmap.Request())
            except Exception as error:
                self._node.get_logger().warning(
                    f"episode reset: {self._service_name(index)} call failed: {error}"
                )
                continue
            if future is None:
                self._node.get_logger().warning(
                    f"episode reset: {self._service_name(index)} returned no future"
                )
                continue
            self._clear_futures[index] = future
            future.add_done_callback(
                lambda done, position=index: self._complete_clear(position, done)
            )
            self._node.get_logger().info(
                f"episode reset: requested {self._service_name(index)}"
            )

        if all(self._clear_done):
            self._state = STATE_EMPTY

    def _complete_clear(self, index: int, future) -> None:
        if self._clear_done[index]:
            return
        try:
            future.result()
        except Exception as error:
            self._node.get_logger().warning(
                f"episode reset: {self._service_name(index)} failed: {error}"
            )
        self._clear_done[index] = True

    @staticmethod
    def _service_name(index: int) -> str:
        return (LOCAL_COSTMAP_SERVICE, GLOBAL_COSTMAP_SERVICE)[index]

    # -- agents and bridge -------------------------------------------------

    def _enter_empty(self) -> None:
        if self._state != STATE_EMPTY:
            return
        if self._tracked_publisher is not None:
            self._tracked_publisher.publish(self._empty_tracked_agents())
        self._signal_publisher.publish(Empty())
        self._node.get_logger().info(
            "episode reset: empty agents published and CoHAN bridge reset signalled"
        )
        self._state = STATE_FINISH
        self._on_finished()

    def _empty_tracked_agents(self):
        if self._tracked_type is not None:
            message = self._tracked_type()
            message.header.frame_id = "map"
            return message
        return None
