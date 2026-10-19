#!/usr/bin/env python3
"""Check the Nav2 episode-reset state machine without ROS or a simulation.

    source /opt/ros/humble/setup.bash && python selftest_nav2_episode_reset.py

The coordinator is driven with fake clients, futures, publishers and a fake
clock. The first case records the required order; the following cases make
services or cancellation disappear and verify that the global deadline always
releases the next goal.
"""

from __future__ import annotations

import importlib.util
import json
import pathlib
import sys
from types import MethodType

from std_msgs.msg import String

HERE = pathlib.Path(__file__).resolve().parent


def load_coordinator_module():
    """Load the tool under test from its file, with no package install."""
    spec = importlib.util.spec_from_file_location(
        "nav2_episode_reset", HERE / "nav2_episode_reset.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_scenario_goal_module():
    """Load the goal node with its sibling coordinator import available."""
    sys.path.insert(0, str(HERE))
    try:
        spec = importlib.util.spec_from_file_location(
            "scenario_goal", HERE / "scenario_goal.py"
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(HERE))


class Reporter:
    """`ok`/`fail` lines and a failure count, matching the other selftests."""

    def __init__(self) -> None:
        self.failures = 0

    def check(self, condition: bool, label: str, detail: str = "") -> None:
        if condition:
            print(f"ok   {label}")
            return
        self.failures += 1
        print(f"FAIL {label}{f' - {detail}' if detail else ''}")


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeFuture:
    def __init__(self, done: bool = False) -> None:
        self._done = done
        self._callbacks = []

    def done(self) -> bool:
        return self._done

    def result(self):
        return None

    def add_done_callback(self, callback) -> None:
        self._callbacks.append(callback)
        if self._done:
            callback(self)

    def complete(self) -> None:
        if self._done:
            return
        self._done = True
        for callback in self._callbacks:
            callback(self)


class FakeLogger:
    def __init__(self, events: list[str]) -> None:
        self.events = events

    def info(self, message: str) -> None:
        self.events.append(f"log:{message}")

    def warning(self, message: str) -> None:
        self.events.append(f"warn:{message}")


class FakePublisher:
    def __init__(self, name: str, events: list[str]) -> None:
        self.name = name
        self.events = events
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)
        if self.name == "/tracked_agents":
            self.events.append("agents")
        elif self.name == "/robotsnap/episode_reset":
            self.events.append("bridge")


class FakeClient:
    def __init__(self, name: str, ready: bool, events: list[str]) -> None:
        self.name = name
        self.ready = ready
        self.events = events
        self.readiness_checks = 0
        self.calls = 0

    def service_is_ready(self) -> bool:
        self.readiness_checks += 1
        return self.ready

    def call_async(self, request):
        self.calls += 1
        self.events.append(f"clear:{self.name}")
        return FakeFuture(done=True)


class FakeNode:
    def __init__(self, ready: dict[str, bool], events: list[str]) -> None:
        self.events = events
        self.clients = {}
        self.publishers = {}
        self._logger = FakeLogger(events)
        self._ready = ready

    def get_logger(self):
        return self._logger

    def create_client(self, service_type, name):
        client = FakeClient(name, self._ready.get(name, True), self.events)
        self.clients[name] = client
        return client

    def create_publisher(self, message_type, name, queue_size):
        publisher = FakePublisher(name, self.events)
        self.publishers[name] = publisher
        return publisher


class FakeTrackedAgents:
    def __init__(self) -> None:
        self.header = type("Header", (), {"frame_id": ""})()
        self.agents = []


class FakeBool:
    def __init__(self, data: bool) -> None:
        self.data = data


class FakeResetCoordinator:
    def __init__(self, events: list[str]) -> None:
        self.events = events
        self.starts = 0

    def start(self) -> None:
        self.starts += 1
        self.events.append("nav2-reset")


def state_message(**body) -> String:
    return type("StateMessage", (), {"data": json.dumps(body)})()


def make_scenario_stub(scenario_goal, interface: str, clock, events: list[str]):
    scenario = type("Scenario", (), {})()
    scenario._interface = interface
    scenario._robot = "robot_1"
    scenario._reset_guard_clock = clock
    scenario._resets = 0
    scenario._last_reset_done_at = None
    scenario._last_sim_time = None
    scenario._goal = None
    scenario._reset_active = False
    scenario._target = None
    scenario._phase = ""
    scenario._busy = False
    scenario._attempt = 0
    scenario._reset_coordinator = (
        FakeResetCoordinator(events)
        if interface == scenario_goal.INTERFACE_ACTION
        else None
    )
    scenario._logger = FakeLogger(events)
    scenario.get_logger = lambda: scenario._logger
    for name in (
        "_on_reset_done",
        "_on_state",
        "_note_episode_boundary",
        "_begin_episode_reset",
    ):
        setattr(
            scenario,
            name,
            MethodType(getattr(scenario_goal.ScenarioGoal, name), scenario),
        )
    return scenario


class FakeGoalHandle:
    accepted = True

    def __init__(self, events: list[str]) -> None:
        self.events = events

    def cancel_goal_async(self):
        self.events.append("cancel-late")
        return FakeFuture(done=True)

    def get_result_async(self):
        return FakeFuture()


def make_coordinator(module, node, clock, request_cancel, pending, events, **kwargs):
    return module.Nav2EpisodeResetCoordinator(
        node,
        request_cancel,
        pending,
        lambda: events.append("goal"),
        clock=clock,
        tracked_agents_type=FakeTrackedAgents,
        **kwargs,
    )


def the_sequence_is_cancel_then_clears_then_empty_then_goal(module, reporter) -> None:
    clock = FakeClock()
    events: list[str] = []
    node = FakeNode({}, events)
    cancel_future = FakeFuture()

    def request_cancel():
        events.append("cancel")
        return cancel_future

    coordinator = make_coordinator(
        module,
        node,
        clock,
        request_cancel,
        lambda: True,
        events,
        timeout=2.0,
        cancel_timeout=0.5,
        retry_interval=0.1,
    )
    coordinator.start()
    coordinator.step()
    visible = [event for event in events if not event.startswith(("log:", "warn:"))]
    reporter.check(
        visible == ["cancel"],
        "the reset waits for the cancel response before clearing",
        f"got {visible}",
    )
    cancel_future.complete()
    coordinator.step()

    sequence = [event for event in events if not event.startswith(("log:", "warn:"))]
    reporter.check(
        sequence
        == [
            "cancel",
            "clear:/local_costmap/clear_entirely_local_costmap",
            "clear:/global_costmap/clear_entirely_global_costmap",
            "agents",
            "bridge",
            "goal",
        ],
        "cancel, local clear, global clear, empty agents, bridge reset, then goal",
        f"got {sequence}",
    )
    reporter.check(
        coordinator.state == module.STATE_FINISH,
        "the coordinator finishes once the empty agents were published",
        f"state={coordinator.state}",
    )
    tracked = node.publishers[module.TRACKED_AGENTS_TOPIC].messages
    reporter.check(
        len(tracked) == 1
        and tracked[0].agents == []
        and tracked[0].header.frame_id == "map",
        "the direct /tracked_agents message is empty and in the map frame",
        f"got {tracked}",
    )


def absent_services_are_retried_then_skipped(module, reporter) -> None:
    clock = FakeClock()
    events: list[str] = []
    node = FakeNode(
        {
            module.LOCAL_COSTMAP_SERVICE: False,
            module.GLOBAL_COSTMAP_SERVICE: False,
        },
        events,
    )
    coordinator = make_coordinator(
        module,
        node,
        clock,
        lambda: None,
        lambda: False,
        events,
        timeout=0.55,
        cancel_timeout=0.1,
        retry_interval=0.1,
    )
    coordinator.start()
    coordinator.step()
    for _ in range(7):
        clock.advance(0.1)
        coordinator.step()

    local = node.clients[module.LOCAL_COSTMAP_SERVICE]
    global_ = node.clients[module.GLOBAL_COSTMAP_SERVICE]
    reporter.check(
        local.readiness_checks > 1 and global_.readiness_checks > 1,
        "each absent service is retried by the timer",
        f"checks local={local.readiness_checks}, global={global_.readiness_checks}",
    )
    reporter.check(
        local.calls == 0 and global_.calls == 0,
        "absent services are never called through a blocking path",
        f"calls local={local.calls}, global={global_.calls}",
    )
    visible = [event for event in events if not event.startswith(("log:", "warn:"))]
    reporter.check(
        visible == ["agents", "bridge", "goal"],
        "the reset continues through empty and finish after the deadline",
        f"got {visible}",
    )
    reporter.check(
        coordinator.state == module.STATE_FINISH,
        "the absent services do not leave the coordinator active",
        f"state={coordinator.state}",
    )


def a_cancel_future_that_never_answers_is_bounded(module, reporter) -> None:
    clock = FakeClock()
    events: list[str] = []
    node = FakeNode({}, events)
    coordinator = make_coordinator(
        module,
        node,
        clock,
        lambda: FakeFuture(),
        lambda: True,
        events,
        timeout=0.8,
        cancel_timeout=0.25,
        retry_interval=0.1,
    )
    coordinator.start()
    coordinator.step()
    for _ in range(4):
        clock.advance(0.1)
        coordinator.step()

    visible = [event for event in events if not event.startswith(("log:", "warn:"))]
    reporter.check(
        coordinator.state == module.STATE_FINISH,
        "an unanswered cancellation is bounded and the sequence still finishes",
        f"state={coordinator.state}",
    )
    reporter.check(
        visible[-3:] == ["agents", "bridge", "goal"],
        "the empty and bridge reset still happen after a cancel timeout",
        f"got {visible[-3:]}",
    )


def a_late_goal_acceptance_is_canceled(scenario_goal, reporter) -> None:
    """A stale send may resolve after the reset's bounded cancel phase."""
    events: list[str] = []
    handle = FakeGoalHandle(events)
    send_future = FakeFuture(done=True)
    send_future.result = lambda: handle
    scenario = type("Scenario", (), {})()
    scenario._pending_send = send_future
    scenario._attempt = 2
    scenario._reset_active = True
    scenario._cancelled_handle = None
    scenario._logger = FakeLogger(events)
    scenario.get_logger = lambda: scenario._logger
    for name in ("_on_accepted", "_cancel_old_goal", "_on_cancel_response", "_on_old_result"):
        setattr(
            scenario,
            name,
            MethodType(getattr(scenario_goal.ScenarioGoal, name), scenario),
        )

    scenario._on_accepted(send_future, 1)
    visible = [event for event in events if not event.startswith("log:")]
    reporter.check(
        visible == ["cancel-late"],
        "an old goal accepted during reset is canceled immediately",
        f"got {visible}",
    )


def topic_interface_does_not_enter_nav2_reset(scenario_goal, reporter) -> None:
    scenario = type("Scenario", (), {})()
    scenario._interface = scenario_goal.INTERFACE_TOPIC
    scenario._reset_active = False
    scenario._reset_coordinator = None

    scenario_goal.ScenarioGoal._begin_episode_reset(scenario)
    reporter.check(
        scenario._reset_active is False,
        "the topic interface keeps its existing reset path",
    )


def reset_done_is_primary_and_clock_is_only_fallback(scenario_goal, reporter) -> None:
    clock = FakeClock()
    events: list[str] = []
    scenario = make_scenario_stub(
        scenario_goal, scenario_goal.INTERFACE_ACTION, clock, events
    )

    scenario._on_reset_done(FakeBool(True))
    reporter.check(
        scenario._resets == 1 and scenario._reset_coordinator.starts == 1,
        "/reset_done True records one episode and starts Nav2 reset once",
        f"resets={scenario._resets}, starts={scenario._reset_coordinator.starts}",
    )

    scenario._last_sim_time = 50.0
    scenario._on_state(state_message(sim_time_seconds=48.0))
    reporter.check(
        scenario._resets == 1 and scenario._reset_coordinator.starts == 1,
        "an immediate sim_time regression does not duplicate /reset_done",
        f"resets={scenario._resets}, starts={scenario._reset_coordinator.starts}",
    )

    scenario._reset_active = False
    clock.advance(scenario_goal._RESET_DONE_GUARD_SECONDS + 0.1)
    scenario._on_state(state_message(sim_time_seconds=46.0))
    reporter.check(
        scenario._resets == 2 and scenario._reset_coordinator.starts == 2,
        "the fallback works again once the short reset_done guard expires",
        f"resets={scenario._resets}, starts={scenario._reset_coordinator.starts}",
    )

    ignored = make_scenario_stub(
        scenario_goal, scenario_goal.INTERFACE_ACTION, clock, []
    )
    ignored._on_reset_done(FakeBool(False))
    reporter.check(
        ignored._resets == 0 and ignored._reset_coordinator.starts == 0,
        "/reset_done False is ignored",
        f"resets={ignored._resets}, starts={ignored._reset_coordinator.starts}",
    )

    fallback = make_scenario_stub(
        scenario_goal, scenario_goal.INTERFACE_ACTION, clock, []
    )
    fallback._last_sim_time = 75.0
    fallback._on_state(state_message(sim_time_seconds=73.0))
    reporter.check(
        fallback._resets == 1 and fallback._reset_coordinator.starts == 1,
        "a sim_time regression without /reset_done still starts one reset",
        f"resets={fallback._resets}, starts={fallback._reset_coordinator.starts}",
    )

    topic = make_scenario_stub(
        scenario_goal, scenario_goal.INTERFACE_TOPIC, clock, []
    )
    topic._on_reset_done(FakeBool(True))
    reporter.check(
        topic._resets == 1 and topic._reset_active is False,
        "the topic interface records the episode without entering Nav2 reset",
        f"resets={topic._resets}, reset_active={topic._reset_active}",
    )


def the_new_goal_is_armed_only_after_reset(scenario_goal, reporter) -> None:
    events: list[str] = []

    class FakeCoordinator:
        def step(self) -> None:
            events.append("reset-step")

    scenario = type("Scenario", (), {})()
    scenario._reset_active = True
    scenario._reset_coordinator = FakeCoordinator()
    scenario._target = None
    scenario._arm = lambda: events.append("goal")
    scenario._goal_handle = None
    scenario._busy = True
    scenario._logger = FakeLogger(events)
    scenario.get_logger = lambda: scenario._logger
    scenario._tick = MethodType(scenario_goal.ScenarioGoal._tick, scenario)
    scenario._episode_reset_finished = MethodType(
        scenario_goal.ScenarioGoal._episode_reset_finished, scenario
    )

    scenario._tick()
    during_reset = list(events)
    scenario._episode_reset_finished()
    scenario._tick()
    visible = [event for event in events if not event.startswith("log:")]
    reporter.check(
        during_reset == ["reset-step"] and visible == ["reset-step", "goal"],
        "the timer arms the new goal only after reset completion",
        f"during={during_reset}, after={visible}",
    )


def main() -> int:
    module = load_coordinator_module()
    scenario_goal = load_scenario_goal_module()
    reporter = Reporter()
    the_sequence_is_cancel_then_clears_then_empty_then_goal(module, reporter)
    absent_services_are_retried_then_skipped(module, reporter)
    a_cancel_future_that_never_answers_is_bounded(module, reporter)
    a_late_goal_acceptance_is_canceled(scenario_goal, reporter)
    topic_interface_does_not_enter_nav2_reset(scenario_goal, reporter)
    reset_done_is_primary_and_clock_is_only_fallback(scenario_goal, reporter)
    the_new_goal_is_armed_only_after_reset(scenario_goal, reporter)

    if reporter.failures:
        print(f"\n{reporter.failures} check(s) failed")
        return 1
    print("\nall checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
