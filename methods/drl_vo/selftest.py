#!/usr/bin/env python3
"""Check the DRL-VO port without starting a simulation.

    python selftest.py

It exercises each piece on its own - the lidar resampling, the pedestrian map,
the sub-goal, the command gating - and then the whole chain at once, and it
prints the action the checkpoint gives for a known observation so the number can
be compared with the published implementation. The reference values come from
running the original ``drl_vo_inference.py`` network in the published Docker
image on the same input.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

import drl_vo_agents
import drl_vo_command
import drl_vo_lidar
import drl_vo_subgoal
from drl_vo_network import DrlVoPolicy

HERE = pathlib.Path(__file__).resolve().parent
MODEL = HERE / "model" / "drl_vo.zip"

#: What the published network answers for the three reference observations.
REFERENCE_ACTIONS = [
    [1.0, -0.21133500337600708],
    [1.0, 1.0],
    [1.0, -0.17264600098133087],
]


def reference_payload(goal_xy, scan_m=10.0) -> np.ndarray:
    """One raw payload as the published ``cnn_data_pub.py`` would have built it."""
    pedestrians = np.zeros(2 * 80 * 80, dtype=np.float64)
    frame = np.full(720, scan_m, dtype=np.float64)
    return np.concatenate((pedestrians, np.tile(frame, 10), np.asarray(goal_xy, dtype=np.float64)))


def check(condition, label) -> bool:
    print(f"{'PASS' if condition else 'FAIL'}  {label}")
    return bool(condition)


def main() -> int:
    ok = True

    # -- lidar -------------------------------------------------------------
    # A 270-beam, 270 degree, 10 m sensor, as the Jackal profile publishes, with
    # one obstacle 1.2 m dead ahead.
    count = 270
    increment = np.deg2rad(270.0) / (count - 1)
    ranges = np.full(count, 12.0)
    forward_index = count // 2
    ranges[forward_index] = 1.2
    grid = drl_vo_lidar.resample_scan(ranges, -np.deg2rad(135.0), increment, 0.12, 10.0)
    ok &= check(grid.size == 720, "the scan comes back on the 720-beam grid")
    ok &= check(
        abs(grid[360] - 1.2) < 0.02,
        f"an obstacle 1.2 m dead ahead lands on the centre beam ({grid[360]:.3f} m)",
    )
    ok &= check(
        abs(grid[0] - drl_vo_lidar.NO_RETURN_M) < 1e-6,
        "a beam that saw nothing reads as the far end of the range",
    )

    # -- pedestrians -------------------------------------------------------
    payload = {
        "agents": [
            {"id": "0", "x": 3.0, "y": 0.0, "z": 0.0, "vx": 0.5, "vy": 0.0},
            {"id": "1", "x": 40.0, "y": 0.0, "z": 0.0, "vx": 0.0, "vy": 0.0},
        ]
    }
    grid = drl_vo_agents.velocity_map(payload).reshape(2, 80, 80)
    ok &= check(grid[0, 12, 40] == 0.5, "a pedestrian 3 m ahead and centred is in cell (12, 40)")
    ok &= check(grid.sum() == 0.5, "a pedestrian past 20 m is dropped")

    # -- sub-goal ----------------------------------------------------------
    ahead = drl_vo_subgoal.subgoal_in_robot_frame(0.0, 0.0, 0.0, 5.0, 0.0)
    ok &= check(
        abs(ahead[0] - 2.0) < 1e-9 and abs(ahead[1]) < 1e-9,
        f"a goal 5 m ahead gives a sub-goal at lookahead ({ahead[0]:.3f}, {ahead[1]:.3f})",
    )
    left = drl_vo_subgoal.subgoal_in_robot_frame(0.0, 0.0, 0.0, 0.0, 5.0)
    ok &= check(abs(left[1] - 2.0) < 1e-9, "a goal to the left gives a sub-goal to the left")
    close = drl_vo_subgoal.subgoal_in_robot_frame(0.0, 0.0, 0.0, 0.5, 0.0)
    ok &= check(abs(close[0] - 0.5) < 1e-9, "a goal closer than the lookahead is the sub-goal")

    # -- the network -------------------------------------------------------
    policy = DrlVoPolicy(MODEL, batchnorm="reference")
    actions = [
        [
            float(v)
            for v in policy.predict(
                drl_vo_command.observation(
                    np.zeros(2 * 80 * 80),
                    np.tile(np.full(720, 10.0), 10),
                    np.asarray(goal),
                )
            )
        ]
        for goal in ((1.5, 0.0), (0.0, 1.5), (2.0, 0.0))
    ]
    ok &= check(
        np.allclose(actions, REFERENCE_ACTIONS, atol=1e-6),
        f"the checkpoint answers what the published implementation answers ({actions})",
    )

    # -- the gating --------------------------------------------------------
    clear = reference_payload((2.0, 0.0))
    forward, yaw = drl_vo_command.command(policy, clear)
    ok &= check(forward > 0.0, f"a clear path and a far goal drives forward ({forward:.3f} m/s)")

    reached = reference_payload((0.5, 0.0))
    forward, yaw = drl_vo_command.command(policy, reached)
    ok &= check((forward, yaw) == (0.0, 0.0), "a goal within the goal margin stops the robot")

    blocked = np.array(clear, copy=True)
    # The newest frame's front +-45 degrees live in the last 720 floats, 180..539.
    blocked[-720 + 360] = 0.3
    forward, yaw = drl_vo_command.command(policy, blocked)
    ok &= check(
        forward == 0.0 and abs(yaw - drl_vo_command.OBSTACLE_TURN_RATE) < 1e-9,
        "an obstacle inside the obstacle margin turns the robot on the spot",
    )

    print("\nall checks passed" if ok else "\nsome checks failed")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
