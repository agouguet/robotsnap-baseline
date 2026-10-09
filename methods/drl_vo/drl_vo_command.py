"""From a DRL-VO observation to a robot command. No ROS, no simulator.

This is the arithmetic of ``drl_vo_inference.py``, kept apart from the node that
runs it so it can be read and tested on its own. The numbers are the published
ones, gathered here so they can be checked in one place:

- the goal margin, 0.9 m, below which the robot is stopped;
- the obstacle margin, 0.4 m on the front +-45 degrees, below which the robot
  turns on the spot at 0.7 rad/s;
- the scaling of each input, and the two output spans, forward 0..0.5 m/s and
  yaw -2..2 rad/s.
"""

from __future__ import annotations

import numpy as np

#: Widths of the three blocks of one observation payload.
PEDESTRIAN_FLOATS = 2 * 80 * 80
SCAN_FLOATS = 10 * 720
GOAL_FLOATS = 2
#: Beams one frame of the scan history holds.
FRAME_BEAMS = 720
#: Observation cells one frame of history is reduced to, before the repeat.
SCAN_CELLS = 80
#: How many times the reduced scan is repeated to fill the 6400-wide window.
SCAN_REPEAT = 4

#: Below this distance to the sub-goal, in metres, the robot stops.
GOAL_MARGIN_M = 0.9
#: Below this distance to an obstacle, in metres, it turns on the spot.
OBSTACLE_MARGIN_M = 0.4
#: Yaw rate, in rad/s, of that turn.
OBSTACLE_TURN_RATE = 0.7

#: Range of the inputs the network was trained on, and of its outputs.
PEDESTRIAN_INPUT_SPAN = (-2.0, 2.0)
SCAN_INPUT_SPAN = (0.0, 30.0)
GOAL_INPUT_SPAN = (-2.0, 2.0)
FORWARD_OUTPUT_SPAN = (0.0, 0.5)
YAW_OUTPUT_SPAN = (-2.0, 2.0)


def scale(values: np.ndarray, low: float, high: float) -> np.ndarray:
    """The published ``MaxAbsScaler``: ``[low, high]`` onto ``[-1, 1]``."""
    return 2.0 * (values - low) / (high - low) - 1.0


def reduce_scan(scan: np.ndarray) -> np.ndarray:
    """The window the network takes, from the raw frames of one observation.

    Each of the ten frames is reduced to 80 cells holding the minimum and the
    mean of its nine beams, and the 1600 numbers that come out are repeated four
    times to fill the 6400 the policy expects. It is exactly the reduction
    ``drl_vo_inference.py`` performs.
    """
    frames = np.asarray(scan, dtype=np.float64).reshape(-1, FRAME_BEAMS)
    cells = np.zeros((2 * frames.shape[0], SCAN_CELLS), dtype=np.float64)
    for index in range(frames.shape[0]):
        chunks = frames[index].reshape(SCAN_CELLS, FRAME_BEAMS // SCAN_CELLS)
        cells[2 * index] = chunks.min(axis=1)
        cells[2 * index + 1] = chunks.mean(axis=1)
    return np.tile(cells.reshape(-1), SCAN_REPEAT)


def observation(pedestrians, scan, goal) -> np.ndarray:
    """The scaled 19202-float vector the network reads, from its three parts."""
    return np.concatenate(
        (
            scale(np.asarray(pedestrians, dtype=np.float64), *PEDESTRIAN_INPUT_SPAN),
            scale(reduce_scan(scan), *SCAN_INPUT_SPAN),
            scale(np.asarray(goal, dtype=np.float64), *GOAL_INPUT_SPAN),
        )
    )


def command(policy, data) -> tuple[float, float]:
    """The forward speed and yaw rate of one step, as the published code picks them.

    ``data`` is one ``/drl_vo/observation`` payload: the pedestrian map, then the
    scan history, then the sub-goal, all raw.
    """
    array = np.asarray(data, dtype=np.float64)
    pedestrians = array[:PEDESTRIAN_FLOATS]
    scan = array[PEDESTRIAN_FLOATS:PEDESTRIAN_FLOATS + SCAN_FLOATS]
    goal = array[-GOAL_FLOATS:]

    if np.linalg.norm(goal) <= GOAL_MARGIN_M:
        return 0.0, 0.0

    # The published obstacle test looks at the central 360 of the 720 beams, that
    # is the front +-45 degrees of the newest frame.
    front = scan[-540:-180]
    front = front[front != 0.0]
    nearest = float(front.min()) if front.size else 10.0
    if nearest <= OBSTACLE_MARGIN_M:
        return 0.0, OBSTACLE_TURN_RATE

    action = policy.predict(observation(pedestrians, scan, goal))
    forward = (action[0] + 1.0) * (FORWARD_OUTPUT_SPAN[1] - FORWARD_OUTPUT_SPAN[0]) / 2.0 \
        + FORWARD_OUTPUT_SPAN[0]
    yaw = (action[1] + 1.0) * (YAW_OUTPUT_SPAN[1] - YAW_OUTPUT_SPAN[0]) / 2.0 \
        + YAW_OUTPUT_SPAN[0]
    return float(forward), float(yaw)
