"""Turn RobotSNAP's agent snapshot into DRL-VO's pedestrian velocity map.

``cnn_data_pub.py`` of the published policy reads ``/track_ped`` in the robot
frame and paints a 2 x 80 x 80 grid of cell size 0.25 m: one channel for vx, one
for vy, x from 0 to 20 m ahead and y from -10 to +10 m across. RobotSNAP already
publishes every agent in the robot frame on ``/simulation/agents``, so this
module only has to place them.
"""

from __future__ import annotations

import numpy as np

#: Cells along x, each 0.25 m: the grid reaches 20 m ahead.
ROWS = 80
#: Cells along y, each 0.25 m: the grid reaches 10 m each side.
COLS = 80
#: Metres per cell.
CELL_M = 0.25


def velocity_map(payload: dict) -> np.ndarray:
    """The flattened 2 x 80 x 80 velocity map of one ``/simulation/agents`` body.

    The channel-major flattening matches the ROS 1 publisher's list
    comprehension, so ``reshape(2, 80, 80)`` gives (vx, vy) back and the network
    reads it as it was trained to.
    """
    grid = np.zeros((2, ROWS, COLS), dtype=np.float64)
    entries = payload.get("agents") if isinstance(payload, dict) else None
    if not isinstance(entries, list):
        return grid.reshape(-1)

    for entry in entries:
        if not isinstance(entry, dict):
            continue
        x = entry.get("x")
        y = entry.get("y")
        if isinstance(x, bool) or isinstance(y, bool):
            continue
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            continue
        if not (0.0 <= x <= ROWS * CELL_M) or abs(y) > COLS * CELL_M * 0.5:
            continue

        column = int(np.floor((COLS * CELL_M * 0.5 - y) / CELL_M))
        row = int(np.floor(x / CELL_M))
        column = min(max(column, 0), COLS - 1)
        row = min(max(row, 0), ROWS - 1)

        vx = entry.get("vx")
        vy = entry.get("vy")
        grid[0, row, column] = vx if isinstance(vx, (int, float)) and not isinstance(vx, bool) else 0.0
        grid[1, row, column] = vy if isinstance(vy, (int, float)) and not isinstance(vy, bool) else 0.0

    return grid.reshape(-1)
