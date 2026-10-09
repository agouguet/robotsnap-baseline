"""The sub-goal DRL-VO's policy steers towards.

The published stack gets it from ``pure_pursuit.py``: a point on the global plan,
``lookahead`` metres ahead of the robot. RobotSNAP's missions are single goals,
for which the global plan is the straight line to the goal, so the same point is
found directly - at most ``lookahead`` metres along that line, in the robot
frame, which is the form the policy wants. The 2 m lookahead and the 0.9 m goal
margin are the values ``pure_pursuit.py`` and ``drl_vo_inference.py`` hard-code.
"""

from __future__ import annotations

import math

#: Distance ahead of the robot the sub-goal sits at, in metres.
LOOKAHEAD_M = 2.0


def subgoal_in_robot_frame(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    goal_x: float,
    goal_y: float,
    lookahead: float = LOOKAHEAD_M,
) -> tuple[float, float]:
    """``(x, y)`` of the sub-goal in the robot frame, in metres.

    The point is ``min(lookahead, |goal - robot|)`` from the robot along the line
    to the goal, rotated into (x forward, y left). When the goal is closer than
    the lookahead the sub-goal *is* the goal, which is what lets the policy's own
    ``|goal| <= 0.9`` test stop the robot.
    """
    delta_x = goal_x - robot_x
    delta_y = goal_y - robot_y
    distance = math.hypot(delta_x, delta_y)
    if distance <= 1e-9:
        return 0.0, 0.0

    step = min(lookahead, distance)
    unit_x, unit_y = delta_x / distance, delta_y / distance
    cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
    return (
        step * (cos_yaw * unit_x + sin_yaw * unit_y),
        step * (-sin_yaw * unit_x + cos_yaw * unit_y),
    )
