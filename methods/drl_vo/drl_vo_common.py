"""Small helpers the DRL-VO nodes share.

Every pose and velocity here is in the ROS frame RobotSNAP publishes in: x
forward, y left, z up, yaw counter-clockwise around z, in radians.
"""

from __future__ import annotations

import json
import math

#: Every agent, in the robot frame, as a JSON body. RobotSNAP writes it.
TOPIC_AGENTS = "/simulation/agents"
#: Whole-session snapshot, including the primary robot's goal, as JSON.
TOPIC_STATE = "/simulation/state"
#: Robot pose and body twist.
TOPIC_ODOM = "/odom"
#: Lidar scan.
TOPIC_SCAN = "/scan"
#: Robot velocity command, consumed by the simulator through the gateway.
TOPIC_CMD_VEL = "/cmd_vel"

#: The observation DRL-VO's policy reads. One ``std_msgs/Float64MultiArray``
#: carrying ``[ped 12800][scan 7200][goal 2]``, exactly what the ROS 1
#: ``cnn_data_pub.py`` published on ``/cnn_data``.
TOPIC_OBSERVATION = "/drl_vo/observation"


def yaw_from_quaternion(q) -> float:
    """Planar yaw of a quaternion, in radians."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def wrap_angle(angle: float) -> float:
    """The same angle brought back into ``(-pi, pi]``."""
    return math.atan2(math.sin(angle), math.cos(angle))


def number(value, default: float = 0.0) -> float:
    """``float(value)`` for a real number, ``default`` for anything else.

    ``bool`` is excluded on purpose: ``True`` is an int in Python, and a flag
    read as a coordinate is a bug that would show up as a robot at (1, 0).
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return default
    return float(value)


def json_body(payload: str) -> dict | None:
    """The object a JSON payload carries, or ``None`` when it is not one."""
    try:
        body = json.loads(payload)
    except (TypeError, ValueError):
        return None
    return body if isinstance(body, dict) else None
