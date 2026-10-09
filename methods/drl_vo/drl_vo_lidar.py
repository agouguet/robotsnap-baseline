"""Reshape a RobotSNAP lidar scan into the beam grid DRL-VO was trained on.

The published policy saw a Hokuyo UTM-30LX: 1080 rays over 270 degrees, 0.25
degrees apart. Its observation builder kept ``ranges[180:900]``, that is the
central 720 rays spanning -90 to +90 degrees, and the network's 6400-wide scan
window is ten of those frames.

RobotSNAP's own lidar has fewer rays and a shorter range, and a scenario may
pick any of the profiles. This module resamples whatever arrives onto that same
-90..+90 degree, 0.25 degree grid, so the network always sees the geometry it
expects whatever sensor the robot carries.
"""

from __future__ import annotations

import math

import numpy as np

#: Rays in one frame, the count DRL-VO's scan history is built from.
BEAMS = 720
#: Angular span of one frame, centred on the robot's forward axis.
SPAN_RAD = math.pi
#: Distance reported for a beam that saw nothing, in metres.
#:
#: DRL-VO normalises the scan against a 30 m sensor: 30 m maps to ``+1`` ("clear
#: ahead") and 0 m to ``-1`` ("obstacle at the bumper"). A no-return has to be
#: reported as the far end of that scale. The ROS 1 publisher instead turned an
#: infinite range into 0, which reads as an obstacle at zero distance and would
#: freeze a robot behind a shorter-range sensor; this node does not reproduce
#: that.
NO_RETURN_M = 30.0


def _no_return(value: float, range_min: float, range_max: float) -> bool:
    if not math.isfinite(value):
        return True
    return value <= range_min or value >= range_max * 0.999


def resample_scan(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    beams: int = BEAMS,
    span_rad: float = SPAN_RAD,
) -> np.ndarray:
    """The ``beams`` distances of one frame, on DRL-VO's centred beam grid.

    A beam whose angle the sensor does not cover, and a beam that saw nothing,
    both come back as :data:`NO_RETURN_M`. Beam *i* of the result sits at
    ``-span/2 + i * span/(beams-1)``.
    """
    raw = np.asarray(ranges, dtype=np.float64)
    count = raw.size
    out = np.full(beams, NO_RETURN_M, dtype=np.float64)
    if count == 0 or angle_increment == 0.0:
        return out

    covered = abs(angle_increment) * (count - 1)
    full_circle = covered >= 2.0 * math.pi - 1e-6

    for index in range(beams):
        angle = -span_rad * 0.5 + index * span_rad / (beams - 1)
        if full_circle:
            # A sensor that sees all around names its first beam at angle_min and
            # wraps; a negative bearing belongs at the far end of its sweep.
            angle = angle_min + (angle - angle_min) % (2.0 * math.pi)
        position = (angle - angle_min) / angle_increment
        nearest = int(round(position))
        if nearest < 0 or nearest >= count:
            continue
        value = float(raw[nearest])
        out[index] = NO_RETURN_M if _no_return(value, range_min, range_max) else value
    return out
