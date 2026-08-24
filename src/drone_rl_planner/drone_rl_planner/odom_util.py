"""Shared odometry helpers.

REP-105: nav_msgs/Odometry twist is expressed in child_frame_id
(base_link). Several consumers need world/map-frame velocity instead.
"""
from __future__ import annotations

import math

import numpy as np


def world_vel_2d(odometry) -> np.ndarray:
    """Body-frame twist.linear rotated into the world frame (XY only)."""
    v = odometry.twist.twist.linear
    q = odometry.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * v.x - s * v.y, s * v.x + c * v.y], dtype=np.float64)
