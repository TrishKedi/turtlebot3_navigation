#!/usr/bin/env python3
import numpy as np
from geometry_msgs.msg import Twist

def laser_to_cartesian(scan):
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    ranges = np.array(scan.ranges)
    mask = np.isfinite(ranges) & (ranges > 0.05)
    xs = ranges[mask] * np.cos(angles[mask])
    ys = ranges[mask] * np.sin(angles[mask])
    return np.stack((xs, ys), axis=-1)

def compute_repulsive_force(points, d0=0.6, k_rep=0.8):
    force = np.zeros(2)
    for p in points:
        d = np.linalg.norm(p)
        if d == 0 or d > d0:
            continue
        direction = -p / d
        magnitude = k_rep * (1.0 / d**2 - 1.0 / d0**2)
        force += max(magnitude, 0) * direction
    return force

def force_to_cmd(force_vector, max_speed=0.3):
    angle = np.arctan2(force_vector[1], force_vector[0])
    speed = min(np.linalg.norm(force_vector), max_speed)
    cmd = Twist()
    cmd.linear.x = speed * np.cos(angle)
    cmd.angular.z = 2.0 * angle
    return cmd
