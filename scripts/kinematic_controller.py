#!/usr/bin/env python3
import numpy as np
from geometry_msgs.msg import Twist

def compute_tracking_cmd(robot_pose, goal_pose, k_rho=0.5, k_alpha=1.5, k_beta=-0.6, epsilon=0.2):
    x, y, theta = robot_pose
    xg, yg = goal_pose

    dx = xg - x
    dy = yg - y
    rho = np.hypot(dx, dy)
    alpha = np.arctan2(dy, dx) - theta
    beta = -theta - alpha

    if rho < epsilon:
        return None

    v = k_rho * rho
    w = k_alpha * alpha + k_beta * beta

    cmd = Twist()
    cmd.linear.x = v
    cmd.angular.z = w
    return cmd
