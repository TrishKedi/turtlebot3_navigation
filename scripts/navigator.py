#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
from global_planner import a_star
from kinematic_controller import compute_tracking_cmd
from potential_fields import laser_to_cartesian, compute_repulsive_force, force_to_cmd

class Navigator:
    def __init__(self):
        rospy.init_node('navigator')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.robot_pose = (0, 0, 0)
        self.scan_data = None
        self.map = None
        self.map_info = None
        self.path = []
        self.current_idx = 0
        self.goal = (2.0, 2.0)  # Hardcoded goal in world coordinates

        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        orientation = euler_from_quaternion([pose.orientation.x,
                                             pose.orientation.y,
                                             pose.orientation.z,
                                             pose.orientation.w])
        self.robot_pose = (pose.position.x, pose.position.y, orientation[2])

    def scan_callback(self, msg):
        self.scan_data = msg

    def map_callback(self, msg):
        self.map_info = msg.info
        grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        grid = np.where(grid == 0, 0, 1)
        self.map = grid

        start = self.world_to_map(self.robot_pose[0], self.robot_pose[1])
        goal = self.world_to_map(self.goal[0], self.goal[1])
        path_cells = a_star(grid, start, goal)
        self.path = [self.map_to_world(r, c) for r, c in path_cells]
        self.publish_path(self.path)

    def world_to_map(self, x, y):
        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return (my, mx)

    def map_to_world(self, row, col):
        x = col * self.map_info.resolution + self.map_info.origin.position.x
        y = row * self.map_info.resolution + self.map_info.origin.position.y
        return (x, y)

    def publish_path(self, path):
        msg = Path()
        msg.header.frame_id = "map"
        for x, y in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.scan_data is None or len(self.path) == 0:
                self.rate.sleep()
                continue

            goal = self.path[self.current_idx]
            cmd = compute_tracking_cmd(self.robot_pose, goal)

            if cmd is None:
                self.current_idx += 1
                if self.current_idx >= len(self.path):
                    self.cmd_pub.publish(Twist())  # stop
                continue

            if min(self.scan_data.ranges) < 0.4:
                points = laser_to_cartesian(self.scan_data)
                force = compute_repulsive_force(points)
                cmd = force_to_cmd(force)

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        navigator = Navigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
