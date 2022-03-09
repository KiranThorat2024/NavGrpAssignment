#!/usr/bin/env python3
# Imports
import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import math
import numpy as np
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
import rospkg
from rospkg import RosPack
from nav_msgs.msg import Odometry
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import matplotlib.pyplot as plt
from matplotlib import patches

# GLOBAL VARIABLES
yaw = 0
idx = 0
way_x = []
way_y = []


class PurePursuit(object):
    def __init__(self, _waypoints):
        # Subscribers, publishers, topics, etc
        rospy.Subscriber("/vesc/odom", Odometry, self.pose_callback)
        self.pub = rospy.Publisher("drive_parameters", drive_param, queue_size=1)  # velocity and angle
        self.rate = rospy.Rate(1)

        # Waypoints, car variables
        self.waypoints = _waypoints
        self.LOOKAHEAD = 1.2
        self.WB = 0.3421

        # Current robot position object
        self.pose = self.CurrPose()

        # Flags
        self.pure_pursuit_flag = True
        self.show_animation = True

    def pose_callback(self, _data):
        # Store current car position
        self.pose.x = _data.pose.pose.position.x
        self.pose.y = _data.pose.pose.position.y

        # Convert Quaternions to Eulers
        self.pose.qx = _data.pose.pose.orientation.x
        self.pose.qy = _data.pose.pose.orientation.y
        self.pose.qz = _data.pose.pose.orientation.z
        self.pose.qw = _data.pose.pose.orientation.w
        self.pose.quaternion = (self.pose.qx, self.pose.qy, self.pose.qz, self.pose.qw)
        self.pose.euler = euler_from_quaternion(self.pose.quaternion)
        self.pose.yaw = self.pose.euler[2]

    def find_distance(self, x1, y1):
        global xc, yc, yaw, waypoints
        distance = math.sqrt((x1 - xc) ** 2 + (y1 - yc) ** 2)
        return distance

    def find_distance_index_based(self, _idx):
        x1 = float(self.waypoints[_idx][0])
        y1 = float(self.waypoints[_idx][1])
        distance = math.sqrt((x1 - self.pose.x) ** 2 + (y1 - self.pose.y) ** 2)
        return distance

    def interpolate(self, _idx):
        global xc, yc, yaw, LOOKAHEAD
        x1 = float(self.waypoints[_idx][0])
        y1 = float(self.waypoints[_idx][1])
        x2 = float(self.waypoints[_idx + 1][0])
        y2 = float(self.waypoints[_idx + 1][1])

        ###### INTERPOLATION ALGEBRA PART ###########
        ## TODO: Find the interpolation. Try catch will help catch the case when there is no solution

        # LEFT OFF HERE
        try:
            flag = "interpolated pt"  # When there are 1 or 2 interpolation points.

        # Resolve between two possible conflicting solutions

        except ValueError:
            flag = "imaginary soln"  # When there is no solution. Set x and y simply to the next waypoint (x2 y2)

        return x, y, flag

    def find_nearest_waypoint(self):
        min_distance = 100

        for point in self.waypoints:
            x1 = float(point[0])
            y1 = float(point[1])
            distance = math.sqrt((x1 - self.pose.x) ** 2 + (y1 - self.pose.y) ** 2)
            if distance < min_distance:
                min_distance = distance
                nearest_idx = self.waypoints.index(point)
        return nearest_idx

    def idx_close_to_lookahead(self, _idx):
        while self.find_distance_index_based(_idx) < self.LOOKAHEAD:
            _idx += 1
        return _idx - 1

    def plot_arrow(self, x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
        """
        Plot arrow
        """
        if not isinstance(x, float):
            for ix, iy, iyaw in zip(x, y, yaw):
                plot_arrow(ix, iy, iyaw)
        else:
            plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                      fc=fc, ec=ec, head_width=width, head_length=width)
            plt.plot(x, y)
            patches.Rectangle((xc, yc), 0.35, 0.2)

    def follow_waypoints(self):
        # PURE PURSUIT CODE
        # for point in waypoints:
        #     way_x.append(float(point[0]))
        #     way_y.append(float(point[1]))

        # for i in range(len(way_x)):
        #     print("i: {:d} x: {:.3f} y: {:.3f}".format(i, way_x[i], way_y[i]))

        try:
            while self.pure_pursuit_flag:
                nearest_idx = self.find_nearest_waypoint()
                idx_near_lookahead = self.idx_close_to_lookahead(nearest_idx)
                print("Nearest Idx: {:d} Near LA Idx: {:d}".format(nearest_idx, idx_near_lookahead))

                target_x, target_y, flag = self.interpolate(idx_near_lookahead)
            #
            #         while flag == "imaginary soln":  # When there is no interpolation
            #
            #             # go-to-goal controller
            #
            #             # publish messages
            #
            #             # Debug info for Controller 2
            #             print("solution :", flag)
            #             print("nearest_index", nearest_idx)
            #             print("nearest_index @ ", d1)
            #             print("next index @ ", d2)
            #             print("-------------")
            #             if d1 > d2:
            #                 print("next index closer than previously nearest index")
            #                 break
            #
            #         # pure pursuit controller
            #
            #         # publish messages
            #
            #         if show_animation:
            #             plt.cla()
            #             # for stopping simulation with the esc key.
            #             plt.gcf().canvas.mpl_connect('key_release_event',
            #                                          lambda event: [exit(0) if event.key == 'escape' else None])
            #             plot_arrow(xc, yc, yaw)
            #             plt.plot(cx, cy, "-r", label="course")
            #             plt.plot(xc, yc, "-b", label="trajectory")
            #             plt.plot(target_x, target_y, "xg", label="target")
            #             plt.axis("equal")
            #             plt.grid(True)
            #             plt.title("Pure Pursuit Control" + str(1))
            #             plt.pause(0.001)
            #
            #         # prints for debugging
            #         print("pure pursuit flag:", pure_pursuit_flag)
            #         print("solution :", flag)
            #         print("nearest_waypoint", nearest_idx)
            #         print("idx_near_lookahead", idx_near_lookahead)
            #         print("vehicle x:", xc, "vehicle y:", yc)
            #         print("following --> target_x :", target_x, "target_y :", target_y)
            #         print("steering angle :", delta)
            #         print("----------------------")
            self.rate.sleep()
        except IndexError:
            print("PURE PURSUIT COMPLETE --> COMPLETED ALL WAYPOINTS")

    class CurrPose:
        x = None
        y = None
        qx = None
        qy = None
        qz = None
        qw = None
        quaternion = None
        euler = None
        yaw = None


def read_points():
    # CHANGE THIS PATH TO WHERE YOU HAVE SAVED YOUR CSV FILES
    rpkg = rospkg.RosPack()
    package_path = rpkg.get_path('pure_pursuit')
    file_name = 'waypoints.csv'
    file_path = package_path + '/waypoints/' + file_name
    with open(file_path) as f:
        path_points = [tuple(line) for line in csv.reader(f)]
    print("Done extracting waypoints...")
    return path_points


if __name__ == '__main__':
    rospy.init_node('pure_pursuit', anonymous=True)

    # Create object and pass the waypoints to it
    pure_pursuit = PurePursuit(read_points())
    while pure_pursuit.pose.x is None:      # Wait until we get at least one data back from pose callback
        time.sleep(1)

    print("Attempting to follow waypoints...")
    pure_pursuit.follow_waypoints()
