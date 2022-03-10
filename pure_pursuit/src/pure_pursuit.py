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


class PurePursuit(object):
    def __init__(self, _waypoints):
        # Subscribers, publishers, topics, etc
        rospy.Subscriber("/vesc/odom", Odometry, self.pose_callback)
        self.pub = rospy.Publisher("drive_parameters", drive_param, queue_size=1)  # velocity and angle
        self.rate = rospy.Rate(1)

        # Waypoints, car variables
        self.waypoints = _waypoints
        self.waypoints_x = []
        self.waypoints_y = []
        for point in self.waypoints:
            self.waypoints_x.append(float(point[0]))
            self.waypoints_y.append(float(point[1]))
        # for i in range(len(way_x)):
        #     print("i: {:d} x: {:.3f} y: {:.3f}".format(i, way_x[i], way_y[i]))
        self.LOOKAHEAD = 1.2
        self.WB = 0.3421

        # Current robot position object
        self.pose = self.CurrPose()

        # Flags
        self.pure_pursuit_flag = True
        self.show_animation = True



        # Wait until we get at least one data back from pose callback
        while self.pose.x is None:
            time.sleep(1)

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
        # 2 Line points (x1, y1) and (x2, y2)
        x1 = float(self.waypoints[_idx][0])
        y1 = float(self.waypoints[_idx][1])
        x2 = float(self.waypoints[_idx + 1][0])
        y2 = float(self.waypoints[_idx + 1][1])

        # Circle center point (h, k) and radius r
        h = self.pose.x
        k = self.pose.y
        r = self.LOOKAHEAD

        # Interpolation algorithm
        # Find intersection between circle of radius LOOKAHEAD and two nearest waypoints
        if x2 == x1:  # Need to check for vertical line to avoid slope calc divide by 0
            # Calculate distance from point (center of circle to line)
            dist = np.abs(h - x1)

            # Polynomial coefficients of circle-line intersection formula
            a = 1
            b = -2 * k
            c = h ** 2 + k ** 2 - r ** 2 + x1 * (x1 - 2 * h)

            # Compute the roots to find the y value (since vertical line, we already know x value)
            roots = np.real(np.roots([a, b, c]))
            print("Vertical line, Roots: ", roots)
            x_intp = x1
            if dist < r:
                # Need to pick intersection closest to next waypoint
                if eucl_dist(x_intp, roots[0], x1, y1) < eucl_dist(x_intp, roots[1], x2, y2):
                    y_intp = y1
                else:
                    y_intp = y2
                print("Vertical line: Two intersections, picking: ({:.2f}, {:.2f})".format(x_intp, y_intp))
            elif dist > r:
                y_intp = y1
                print("Vertical line: No intersection, picking ({:.2f}, {:.2f})".format(x_intp, y_intp))
            else:
                y_intp = roots[0]
                print("Vertical line: One intersection: ({:.2f}, {:.2f})".format(x_intp, y_intp))
        else:
            # Compute slope and intercept of line
            m = (y2 - y1) / (x2 - x1)
            intercept = y1 - m * x1

            # Calculate distance from point (center of circle) to line
            dist = np.abs(-m * h + k - 1) / np.sqrt(m ** 2 + 1)

            # Polynomial coefficients of circle-line intersection formula
            a = 1 + m ** 2
            b = 2 * (m * (intercept - k) - h)
            c = -r ** 2 + h ** 2 + k ** 2 + intercept ** 2 - (2 * k * intercept)

            # Compute the roots to find the x value of intercept
            roots = np.real(np.roots([a, b, c]))
            print("Roots: ", roots)
            # From distance calculation we know if there is an intersection
            if dist < r:
                # Need to pick intersection closest to next waypoint
                roots_y = [np.sqrt(r**2-(roots[0]+h)**2), np.sqrt(r**2-(roots[1]+h)**2)]
                if eucl_dist(roots[0], roots_y[0], x1, y1) < eucl_dist(roots[1], roots_y[1], x2, y2):
                    x_intp = roots[0]
                    y_intp = roots_y[0]
                else:
                    x_intp = roots[1]
                    y_intp = roots_y[1]
                print("Two intersections, picking: ({:.2f}, {:.2f})".format(x_intp, y_intp))
            elif dist > r:
                x_intp = x2
                y_intp = y2
                print("No intersection, picking ({:.2f}, {:.2f})".format(x_intp, y_intp))
            else:
                x_intp = roots[0]
                y_intp = np.sqrt(r**2-(roots[0]+h)**2)
                print("One intersection: ({:.2f}, {:.2f})".format(x_intp, y_intp))

        # From distance calculation we know if line intersects circle
        print("r: {:.2f} dist: {:.2f}".format(r, dist))

        return x_intp, y_intp

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

    def plot_arrow(self, _x, _y, _yaw, length=1.0, width=0.5, fc="r", ec="k"):
        """
        Plot arrow
        """
        if not isinstance(_x, float):
            for ix, iy, iyaw in zip(_x, _y, _yaw):
                self.plot_arrow(ix, iy, iyaw)
        else:
            plt.arrow(_x, _y, length * math.cos(_yaw), length * math.sin(_yaw),
                      fc=fc, ec=ec, head_width=width, head_length=width)
            plt.plot(_x, _y)
            # patches.Rectangle((xc, yc), 0.35, 0.2)
            patches.Rectangle((self.pose.x, self.pose.y), 0.35, 0.2)

    def follow_waypoints(self):
        # PURE PURSUIT CODE
        try:
            while self.pure_pursuit_flag:
                print("\nCurrent pose: ({:.2f}, {:.2f})".format(self.pose.x, self.pose.y))
                nearest_idx = self.find_nearest_waypoint()
                idx_near_lookahead = self.idx_close_to_lookahead(nearest_idx)
                print("Nearest Idx: {:d} Near LA Idx: {:d}".format(nearest_idx, idx_near_lookahead))

                target_x, target_y = self.interpolate(idx_near_lookahead)
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
                if self.show_animation:
                    plt.cla()
                    # for stopping simulation with the esc key.
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event: [exit(0) if event.key == 'escape' else None])
                    self.plot_arrow(self.pose.x, self.pose.y, self.pose.yaw)
                    plt.plot(self.waypoints_x, self.waypoints_y, "-r", label="course")
                    plt.plot(self.pose.x, self.pose.y, "-b", label="trajectory")
                    plt.plot(target_x, target_y, "xg", label="target")
                    plt.axis("equal")
                    plt.grid(True)
                    plt.title("Pure Pursuit Control" + str(1))
                    plt.pause(0.001)
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


def eucl_dist(x1, y1, x2, y2):
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


if __name__ == '__main__':
    rospy.init_node('pure_pursuit', anonymous=True)

    # Create object and pass the waypoints to it
    pure_pursuit = PurePursuit(read_points())
    print("Attempting to follow waypoints...")
    pure_pursuit.follow_waypoints()
