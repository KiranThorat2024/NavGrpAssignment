#!/usr/bin/env python3
# Imports
from ackermann_msgs.msg import AckermannDriveStamped
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
    def __init__(self, _waypoints, _rate):
        # Subscribers, publishers, topics, etc
        rospy.Subscriber("/vesc/odom", Odometry, self.pose_callback)
        self.pub = rospy.Publisher("drive_parameters", drive_param, queue_size=1)  # velocity and angle
        self.pub2 = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
        self.rate = rospy.Rate(_rate)

        # Waypoints, car variables, constants, etc.
        self.waypoints = _waypoints
        self.waypoints_x = []
        self.waypoints_y = []
        for point in self.waypoints:
            self.waypoints_x.append(float(point[0]))
            self.waypoints_y.append(float(point[1]))
        self.LOOKAHEAD = 1.2
        self.WB = 0.3421
        self.MAX_VELOCITY = 0.6
        self.k = 0.75

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
        # 2 points (x1, y1) and (x2, y2) to form line
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
        # TODO: NEED TO UPDATE THIS VERTICAL LINE SECTION
        if x2 == x1:  # Need to check for vertical line to avoid slope calculation divide by 0
            # Calculate distance from point (center of circle to line)
            dist = np.abs(h - x1)

            # Polynomial coefficients of circle-line intersection formula
            a = 1
            b = -2 * k
            c = h ** 2 + k ** 2 - r ** 2 + x1 * (x1 - 2 * h)

            # Compute the roots to find the y value (since vertical line, we already know x value)
            roots = np.real(np.roots([a, b, c]))
            print("Vertical line, Roots: ", roots)

            # Calculate the discriminant to determine incidence of line and circle
            dscmt = b ** 2 - 4 * a * c

            x_intp = x1
            if dscmt > 0:  # Two solutions
                # Need to pick intersection closest to middle of next lookahead waypoints
                mid_la_waypoint = [x1 + (x2 - x1) / 2, y1 + (y2 - y1) / 2]
                if eucl_dist(x_intp, roots[0], mid_la_waypoint[0], mid_la_waypoint[1]) < \
                        eucl_dist(x_intp, roots[1], mid_la_waypoint[0], mid_la_waypoint[1]):
                    y_intp = roots[0]
                else:
                    y_intp = roots[1]
                print("Vertical line: Two intersections, picking: ({:.2f}, {:.2f})".format(x_intp, y_intp))
            elif dscmt < 0:  # No solutions
                y_intp = y1
                print("Vertical line: No intersection, picking ({:.2f}, {:.2f})".format(x_intp, y_intp))
            else:  # One solution
                y_intp = roots[0]
                print("Vertical line: One intersection: ({:.2f}, {:.2f})".format(x_intp, y_intp))
        else:
            # Compute slope and intercept of line
            m = (y2 - y1) / (x2 - x1)
            intercept = y1 - m * x1
            print("LA Points: ({:.2f}, {:.2f}) ({:.2f}, {:.2f}) m={:.2f} b={:.2f}".
                  format(x1, y1, x2, y2, m, intercept))

            # Polynomial coefficients of circle-line intersection formula
            a = 1 + m ** 2
            b = 2 * (m * (intercept - k) - h)
            c = -r ** 2 + h ** 2 + k ** 2 + intercept ** 2 - (2 * k * intercept)

            # Compute the roots to find the x value of intercept
            roots = np.real(np.roots([a, b, c]))
            print("Roots: ", roots)

            # Calculate the discriminant to determine incidence of line and circle
            dscmt = b ** 2 - 4 * a * c
            if dscmt > 0:  # Two solutions
                # Need to pick intersection closest to middle of next lookahead waypoints
                mid_la_waypoint = [x1 + (x2 - x1) / 2, y1 + (y2 - y1) / 2]
                roots_y = [m * roots[0] + intercept, m * roots[1] + intercept]
                if eucl_dist(roots[0], roots_y[0], mid_la_waypoint[0], mid_la_waypoint[1]) < \
                        eucl_dist(roots[1], roots_y[1], mid_la_waypoint[0], mid_la_waypoint[1]):
                    x_intp = roots[0]
                    y_intp = roots_y[0]
                else:
                    x_intp = roots[1]
                    y_intp = roots_y[1]
                print("Two intersections: ({:.2f}, {:.2f}), ({:.2f}, {:.2f})".
                      format(roots[0], roots_y[0], roots[1], roots_y[1]))
            elif dscmt < 0:  # No solutions
                x_intp = x2
                y_intp = y2
                print("No intersection, picking ({:.2f}, {:.2f})".format(x_intp, y_intp))
            else:  # One solution
                x_intp = roots[0]
                y_intp = m * roots[0] + intercept
                print("One intersection: ({:.2f}, {:.2f})".format(x_intp, y_intp))

        print("Target: ({:.2f}, {:.2f})".format(x_intp, y_intp))

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

    def follow_waypoints(self):
        # PURE PURSUIT CODE
        try:
            while self.pure_pursuit_flag:
                # Find nearest waypoint and calculated next waypoint lookahead distance away
                nearest_idx = self.find_nearest_waypoint()
                idx_near_lookahead = self.idx_close_to_lookahead(nearest_idx)
                print("\nCurrent pose: ({:.2f}, {:.2f} Nearest Idx: {:d} Near LA Idx: {:d})".
                      format(self.pose.x, self.pose.y, nearest_idx, idx_near_lookahead))

                # Calculate target pose using interpolation
                target_x, target_y = self.interpolate(idx_near_lookahead)

                # Show some animations
                if self.show_animation:
                    self.animate(target_x, target_y, idx_near_lookahead)

                # Calculate error = alpha = theta - yaw
                # theta is slope of line between reference and waypoint
                # yaw is orientation of vehicle from odometer
                rise = target_y - self.pose.y
                run = target_x - self.pose.x

                theta = np.arctan(np.abs(rise / run))
                if run > 0:  # Going forward
                    if rise > 0:  # Positive slope
                        e = theta - self.pose.yaw
                        print("Forward +m")
                    elif rise < 0:  # Negative slope
                        e = -theta - self.pose.yaw
                        # e = self.pose.yaw + theta
                        print("Forward -m")
                    else:  # Horizontal line pointing right
                        e = - self.pose.yaw
                        print("Right horizontal")
                elif run < 0:  # Going backward
                    if rise > 0:  # Negative slope
                        # e = np.pi - theta - self.pose.yaw
                        if self.pose.yaw > 0:
                            e = np.pi - theta - self.pose.yaw
                        else:
                            e = -(np.pi + theta + self.pose.yaw)
                        print("Backward -m")
                    elif rise < 0:  # Positive slope
                        # e = np.pi + theta - self.pose.yaw

                        if self.pose.yaw > 0:
                            e = np.pi - self.pose.yaw + theta
                        else:
                            e = theta - np.pi - self.pose.yaw
                        print("Backward +m")
                    else:  # Horizontal line pointing left
                        e = np.pi - self.pose.yaw
                        print("Left horizontal")
                else:  # Vertical line
                    if rise > 0:  # Pointing up
                        e = np.pi / 2 - self.pose.yaw
                        print("Up vertical")
                    elif rise < 0:  # Pointing down
                        e = 3 * np.pi / 2 - self.pose.yaw
                        print("Down vertical")
                    else:  # On the dot, don't know what to do...
                        print("Error: Current pose and waypoint are equal")
                        break

                # A different method for calculating e
                # ld = eucl_dist(self.pose.x, self.pose.y, target_x, target_y)
                # e = ld * np.sin(self.pose.yaw)

                # P controller for steering angle
                steering_angle = e * self.k
                if steering_angle > 0.5:
                    steering_angle = 0.5
                if steering_angle < -0.5:
                    steering_angle = -0.5
                print("Theta: {:.2f} Yaw: {:.2f} Error: {:.2f} Steering angle: {:.2f}".
                      format(theta, self.pose.yaw, e, steering_angle))

                # For now just use constant velocity
                velocity = self.MAX_VELOCITY

                # Publish the message
                # msg = drive_param()
                # msg.velocity = velocity
                # msg.angle = steering_angle
                # self.pub.publish(msg)
                msg = AckermannDriveStamped()
                msg.drive.speed = velocity
                msg.drive.steering_angle = steering_angle
                self.pub2.publish(msg)

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

    def animate(self, target_x, target_y, idx_near_lookahead):
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        plot_arrow(self.pose.x, self.pose.y, self.pose.yaw)
        plt.plot(self.waypoints_x, self.waypoints_y, ".-r", label="course")
        plt.plot(self.pose.x, self.pose.y, "-b", label="trajectory")
        plt.plot(target_x, target_y, "xk", label="target", markersize=12)
        plt.axline((self.waypoints_x[idx_near_lookahead], self.waypoints_y[idx_near_lookahead]),
                   (self.waypoints_x[idx_near_lookahead + 1], self.waypoints_y[idx_near_lookahead + 1]))
        plt.axis("equal")
        plt.grid(True)
        plt.title("Pure Pursuit Control")

        # Add some more graphics to plot for debugging
        ax = plt.gcf().gca()
        ax.add_patch(plot_circle(self.pose.x, self.pose.y, self.LOOKAHEAD))

        plt.pause(0.001)


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


def plot_circle(_h, _k, _r):
    return patches.Circle((_h, _k), radius=_r, color='g', fill=False)


def plot_arrow(_x, _y, _yaw, length=1.0, width=0.5, fc="b", ec="k"):
    plt.arrow(_x, _y, length * math.cos(_yaw), length * math.sin(_yaw),
              fc=fc, ec=ec, head_width=width, head_length=width)


if __name__ == '__main__':
    rospy.init_node('pure_pursuit', anonymous=True)

    # Create object and pass the waypoints to it
    pure_pursuit = PurePursuit(_waypoints=read_points(), _rate=5)
    print("Attempting to follow waypoints...")
    pure_pursuit.follow_waypoints()
