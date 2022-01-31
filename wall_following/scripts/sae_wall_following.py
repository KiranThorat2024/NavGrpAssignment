#!/usr/bin/env python3

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import pdb

# Vehicle parameters
ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degrees scan
DISTANCE_RIGHT_THRESHOLD = 0.5  # (m)
VELOCITY = 0.1  # meters per second

# Controller parameters
kp = 0.7
# kd =

# Other global variables
COS_SIN_45 = np.cos(np.pi / 4)  # cos(45deg) = sin(45deg)
error = 0.0
prev_error = 0.0


def control(error):
    global kp
    # global kd
    global VELOCITY

    # TO-DO: Implement controller
    # ---
    steering_angle = 0  # just here to remove red squiggles

    # Set maximum thresholds for steering angles
    if steering_angle > 0.5:
        steering_angle = 0.5
    elif steering_angle < -0.5:
        steering_angle = -0.5

    print("Steering Angle is = %f" % steering_angle)


# TO-DO: Publish the message


def get_index(angle, data):
    # For a given angle, return the corresponding index for the data.ranges array
    index = int(np.round((np.deg2rad(angle - 90) - data.angle_min) / data.angle_increment + 1, 0))
    return index


def distance(angle, angle_lookahead, data):
    global ANGLE_RANGE
    global DISTANCE_RIGHT_THRESHOLD
    global COS_SIN_45

    # Find the actual distance from the wall.
    # alpha is the angle of the vehicle from the desired path
    # alpha = tan^-1( (a*cos(theta) - b) / (a*sin(theta)))
    # a = distance at angle_lookahead
    # b = distance at angle_right
    # theta = angle between angle_right and angle_lookahead, which was chosen as 45 deg
    # Taking advantage that cos(45 deg) = sin(45 deg)
    # alpha = tan^-1(1 + b/(a*sin(theta)))

    a = data.ranges[get_index(angle, data)]
    b = data.ranges[get_index(angle_lookahead, data)]
    alpha = np.arctan(1 + b / (a * COS_SIN_45))
    dist_m = b * np.cos(alpha)

    # For greater accuracy, calculate predicted error
    # dist_p = dist_m + L*np.sin(alpha)

    # Calculate error
    error = DISTANCE_RIGHT_THRESHOLD - dist_m

    return error, dist_m


def follow_center(angle_right, angle_lookahead_right, data):
    angle_lookahead_left = 180 + angle_right
    angle_left = 180 - angle_lookahead_right

    er, dr = distance(angle_right, angle_lookahead_right, data)
    el, dl = distance(angle_left, angle_lookahead_left, data)

    # Find Centerline error
    centerline_error = dr - dl
    print("Distance from left wall : %f" % dl)
    print("Distance from right wall : %f" % dr)
    print("Centerline error = %f " % centerline_error)

    return centerline_error


def callback(data):
    # Pick two rays at two angles
    angle_right = 0
    angle_lookahead = 45

    # To follow right wall
    # er, dr = distance(angle_right,angle_lookahead, data)

    # To follow the centerline
    ec = follow_center(angle_right, angle_lookahead, data)


# control(ec)


if __name__ == '__main__':
    print("Wall following started")
    rospy.init_node('wall_following', anonymous=True)
    s = rospy.Subscriber("/scan", LaserScan, callback)

    rospy.spin()
