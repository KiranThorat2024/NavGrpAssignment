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
VELOCITY = 1.35  # meters per second

# PID Control parameters
kp = 2
kd = 0.0075
error_prior = 0.0
steering_angle_prior = 0.0
kpv = 10  # P controller for velocity

# Other global variables
error_prior = 0.0
dl_pr = 0.0
dr_pj_pr = 0.0
dr_pr = 0.0
cl_err_pr = 0.0
rw_err_pr = 0.0
sa_pr = 0.0
vel_pr = 0.0
al_pr = 0.0
RIGHT_WALL_MODE = 1  # 0=centerline, 1=right wall

pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)


def control(error):
    global kp, kd
    global kpv
    global VELOCITY
    global error_prior, steering_angle_prior
    global sa_pr, vel_pr

    # Steering PD controller
    derivative = (error - error_prior)
    steering_angle = kp * error + kd * derivative

    # Velocity P controller
    steering_delta = steering_angle - steering_angle_prior
    velocity = VELOCITY - kpv * steering_delta

    # Update priors
    error_prior = error
    steering_angle_prior = steering_angle

    # Set maximum thresholds for steering angles
    if steering_angle > 0.5:
        steering_angle = 0.5
    elif steering_angle < -0.5:
        steering_angle = -0.5

    # Try to slow down on sharp turns
    if steering_angle > 0.45:
        velocity = 0.15
    elif steering_angle > 0.4:
        velocity = 0.25
    elif steering_angle > 0.325:
        velocity = 0.6
    elif steering_angle > 0.25:
        velocity = 0.9

    # Set maximum threshold for velocity
    if velocity > VELOCITY:
        velocity = VELOCITY

    # Store for debugging
    sa_pr = "%.3f" % steering_angle
    vel_pr = "%.3f" % velocity

    # Publish the message
    msg = AckermannDriveStamped()
    msg.drive.speed = velocity
    msg.drive.steering_angle = steering_angle
    pub.publish(msg)


def get_index(angle, data):
    # For a given angle, return the corresponding index for the data.ranges array
    index = int(np.round((np.deg2rad(angle - 90) - data.angle_min) / data.angle_increment + 1, 0))
    return index


def distance(angle, angle_lookahead, data):
    # Find the actual distance from the wall.
    # alpha is the angle of the vehicle from the desired path
    # alpha = tan^-1( (a*cos(theta) - b) / (a*sin(theta)))
    # a = distance at angle_lookahead
    # b = distance at angle_right
    # theta = angle between angle_right and angle_lookahead
    theta = np.deg2rad(np.abs(angle_lookahead - angle))
    a = data.ranges[get_index(angle_lookahead, data)]
    b = data.ranges[get_index(angle, data)]
    alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
    dist_m = b * np.cos(alpha)
    dist_p = dist_m + 0.375 * np.sin(alpha)

    return dist_m, dist_p, alpha


def follow_center(angle_right, angle_lookahead_right, data):
    global dl_pr, dr_pr, cl_err_pr

    angle_lookahead_left = 180 - angle_lookahead_right
    angle_left = 180 - angle_right

    dr = distance(angle_right, angle_lookahead_right, data)
    dl = distance(angle_left, angle_lookahead_left, data)

    # Find Centerline error
    centerline_error = dr - dl

    # Store for debugging
    dl_pr = "%.3f" % dl
    dr_pr = "%.3f" % dr
    cl_err_pr = "%.3f" % centerline_error
    return centerline_error


def follow_right_wall(angle_right, angle_lookahead, data):
    global DISTANCE_RIGHT_THRESHOLD
    global dr_pr, rw_err_pr, dr_pj_pr, al_pr

    dr, dr_p, alpha = distance(angle_right, angle_lookahead, data)

    # Calculate error to right wall
    error = DISTANCE_RIGHT_THRESHOLD - dr_p

    # Store for debugging
    dr_pr = "%.3f" % dr
    dr_pj_pr = "%.3f" % dr_p
    rw_err_pr = "%.3f" % error
    al_pr = "%.3f" % np.rad2deg(alpha)

    return error


def callback(data):
    global dl_pr, dr_pr, dr_pj_pr, rw_err_pr, cl_err_pr, sa_pr, vel_pr, al_pr

    # Pick two rays at two angles
    angle_right = 0
    angle_lookahead = 30

    if RIGHT_WALL_MODE:
        # To follow right wall
        er = follow_right_wall(angle_right, angle_lookahead, data)
        control(er)
        # For debugging, print variables
        sys.stdout.write('\r' + "{:<7} {:<7} {:<7} {:<7} {:<7} {:<7}".
                         format(dr_pr, dr_pj_pr, rw_err_pr, sa_pr, vel_pr, al_pr))
    else:
        # To follow the centerline
        ec = follow_center(angle_right, angle_lookahead, data)
        control(ec)
        # For debugging, print variables
        sys.stdout.write('\r' + "{:<9} {:<9} {:<11} {:<7} {:<7}".
                         format(dl_pr, dr_pr, cl_err_pr, sa_pr, vel_pr))


if __name__ == '__main__':
    print("Wall following started")
    rospy.init_node('wall_following', anonymous=True)

    if RIGHT_WALL_MODE:
        print("{:<7} {:<7} {:<7} {:<7} {:<7} {:<7}".
              format('Right', 'Proj', 'Err', 'Steer', 'Vel', 'Alpha'))
    else:
        print("{:<9} {:<9} {:<11} {:<7} {:<11}".
              format('Left', 'Right', 'C-line Err', 'Steer', 'Vel'))

    # Start the subscriber
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()
