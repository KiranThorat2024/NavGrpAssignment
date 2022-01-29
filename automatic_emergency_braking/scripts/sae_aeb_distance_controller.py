#!/usr/bin/env python3

import rospy
import math
import numpy as np
import yaml
import sys
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import pdb

# Vehicle parameters
ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degree scan.
DISTANCE_THRESHOLD = 3  # Distance threshold before collision (m)
VELOCITY = 0.5  # Maximum Velocity of the vehicle
TIME_THRESHOLD = 1.0  # Time threshold before collision (s)
STEERING_ANGLE = 0  # Steering angle is uncontrolled

# P-Controller Parameters
kp_dist = 0.70
kp_ttc = 0.75

# Initial velocity
VELOCITY_TTC = VELOCITY

pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)


def dist_control(distance):
    global kp_dist
    global VELOCITY
    global DISTANCE_THRESHOLD
    global STEERING_ANGLE

    # Calculate Distance to Collision Error
    # Co = Kp*e + Cb
    # Co = controller output
    # Kp = proportional gain
    # e = error = set_point - measurement
    # Cb = controller output with error = 0
    dist_error = distance - DISTANCE_THRESHOLD
    velocity = kp_dist * dist_error

    # Clamp velocity maximum speed
    if velocity >= VELOCITY:
        velocity = VELOCITY
    # Clamp velocity to 0 when practically stopped
    elif velocity <= 0.05:
        velocity = 0

    print("Distance before collision is = ", round(distance, 2))
    print("Vehicle velocity= ", round(velocity, 2))

    msg = AckermannDriveStamped()
    msg.drive.speed = velocity
    msg.drive.steering_angle = STEERING_ANGLE
    pub.publish(msg)


def TTC_control(distance):
    global kp_ttc
    global TIME_THRESHOLD
    global VELOCITY
    global STEERING_ANGLE
    global VELOCITY_TTC

    # Calculate Time To Collision Error
    if VELOCITY_TTC > 0:
        time = distance / kp_ttc
        time_error = time - TIME_THRESHOLD
        VELOCITY_TTC = time_error / kp_ttc 
 
        # Clamp velocity maximum speed
        if VELOCITY_TTC >= VELOCITY:
            VELOCITY_TTC = VELOCITY
        # Clamp velocity to 0 when practically stopped
        elif VELOCITY_TTC <= 0.05:
            VELOCITY_TTC = 0

        print("Distance = ", round(distance, 2))
        print("Time to collision in seconds is = ", round(time, 2))
        print("Vehicle velocity = ", round(VELOCITY_TTC, 2))
        
    msg = AckermannDriveStamped()
    msg.drive.speed = VELOCITY_TTC
    msg.drive.steering_angle = STEERING_ANGLE
    pub.publish(msg)
    

def get_index(angle, data):
    # For a given angle, return the corresponding index for the data.ranges array
    index = int(np.round((np.deg2rad(angle - 90) - data.angle_min) / data.angle_increment + 1, 0))
    return index


# Use this function to find the average distance of a range of points directly in front of the vehicle.
def get_distance(data):
    global ANGLE_RANGE

    # Range of angle in the front of the vehicle we want to observe
    angle_front = np.arange(85, 96, 1)
    front_hits = 0   # Used to count number of laser hits in the angle_front range
    avg_dist = 0

    # Get the corresponding list of indices for given range of angles
    for angle in angle_front:
        index = get_index(angle, data)
        dist = data.ranges[index]
        if ~np.isinf(dist):
            front_hits += 1
            avg_dist += data.ranges[index]

    if front_hits:      # Checks for at least 1 hit
        avg_dist /= front_hits
        # print("Front hits: ", front_hits)
    else:
        avg_dist = np.inf
    return avg_dist


def callback(data):
    # Get the distance and input it into the controller
    laserscan = data
    distance = get_distance(laserscan)

    dist_control(distance)
    #TTC_control(distance)


if __name__ == '__main__':
    print("AEB started")
    rospy.init_node('aeb', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()
