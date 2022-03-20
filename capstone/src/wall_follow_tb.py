#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class WallFollow(object):
    def __init__(self):
        # Variables, constants, etc
        self.laserScanData = None
        self.msg = Twist()
        self.msg.linear.x = 0.15  # Constant linear speed
        self.ANGLE_PERP = 90  # Recall that 0deg is straight ahead and 90deg is straight right
        self.ANGLE_LOOKAHEAD = 60
        self.DIST_STEADY = 0.5  # This is how far we want to be from wall
        self.error = 0  # Distance error

        # Subscribers and publishers
        self.lscan_sub = rospy.Subscriber("/scan", LaserScan, self.lscan_callback)  # Laser scan subscriber
        self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)

        # Wait until we get at least one data back from callbacks
        while self.laserScanData is None:
            print("Waiting for all callbacks to get at least one data back")
            self.rate.sleep()

    # LaserScan callback
    def lscan_callback(self, _data):
        self.laserScanData = _data

    # Calculate distance from wall based on two angles
    def distance_from_wall(self, angle_perp, angle_la):
        # Find the actual distance from the wall.
        # alpha is the angle of the vehicle from the desired path
        # alpha = tan^-1( (a*cos(theta) - b) / (a*sin(theta)))
        # a = distance at angle_lookahead
        # b = distance at angle_perp
        # theta = angle between angle_perp and angle_lookahead
        theta = np.deg2rad(np.abs(angle_la - angle_perp))
        a = self.laserScanData.ranges[-angle_la]
        b = self.laserScanData.ranges[-angle_perp]

        # Exit if distance cannot be determined
        if np.isnan(a) or np.isnan(b):
            return np.nan, np.nan, np.nan

        alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
        dist_m = b * np.cos(alpha)
        dist_p = dist_m + 0.85 * np.sin(alpha)  # Projected distance

        return dist_m, dist_p, alpha

    # Run the algorithm
    def follow_the_wall(self):
        # Calculate distance from wall using two angles
        dist, dist_proj, alpha = self.distance_from_wall(
            self.ANGLE_PERP, self.ANGLE_LOOKAHEAD)

        # Calculate error
        if np.isnan(dist_proj):
            pass  # Keep last known error
        else:
            self.error = self.DIST_STEADY - dist_proj

        # Print some stuff for debugging
        print("Right: {:.2f} Proj: {:.2f} Error: {:.2f}".
              format(dist, dist_proj, self.error))

        self.rate.sleep()
