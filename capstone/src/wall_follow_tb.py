#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class WallFollow(object):
    def __init__(self, desired_wall_distance, desired_speed, collision_distance):
        # Variables and objects
        self.laserScanData = None
        self.msg = Twist()
        self.error = 0  # Distance error

        # Constants
        self.DIST_STEADY = desired_wall_distance  # This is how far we want to be from wall
        self.k = 2.0
        self.msg.linear.x = desired_speed  # Constant linear speed
        self.ANGLE_PERP = 90  # Recall that 0deg is straight ahead and 90deg is straight right
        self.ANGLE_LOOKAHEAD = 60
        self.MAX_STEERING_ANGLE = 2.0  # Max is 2.84
        self.COLLISION_DISTANCE = collision_distance

        # Subscribers and publishers
        self.lscan_sub = rospy.Subscriber("/scan", LaserScan, self.lscan_callback)  # Laser scan subscriber
        self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(20)

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

    # Detect if head on collision is imminent
    def head_on_collision_imminent(self):
        head_dist = self.laserScanData.ranges[0]
        if self.COLLISION_DISTANCE > head_dist > 0.2:
            return True
        else:
            return False

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

        # Detect if head on collision imminent, make robot start turning earlier to avoid
        if self.head_on_collision_imminent():
            offset = 0.55
            steering_angle = offset
        else:
            offset = 0.0
            steering_angle = self.k * self.error + offset

        # Move the vehicle
        if steering_angle > self.MAX_STEERING_ANGLE:
            steering_angle = self.MAX_STEERING_ANGLE
        if steering_angle < -self.MAX_STEERING_ANGLE:
            steering_angle = -self.MAX_STEERING_ANGLE
        self.msg.angular.z = steering_angle
        self.cmdvel_pub.publish(self.msg)

        # Print some stuff for debugging
        print("Right: {:.2f} Proj: {:.2f} Error: {:.2f} SteerAng: {:.2f} Offset: {:.1f} Front Dist: {:.2f}".
              format(dist, dist_proj, self.error, steering_angle, offset, self.laserScanData.ranges[0]))

        self.rate.sleep()

    # Stop robot
    def clean_class(self):
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        self.cmdvel_pub.publish(self.msg)
        self.lscan_sub.unregister()
