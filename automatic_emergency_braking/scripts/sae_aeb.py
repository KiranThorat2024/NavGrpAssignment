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
ANGLE_RANGE = 270           # Hokuyo 10LX has 270 degree scan.
DISTANCE_THRESHOLD = 3      # Distance threshold before collision (m)
VELOCITY = 0.5              # Maximum Velocity of the vehicle
TIME_THRESHOLD = 1          # Time threshold before collision (s)
STEERING_ANGLE = 0          # Steering angle is uncontrolled

# P-Controller Parameters
#kp_dist = 
#kp_ttc = 

dist_error = 0.0
time_error = 0.0

pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)

def dist_control(distance):
	global kp_dist
	global VELOCITY
	global DISTANCE_THRESHOLD 
	global STEERING_ANGLE
	
	# TO-DO: Calculate Distance to Collision Error
	# ---

	# ---

	print("Distance before collision is = ", distance)
	print("Vehicle velocity= ", velocity)

	msg = AckermannDriveStamped()
	msg.drive.speed = velocity
	msg.drive.steering_angle = STEERING_ANGLE
	pub.publish(msg)

def TTC_control(distance):
	global kp_ttc
	global TIME_THRESHOLD
	global VELOCITY
	global STEERING_ANGLE

	# TO-DO: Calculate Time To Collision Error
	# ---

	# ---
		
	print("Time to collision in seconds is = ", time)
	print("Vehicle velocity = ", velocity)

	msg = AckermannDriveStamped()
	msg.drive.speed = velocity
	msg.drive.steering_angle = STEERING_ANGLE
	pub.publish(msg)

def get_index(angle, data):
	# TO-DO: For a given angle, return the corresponding index for the data.ranges array
	# ---

	# ---
	print("")
	
# Use this function to find the average distance of a range of points directly in front of the vehicle.
def get_distance(data): 
	global ANGLE_RANGE
	
	angle_front = 90   # Range of angle in the front of the vehicle we want to observe
	avg_dist = 0
	
	# Get the corresponding list of indices for given range of angles
	index_front = get_index(angle_front, data)

	# TO-DO: Find the avg range distance
	# ---

	# ---
	
	print("Average Distance = ", avg_dist)

	return avg_dist

def callback(data):

	# TO-DO: Complete the Callback. Get the distance and input it into the controller
	# ---

	# ---
	laserscan = data
	laserscan_ranges = laserscan.ranges
	#laserscan_ranges = np.round(laserscan.ranges, 4)
	print("Ranges: ", laserscan_ranges)
	

if __name__ == '__main__':
	print("AEB started")
	rospy.init_node('aeb',anonymous = True)
	rospy.Subscriber("/scan",LaserScan,callback)
	laserscan = LaserScan()
	rospy.spin()
