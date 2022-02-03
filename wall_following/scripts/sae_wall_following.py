#!/usr/bin/env python3

import rospy
import numpy as np
import sys
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# Vehicle parameters
ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degrees scan
DISTANCE_RIGHT_THRESHOLD = 0.5  # (m)
VELOCITY = 1.35  # Maximum meters per second

# PID Control parameters defaults for centerline wall follow
kp = 1.5
kd = 0.005
ki = 0.75

# Other global variables
steering_angle_prior = 0.0
error_prior = 0.0
integral_prior = 0.0
dl_pr = 0.0
dr_pj_pr = 0.0
dr_pr = 0.0
cl_err_pr = 0.0
rw_err_pr = 0.0
sa_pr = 0.0
sa_delta_pr = 0.0
vel_pr = 0.0
al_pr = 0.0
RIGHT_WALL_MODE = 1  # 0=centerline, 1=right wall
rate = 40.0
rate_s = 1.0 / rate
velocity = 0.0
data_buffer = LaserScan()
angle_right = 0         # Pick two rays at two angles
angle_lookahead = 30    # lookahead angle

pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)


def control(error):
    global kp, kd, ki
    global velocity, VELOCITY
    global error_prior, integral_prior, steering_angle_prior
    global sa_pr, vel_pr, sa_delta_pr

    # Steering PID controller
    derivative = (error - error_prior)/rate_s
    integral = integral_prior + error*rate_s
    steering_angle = kp*error + kd*derivative + ki*integral

    # Update priors
    error_prior = error
    integral_prior = integral

    # Set maximum thresholds for steering angles
    if steering_angle > 0.5:
        steering_angle = 0.5
    elif steering_angle < -0.5:
        steering_angle = -0.5

    steering_delta = steering_angle - steering_angle_prior
    steering_angle_prior = steering_angle

    if np.abs(error) < 0.1:
        velocity = VELOCITY
    elif np.abs(error) < 0.125:
        velocity = 1.25
    elif np.abs(error) < 0.15:
        velocity = 1.0
    elif np.abs(error) < 0.16:
        velocity = 0.75
    else:
        if RIGHT_WALL_MODE:
            velocity = 0.4
        else:
            velocity = 0.5

    # Set maximum threshold for velocity
    if velocity > VELOCITY:
        velocity = VELOCITY

    # Store for debugging
    sa_pr = "%.3f" % steering_angle
    vel_pr = "%.3f" % velocity
    sa_delta_pr = "%.3f" % steering_delta

    # Publish the message
    msg = AckermannDriveStamped()
    msg.drive.speed = velocity
    msg.drive.steering_angle = steering_angle
    pub.publish(msg)


def get_index(angle, data):
    # For a given angle, return the corresponding index for the data.ranges array
    index = int(np.round((np.deg2rad(angle - 90) - data.angle_min) / data.angle_increment, 0))
    return index


def distance(angle, angle_la, data):
    global velocity
    # Find the actual distance from the wall.
    # alpha is the angle of the vehicle from the desired path
    # alpha = tan^-1( (a*cos(theta) - b) / (a*sin(theta)))
    # a = distance at angle_lookahead
    # b = distance at angle_right
    # theta = angle between angle_right and angle_lookahead
    theta = np.deg2rad(np.abs(angle_la - angle))
    a = data.ranges[get_index(angle_la, data)]
    b = data.ranges[get_index(angle, data)]

    # Exit if distance cannot be determined
    if np.isnan(a) or np.isnan(b):
        return np.nan, np.nan, np.nan

    alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
    dist_m = b * np.cos(alpha)
    dist_p = dist_m + 0.5 * np.sin(alpha)

    return dist_m, dist_p, alpha


def follow_center(ray_angle_right, ray_angle_right_lookahead, data):
    global dl_pr, dr_pr, cl_err_pr, al_pr, error_prior

    ray_angle_left_lookahead = 180 - ray_angle_right_lookahead
    ray_angle_left = 180 - ray_angle_right

    dr, dr_p, right_alpha = distance(ray_angle_right, ray_angle_right_lookahead, data)
    dl, dl_p, alpha = distance(ray_angle_left, ray_angle_left_lookahead, data)

    # Find Centerline error
    if np.isnan(dr_p) or np.isnan(dl_p):
        centerline_error = error_prior
    else:
        centerline_error = dl_p - dr_p

    # Store for debugging
    dl_pr = "%.3f" % dl_p
    dr_pr = "%.3f" % dr_p
    cl_err_pr = "%.3f" % centerline_error
    al_pr = "%.1f" % right_alpha
    return centerline_error


def follow_right_wall(angle, angle_la, data):
    global DISTANCE_RIGHT_THRESHOLD
    global dr_pr, rw_err_pr, dr_pj_pr, al_pr

    dr, dr_p, alpha = distance(angle, angle_la, data)

    # Calculate error to right wall
    if np.isnan(dr_p):
        error = error_prior
    else:
        error = DISTANCE_RIGHT_THRESHOLD - dr_p

    # Store for debugging
    dr_pr = "%.3f" % dr
    dr_pj_pr = "%.3f" % dr_p
    rw_err_pr = "%.3f" % error
    al_pr = "%.1f" % np.rad2deg(alpha)

    return error


def callback(data):
    global data_buffer
    data_buffer = data  # Store the data into the buffer


def run():
    global angle_right, angle_lookahead, data_buffer, RIGHT_WALL_MODE
    global dl_pr, dr_pr, dr_pj_pr, rw_err_pr, kp, kd, ki
    global cl_err_pr, sa_pr, vel_pr, al_pr, sa_delta_pr

    # Run loop at steady rate until Ctr-C or program is terminated
    while not rospy.is_shutdown():
        if RIGHT_WALL_MODE:
            # Override the PID gains for the right wall follow mode
            kp = 3.5
            kd = 0.025
            ki = 0.6

            # To follow right wall
            er = follow_right_wall(angle_right, angle_lookahead, data_buffer)
            control(er)

            # For debugging, print variables
            sys.stdout.write("{:<7} {:<7} {:<7} {:<7} {:<7} {:<7} {:<7}\n".
                             format(dr_pr, dr_pj_pr, rw_err_pr, sa_pr, vel_pr, al_pr, sa_delta_pr))
        else:
            # To follow the centerline
            ec = follow_center(angle_right, angle_lookahead, data_buffer)
            control(ec)

            # For debugging, print variables
            sys.stdout.write("{:<7} {:<7} {:<7} {:<7} {:<7} {:<7} {:<7}\n".
                             format(dl_pr, dr_pr, cl_err_pr, sa_pr, vel_pr, al_pr, sa_delta_pr))

        r.sleep()


if __name__ == '__main__':
    # Get command line argument if it was given, else defaults to Right Wall follow mode
    if len(sys.argv) == 2:
        RIGHT_WALL_MODE = int(sys.argv[1])

    rospy.init_node('wall_following', anonymous=True)
    r = rospy.Rate(rate)

    if RIGHT_WALL_MODE:

        print("Wall follow 0.5 from right wall")
        print("{:<7} {:<7} {:<7} {:<7} {:<7} {:<7} {:<7}".
              format('Right', 'Proj', 'Err', 'Steer', 'Vel', 'Alpha', 'SA Delta'))
    else:
        print("Wall follow using centerline")
        print("{:<7} {:<7} {:<7} {:<7} {:<7} {:<7} {:<7}".
              format('Left', 'Right', 'Err', 'Steer', 'Vel', 'Alpha', 'SA Delta'))

    rospy.Subscriber("/scan", LaserScan, callback)  # Start the subscriber

    # Wait for subscriber to receive at least first message
    while len(data_buffer.ranges) == 0:
        r.sleep()

    # Subscriber got data at least once, start program
    run()
