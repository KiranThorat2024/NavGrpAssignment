#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int64
from std_msgs.msg import Float32
import math

steering_angle = 0
velocity = 0.3  # (m/s)


class LaneKeepAssist(object):
    def __init__(self):
        # Subscribers/publishers/topics etc.
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
        self.msg = AckermannDriveStamped()
        self.rate = rospy.Rate(10)

    def canny(self, image):
        # TO-DO: Extract the canny lines
        # ---

        # ---
        # return canny
        pass

    def region_of_interest(self, image):
        triangle = np.array([[(0, 480), (0, 288), (639, 288), (639, 480)]])
        # TO:DO Find the  Region of Interest
        # ---

        # ---
        # return masked_image
        pass

    def average_slope_intercept(self, image, lines):
        # TO-DO: Get and average of the left and right Hough Lines and extract the centerline.
        # The angle between the extracted centerline and desired centerline will be the error.
        # Use cv2.line to display the lines.
        # ---

        # ---
        pass

    def camera_callback(self, data):
        global steering_angle
        global velocity

        # Convert the image message to cv data
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        print("here")
        cv.imshow("CV Image", cv_image)

        # TO-DO: Extract the canny lines
        # canny_image = canny(cv_image)

        # TO:DO Find the  Region of Interest
        # cropped_image = region_of_interest(canny_image)

        # Extract the Hough Lines
        # lines = cv2.HoughLinesP(# --   --- #)

        # TO-DO: Get and average of the left and right Hough Lines and extract the centerline.
        # The angle between the extracted centerline and desired centerline will be the error.
        # Use cv2.line to display the lines.
        # averaged_lines = average_slope_intercept(cv_image, lines)

        # TO-DO: Implement the final controller
        # ---

        # ---

        # while not rospy.is_shutdown():
            # TO-DO: Publish the steering angle and velocity
            # ---

            # ---

            # vel.header.stamp = rospy.Time.now()
            # vel.header.frame_id = "base_link"

            # print("Steering angle: %f" % m)
            # pub.publish(vel)
            # self.rate.sleep()

        # pub.publish(vel)

        # Display converted images
        # cv2.imshow('canny',canny_image)
        # cv2.imshow('ROI',cropped_image)
        # cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('lane_keeping')
    lka = LaneKeepAssist()
    rospy.spin()
