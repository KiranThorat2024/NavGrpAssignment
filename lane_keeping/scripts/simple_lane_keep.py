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
        self.data = None  # This buffer holds newest callback data
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
        self.msg = AckermannDriveStamped()
        self.rate = rospy.Rate(20)

        # Size of image is known and region of interest is fixed
        self.height = 480
        self.width = 640

        # Thresholds
        self.crop_yThresh = [288, self.height]  # [Crop from, Crop to]
        self.canny_thresh = [100, 200]  # [Min, Max]

        print("Init complete")

    def canny(self, _im):
        return cv.Canny(_im, self.canny_thresh[0], self.canny_thresh[1])

    # Crop image and apply image threshold using Otsu's Binarization
    def region_of_interest(self, _im):
        _im_gray = cv.cvtColor(_im, cv.COLOR_BGR2GRAY)
        _im_crop = _im_gray[self.crop_yThresh[0]:self.crop_yThresh[1]][0:self.width]
        _im_blur = cv.GaussianBlur(_im_crop, (5, 5), 0)
        _, _im_bin = cv.threshold(_im_blur, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        return _im_bin

    def hough_lines(self, _im, draw_lines):
        lines_p = cv.HoughLinesP(_im, 1, np.pi / 180, 40, None, 40, 10)
        _im_lines = cv.cvtColor(_im, cv.COLOR_GRAY2BGR)

        # Draw lines
        if draw_lines:
            if lines_p is not None:
                for i in range(0, len(lines_p)):
                    line = lines_p[i][0]
                    cv.line(_im_lines, (line[0], line[1]), (line[2], line[3]), (0, 0, 255), 3, cv.LINE_AA)
            cv.imshow("Extracted Lines", _im_lines)

        return lines_p

    def average_slope_intercept(self, image, lines):
        # TO-DO: Get and average of the left and right Hough Lines and extract the centerline.
        # The angle between the extracted centerline and desired centerline will be the error.
        # Use cv2.line to display the lines.
        # ---

        # ---
        pass

    def camera_callback(self, _data):
        self.data = _data

    def run(self):
        global steering_angle
        global velocity

        # Convert the image message to cv data
        im = self.bridge_object.imgmsg_to_cv2(self.data, desired_encoding="bgr8")

        # Crop Region of Interest
        im_roi = self.region_of_interest(im)

        # Extract the edges using canny edge detector
        im_canny = self.canny(im_roi)

        # Extract the Hough Lines
        lines_p = self.hough_lines(im_canny, draw_lines=True)

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
        cv.waitKey(1)  # Process the opencv stuff
        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('lane_keeping', anonymous=True)
    lka = LaneKeepAssist()
    # rospy.spin()
    while lka.data is None:
        # Wait until first data received from camera
        pass

    while not rospy.is_shutdown():
        lka.run()
