#!/usr/bin/env python3
import time
import rospy
from cv_bridge import CvBridge
import cv2 as cv
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
import numpy as np
from tf.transformations import euler_from_quaternion


class Capstone(object):
    def __init__(self):
        # Variables, constants, etc.
        self.laserScanData = None
        self.pose = self.CurrPose()
        self.camRawData = None
        self.task3_complete = False
        self.bridge = CvBridge()

        # Subscribers, publishers, topics, etc
        self.lscan_sub = rospy.Subscriber("/scan", LaserScan, self.lscan_callback)  # Laser scan subscriber
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)  # Odometry pose subscriber
        self.cam_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.cam_callback)  # Camera subscriber
        self.rate = rospy.Rate(1)

        # Wait until we get at least one data back from callbacks
        while self.laserScanData is None or self.pose.x is None or self.camRawData is None:
            print("Waiting for all callbacks to get at least on data back")
            time.sleep(1)

    # LaserScan callback
    def lscan_callback(self, _data):
        self.laserScanData = _data

    # Odometry callback
    def odom_callback(self, _data):
        # Position
        self.pose.x = _data.pose.pose.position.x
        self.pose.y = _data.pose.pose.position.y

        # Convert Quaternions to Eulers to calculate yaw
        _qx = _data.pose.pose.orientation.x
        _qy = _data.pose.pose.orientation.y
        _qz = _data.pose.pose.orientation.z
        _qw = _data.pose.pose.orientation.w
        _quaternion = (_qx, _qy, _qz, _qw)
        _euler = euler_from_quaternion(_quaternion)
        self.pose.yaw = _euler[2]

    # Camera callback
    def cam_callback(self, _data):
        self.camRawData = _data

    def find_largest_gap(self):
        # See image in README file on how ranges are defined
        ranges = np.array([self.laserScanData.ranges[228:282],
                           self.laserScanData.ranges[281:335],
                           self.laserScanData.ranges[0:28] + self.laserScanData.ranges[334:360],
                           self.laserScanData.ranges[28:82],
                           self.laserScanData.ranges[81:135]])

        # Max range of laser scan sensor is 3.5m. Here we replace 'infinite' with 5m so that we can compare
        # accurately when multiple ranges have infinite values
        ranges[ranges == np.inf] = 5.0
        ranges_ss = ss_iterable(ranges)
        return np.argmax(ranges_ss) + 1

    class CurrPose:
        x = None
        y = None
        yaw = None

    # Obstacle avoidance
    def task1(self):
        # Find the largest gap among the 5 defined laser scan regions
        gap = self.find_largest_gap()
        print("S1: Largest gap at: {}, Current Pose (x,y,yaw): ({:.2f}, {:.2f}, {:.2f})".
              format(gap, self.pose.x, self.pose.y, self.pose.yaw))

    # Line following
    def task2(self):
        # Manipulate image
        img = self.bridge.imgmsg_to_cv2(self.camRawData, desired_encoding='bgr8')
        img_height, img_width, _ = img.shape
        img_cropped = img[int(img_height / 2):img_height][1:img_width]
        img_hsv = cv.cvtColor(img_cropped, cv.COLOR_BGR2HSV)
        blue_range = np.array([[140, 127, 38], [191, 255, 255]])
        img_masked_blue = cv.inRange(img_hsv, blue_range[0], blue_range[1])

        cv.imshow("Orig", img)
        cv.waitKey(1)
        cv.imshow("Only blue", img_masked_blue)
        cv.waitKey(1)

    # Navigation goal
    def task3(self):
        print("In task 3")
        return False

    def run_statemachine(self):
        state = 1
        while not self.task3_complete:  # Run state machine until task 3 is complete
            if state == 1:
                self.task1()
                if self.pose.y >= 4.0:
                    state = 2
            elif state == 2:
                self.task2()
                if self.pose.x <= 0.8:
                    state = 3
            elif state == 3:
                self.task3_complete = self.task3()

            self.rate.sleep()


# An iterable sum of squares method
def ss_iterable(_m):
    _result = []
    for i in range(len(_m)):
        _ss = 0
        for j in range(len(_m[i])):
            _ss += _m[i][j] ** 2
        _result.append(_ss)
    return _result


if __name__ == '__main__':
    rospy.init_node('capstone', anonymous=True)
    capstone = Capstone()
    capstone.run_statemachine()

    # Run Task 1
    # capstone.task1()
