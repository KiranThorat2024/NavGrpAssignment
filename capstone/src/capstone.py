#!/usr/bin/env python3
import time
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np


class Capstone(object):
    def __init__(self):
        # Variables, constants, etc.
        self.laserScanData = None

        # Subscribers, publishers, topics, etc
        self.ls_sub = rospy.Subscriber("/scan", LaserScan, self.ls_callback)  # Laser scan subscriber
        self.rate = rospy.Rate(1)

        # Wait until we get at least one data back from callbacks
        while self.laserScanData is None:
            print("Waiting for callbacks to get at least on data back")
            time.sleep(1)

    # LaserScan callback
    def ls_callback(self, _data):
        self.laserScanData = _data

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
        return np.argmax(ranges_ss)+1

    def task1(self):
        while True:
            gap = self.find_largest_gap()
            print("Largest gap at {}".format(gap))
            # inf_array = ""
            # for i in range(360):
            #     if not self.laserScanData.ranges[i] == np.inf:
            #         inf_array += str(i) + ", "
            # print("\n Found something at: {}".format(inf_array))
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

    # Run Task 1
    capstone.task1()
