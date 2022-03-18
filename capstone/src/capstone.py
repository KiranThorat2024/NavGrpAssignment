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

        # Wait until we get at least one data back from callbacks
        while self.laserScanData is None:
            print("Waiting for callbacks to get at least on data back")
            time.sleep(1)

    # LaserScan callback
    def ls_callback(self, _data):
        self.laserScanData = _data

    def task1(self):
        # See image in README file on how ranges are defined
        ranges = [np.array(self.laserScanData.ranges[81:135]),
                  np.array(self.laserScanData.ranges[28:82]),
                  np.array(self.laserScanData.ranges[0:28] + self.laserScanData.ranges[334:360]),
                  np.array(self.laserScanData.ranges[281:335]),
                  np.array(self.laserScanData.ranges[228:282])]

        ranges_ss = ss_iterable(ranges)
        print("ranges sum of squares: ".format(ranges_ss))


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
