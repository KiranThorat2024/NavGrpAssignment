#!/usr/bin/env python3
import time

import rospkg

import rospy
from cv_bridge import CvBridge
import cv2 as cv

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
import numpy as np
from tf.transformations import euler_from_quaternion
import pure_pursuit_tb


class Capstone(object):
    def __init__(self):
        # Variables, constants, etc.
        self.laserScanData = None
        self.odom = self.CurrOdom()
        self.camRawData = None
        self.task3_complete = False
        self.bridge = CvBridge()
        self.twist_object = Twist()
        self.fixed_linear_speed = 0.18
        self.angular_rate_max = 2.84
        self.kp_task2 = 0.005

        # Subscribers, publishers, topics, etc
        self.lscan_sub = rospy.Subscriber("/scan", LaserScan, self.lscan_callback)  # Laser scan subscriber
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)  # Odometry pose subscriber
        self.cam_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.cam_callback)  # Camera subscriber
        self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.cmdvel_pub_rate = rospy.Rate(10)

        # Wait until we get at least one data back from callbacks
        while self.laserScanData is None or self.odom.Pose.x is None or self.camRawData is None:
            print("Waiting for all callbacks to get at least on data back")
            time.sleep(1)

    # LaserScan callback
    def lscan_callback(self, _data):
        self.laserScanData = _data

    # Odometry callback
    def odom_callback(self, _data):
        # Position data of relevance
        self.odom.Pose.x = _data.pose.pose.position.x
        self.odom.Pose.y = _data.pose.pose.position.y

        # Convert Quaternions to Eulers to calculate yaw
        _qx = _data.pose.pose.orientation.x
        _qy = _data.pose.pose.orientation.y
        _qz = _data.pose.pose.orientation.z
        _qw = _data.pose.pose.orientation.w
        _quaternion = (_qx, _qy, _qz, _qw)
        _euler = euler_from_quaternion(_quaternion)
        self.odom.Pose.yaw = _euler[2]

        # Twist data of relevance
        self.odom.Twist.x_lin = _data.twist.twist.linear.x
        self.odom.Twist.z_ang = _data.twist.twist.angular.z

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

    class CurrOdom:
        class Pose:
            x = None
            y = None
            yaw = None

        class Twist:
            x_lin = None
            # y_lin = None
            # z_lin = None
            # x_ang = None
            # y_ang = None
            z_ang = None

    # Obstacle avoidance
    def task1(self):
        # Find the largest gap among the 5 defined laser scan regions
        gap = self.find_largest_gap()
        print("S1: Largest gap at: {}, Current Pose (x,y,yaw): ({:.2f}, {:.2f}, {:.2f})".
              format(gap, self.odom.Pose.x, self.odom.Pose.y, self.odom.Pose.yaw))

    # Line following
    def task2(self):
        # Don't need the laser scan data anymore, release the subscriber
        self.lscan_sub.unregister()

        # Manipulate image and get the mask of blue line
        img = self.bridge.imgmsg_to_cv2(self.camRawData, desired_encoding='bgr8')
        img_height, img_width, _ = img.shape
        img_cropped = img[img_height - 40:img_height][1:img_width]
        img_hsv = cv.cvtColor(img_cropped, cv.COLOR_BGR2HSV)
        blue_range = np.array([[108, 40, 20], [125, 255, 255]])
        img_masked_blue = cv.inRange(img_hsv, blue_range[0], blue_range[1])

        # Calculate centroid
        m = cv.moments(img_masked_blue, False)
        try:
            cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
        except ZeroDivisionError:
            cx, cy = img_height / 2, img_width / 2

        # Draw centroid on image and display
        cv.circle(img_masked_blue, (int(cx), int(cy)), 10, (0, 0, 255), -1)
        cv.imshow("Original", img)
        cv.imshow("Mask", img_masked_blue)

        # Calculate error
        err = (img_width / 2) - cx  # (+) = too far right, (-) = too far left

        # Clamp angular speed to maximum per Burger bot specs 2.84 rad/s
        angular_rate = self.kp_task2 * err
        if angular_rate > self.angular_rate_max:
            angular_rate = self.angular_rate_max
        elif angular_rate < -self.angular_rate_max:
            angular_rate = -self.angular_rate_max

        # Note turtlebot3 max linear speed is 0.22m/s
        self.twist_object.linear.x = self.fixed_linear_speed
        self.twist_object.angular.z = angular_rate

        # self.move_robot(self.twist_object)
        self.cmdvel_pub.publish(self.twist_object)
        print("**Task2: Err: {:.2f} AngRate: {:.2f}".format(err, angular_rate))
        cv.waitKey(1)

    def clean_task2(self):
        self.clean_class()
        cv.destroyAllWindows()

    # Navigation goal
    def task3(self):
        print("***Task3: Running pure pursuit ...")
        # Get the waypoints
        print("Extracting waypoints...")
        rpkg = rospkg.RosPack()
        path_waypoints = rpkg.get_path('capstone') + '/waypoints/waypoints.csv'

        # Hand over the heavy lifting to the pure pursuit module. Release subs/pubs from this module first
        self.cam_sub.unregister()
        self.odom_sub.unregister()
        self.cmdvel_pub.unregister()
        pure_pursuit = pure_pursuit_tb.PurePursuit(_waypoints=pure_pursuit_tb.read_points(path_waypoints),
                                                   _rate=5)
        # Follow the waypoints
        pure_pursuit.follow_waypoints()
        return True

    def run_statemachine(self):
        state = 1
        while not self.task3_complete:  # Run state machine until task 3 is complete
            if state == 1:
                self.task1()
                if self.odom.Pose.y >= 4.0:
                    state = 2
            elif state == 2:
                self.task2()
                if self.odom.Pose.x <= 0.8:
                    self.clean_task2()
                    state = 3
            elif state == 3:
                self.task3_complete = self.task3()

            self.rate.sleep()

    def compare_twist_commands(self, pub_twist):
        LX = round(pub_twist.linear.x, 2) == round(self.odom.Twist.x_lin, 2)
        AZ = round(pub_twist.angular.z, 2) == round(self.odom.Twist.z_ang, 2)
        equal = LX and AZ
        if not equal:
            rospy.logwarn("The Current Twist is not the same as the one sent, Resending")
            print("  Pub linx: {} actual linx: {}".format(round(pub_twist.linear.x, 2),
                                                          round(self.odom.Twist.x_lin, 2)))
            print("  Pub angx: {} actual angx: {}".format(round(pub_twist.angular.z, 2),
                                                          round(self.odom.Twist.z_ang, 2)))
        else:
            print("Equal")
        return equal

    def move_robot(self, twist_object):
        # We make this to avoid Topic loss, specially at the start
        current_equal_to_new = False
        while not current_equal_to_new:
            self.cmdvel_pub.publish(twist_object)
            self.cmdvel_pub_rate.sleep()
            current_equal_to_new = self.compare_twist_commands(twist_object)

    def clean_class(self):
        # Stop robot with this shutdown hook
        self.twist_object.linear.x = 0.0
        self.twist_object.angular.z = 0.0
        self.cmdvel_pub.publish(self.twist_object)


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


    def shutdownhook():
        rospy.loginfo("Shutdown time!")
        if not capstone.task3_complete:
            capstone.clean_class()


    rospy.on_shutdown(shutdownhook)
    capstone.run_statemachine()

    # Run Task 1
    # capstone.task1()
