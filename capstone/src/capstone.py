#!/usr/bin/env python3
import time

import rospkg
import rospy
from cv_bridge import CvBridge
import cv2 as cv
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
from tf.transformations import euler_from_quaternion
from pure_pursuit_tb import PurePursuit, read_points
from wall_follow_tb import WallFollow


class Capstone(object):
    def __init__(self, _sim_mode, wall_distance, wall_follow_speed, collision_distance,
                 task1_thresh):
        # Variables and objects
        self.odom = self.CurrOdom()
        self.camRawData = None
        self.task3_complete = False
        self.bridge = CvBridge()
        self.twist_object = Twist()
        self.wall_follow = None
        self.dist_traveled_x = 0.0
        self.dist_traveled_y = 0.0
        self.PREV_X = 0.0
        self.PREV_Y = 0.0

        # Constants
        self.fixed_linear_speed = 0.12
        self.angular_rate_max = 2.84
        self.kp_task2 = 0.005
        self.wall_distance = wall_distance
        self.wall_follow_speed = wall_follow_speed
        self.collision_distance = collision_distance
        self.sim_mode = _sim_mode
        self.TASK1_THRESH = task1_thresh

        # Subscribers, publishers, topics, etc
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)  # Odometry pose subscriber
        if sim_mode:  # Camera subscriber
            self.cam_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.cam_callback)
        else:
            self.cam_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.cam_callback)
        self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(20)
        self.cmdvel_pub_rate = rospy.Rate(10)

        # Wait until we get at least one data back from callbacks
        while self.odom.Pose.x is None or self.camRawData is None:
            print("Waiting for callbacks to get at least one data back")
            rospy.Rate(1).sleep()

        self.PREV_X = self.odom.Pose.x
        self.PREV_Y = self.odom.Pose.y

    # Odometry callback
    def odom_callback(self, _data):
        # Position data of relevance
        self.odom.Pose.x = _data.pose.pose.position.x
        self.odom.Pose.y = _data.pose.pose.position.y
        
        # Track distance
        if not self.PREV_X is None:
            self.dist_traveled_x += self.odom.Pose.x - self.PREV_X
            self.dist_traveled_y += self.odom.Pose.y - self.PREV_Y

        self.PREV_X = self.odom.Pose.x
        self.PREV_Y = self.odom.Pose.y

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

    # Wall follow
    def task1(self):
        if self.wall_follow is None:  # Initialize object only once
            print("*Task1: Running wall follow algorithm ...")
            self.wall_follow = WallFollow(self.wall_distance, self.wall_follow_speed, self.collision_distance)

        print("Dist Travelled: ({:.2f}, {:.2f})".format(self.dist_traveled_x, self.dist_traveled_y))
        self.wall_follow.follow_the_wall()

    # Line following
    def task2(self):
        print("task2, dist travelled ({:.2f}, {:.2f})".format(self.dist_traveled_x, self.dist_traveled_y))
        # Manipulate image and get the mask of blue line
        if self.sim_mode:
            img = self.bridge.imgmsg_to_cv2(self.camRawData, desired_encoding='bgr8')
        else:
            img = cv.imdecode(np.fromstring(self.camRawData.data, np.uint8), cv.IMREAD_COLOR)

        img_height, img_width, _ = img.shape
        img_cropped = img[img_height - 40:img_height][1:img_width]
        img_hsv = cv.cvtColor(img_cropped, cv.COLOR_BGR2HSV)
        blue_range = np.array([[100, 60, 60], [129, 255, 255]])
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

    # Pure pursuit
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
        pure_pursuit = PurePursuit(_waypoints=read_points(path_waypoints), _rate=5)

        # Follow the waypoints
        pure_pursuit.follow_waypoints()
        return True

    def clean_task1(self):
        self.wall_follow.clean_class()

    def clean_task2(self):
        self.clean_class()
        cv.destroyAllWindows()

    def run_statemachine(self):
        state = 1
        while not self.task3_complete:  # Run state machine until task 3 is complete
            if state == 1:
                self.task1()
                if self.odom.Pose.y >= self.TASK1_THRESH:
                    self.clean_task1()
                    state = 2
            elif state == 2:
                self.task2()
                # if self.odom.Pose.x <= 0.8:
                #     self.clean_task2()
                #     state = 3
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


if __name__ == '__main__':
    rospy.init_node('capstone', anonymous=True)

    # Choose simulation mode else the custom Cristian Valadez room setup mode
    sim_mode = False
    if sim_mode:
        capstone = Capstone(_sim_mode=True, wall_distance=0.5, wall_follow_speed=0.22, collision_distance=0.8,
                            task1_thresh=4.0)
    else:
        capstone = Capstone(_sim_mode=False, wall_distance=0.2, wall_follow_speed=0.12, collision_distance=0.6,
                            task1_thresh=0.7)


    def shutdownhook():
        rospy.loginfo("Shutdown time!")
        if not capstone.task3_complete:
            capstone.clean_class()


    rospy.on_shutdown(shutdownhook)
    capstone.run_statemachine()
