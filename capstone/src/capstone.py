#!/usr/bin/env python3
import rospkg
import rospy
from cv_bridge import CvBridge
import cv2 as cv
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
from pure_pursuit_tb import PurePursuit, read_points
from wall_follow_tb import WallFollow
from classify_resnet50_capstone import ClassifyResnet50


class Capstone(object):
    def __init__(self, start_on_state, _sim_mode, wall_distance, wall_follow_speed, collision_distance,
                 task1_thresh, task2_thresh, waypoints_fpath, pp_normalize, pp_lookahead, pp_max_vel):
        # Variables and objects
        self.state = start_on_state
        self.odom = self.CurrOdom()
        self.camRawData = None
        self.task3_complete = False
        self.bridge = CvBridge()
        self.twist_object = Twist()
        self.classify = ClassifyResnet50()
        self.wall_follow = None
        self.dist_traveled_x = 0.0
        self.dist_traveled_y = 0.0
        self.init_pose_x = None
        self.init_pose_y = None
        self.traffic_light_found_flag = False

        # Constants
        self.freq = 20
        self.fixed_linear_speed = 0.12
        self.angular_rate_max = 2.84
        self.kp_task2 = 0.005
        self.wall_distance = wall_distance
        self.wall_follow_speed = wall_follow_speed
        self.collision_distance = collision_distance
        self.sim_mode = _sim_mode
        self.TASK1_THRESH = task1_thresh
        self.TASK2_THRESH = task2_thresh
        self.waypoints_fpath = waypoints_fpath
        self.PP_NORMALIZE = pp_normalize
        self.PP_LOOKAHEAD = pp_lookahead
        self.PP_MAX_VEL = pp_max_vel

        # Subscribers, publishers, topics, etc
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)  # Odometry pose subscriber
        if sim_mode:  # Camera subscriber
            self.cam_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.cam_callback)
        else:
            self.cam_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.cam_callback)
        self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(self.freq)
        self.cmdvel_pub_rate = rospy.Rate(10)

        # Wait until we get at least one data back from callbacks
        while self.odom.Pose.x is None or self.camRawData is None:
            print("Waiting for callbacks to get at least one data back")
            rospy.Rate(2).sleep()

    # Odometry callback
    def odom_callback(self, _data):
        # Get the initial Pose
        if self.init_pose_x is None:
            self.init_pose_x = _data.pose.pose.position.x
            self.init_pose_y = _data.pose.pose.position.y

        # Position data zeroed out
        self.odom.Pose.x = _data.pose.pose.position.x - self.init_pose_x
        self.odom.Pose.y = _data.pose.pose.position.y - self.init_pose_y

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
        self.wall_follow.follow_the_wall()

    # Line following
    def task2(self):
        # Manipulate image and get the mask of blue line
        if self.sim_mode:
            img = self.bridge.imgmsg_to_cv2(self.camRawData, desired_encoding='bgr8')
        else:
            # img = cv.imdecode(np.fromstring(self.camRawData.data, np.uint8), cv.IMREAD_COLOR)
            img = cv.imdecode(np.frombuffer(self.camRawData.data, np.uint8), cv.IMREAD_COLOR)

        img_height, img_width, _ = img.shape
        img_cropped = img[img_height - 40:img_height][0:img_width]
        img_cropped_square = img[40:img_height - 40,
                                 int(img_width / 2 - img_height / 2 - 80):int(img_width / 2 + img_height / 2 - 80)]
        img_cropped_square = cv.resize(img_cropped_square, (224, 224))
        img_hsv = cv.cvtColor(img_cropped, cv.COLOR_BGR2HSV)
        blue_range = np.array([[95, 60, 60], [129, 255, 255]])
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
        # cv.imshow("Vertical Cropped", img_cropped_square)
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

        # First look for a traffic light and stop for 3 seconds if found
        if not self.traffic_light_found_flag:
            prediction = self.classify.classify(img_cropped_square)
            print("Prediction: {}".format(prediction))
            if self.classify.find_traffic_light(prediction[0][0][1]):
                rospy.loginfo("Stopping at traffic light for 3 seconds")
                self.traffic_light_found_flag = True
                self.stop_robot()
                self.wait_sec(3)

        self.cmdvel_pub.publish(self.twist_object)
        print("**Task2: Err: {:.2f} AngRate: {:.2f}".format(err, angular_rate))
        cv.waitKey(1)
        self.rate.sleep()

    # Pure pursuit
    def task3(self):
        print("***Task3: Running pure pursuit ...")
        # Get the waypoints
        print("Extracting waypoints...")
        rpkg = rospkg.RosPack()
        if self.sim_mode:
            path_waypoints = rpkg.get_path('capstone') + self.waypoints_fpath
        else:
            path_waypoints = rpkg.get_path('capstone') + self.waypoints_fpath

        # Run pure pursuit code
        pure_pursuit = PurePursuit(_waypoints=read_points(path_waypoints), _rate=5, _normalize=self.PP_NORMALIZE,
                                   _lookahead=self.PP_LOOKAHEAD, _max_vel=self.PP_MAX_VEL)

        # Follow the waypoints
        pure_pursuit.follow_waypoints()
        return True

    def clean_task1(self):
        rospy.loginfo("Task1 complete, zero-ing a new pose and waiting 3 seconds")
        self.wall_follow.clean_class()

        # Cause a trigger to re-zero the pose measurement
        self.init_pose_x = None
        self.init_pose_y = None

        # Stop for 3 seconds
        self.wait_sec(3)

    def clean_task2(self):
        rospy.loginfo("Task2 complete, zero-ing a new pose and waiting 3 seconds")
        self.stop_robot()
        rospy.loginfo("Unregistering cam_sub")
        self.cam_sub.unregister()
        rospy.loginfo("Unregistering odom_sub")
        self.odom_sub.unregister()
        # rospy.loginfo("Unregistering cmdvel_vel")
        # self.cmdvel_pub.unregister()

        # Cause a trigger to re-zero the pose measurement
        self.init_pose_x = None
        self.init_pose_y = None

        # Stop for 3 seconds
        self.wait_sec(3)
        cv.destroyAllWindows()

    def run_statemachine(self):
        while not self.task3_complete:  # Run state machine until task 3 is complete
            if self.state == 1:
                self.task1()
                print("Zeroed Pose: ({:.2f}, {:.2f}))".
                      format(self.odom.Pose.x, self.odom.Pose.y))
                if self.sim_mode:
                    if self.odom.Pose.y >= self.TASK1_THRESH:
                        self.clean_task1()
                        self.state = 2
                else:
                    if self.odom.Pose.y >= self.TASK1_THRESH:
                        self.clean_task1()
                        self.state = 2
            elif self.state == 2:
                print("Zeroed Pose: ({:.2f}, {:.2f}))".
                      format(self.odom.Pose.x, self.odom.Pose.y))
                self.task2()
                if self.odom.Pose.x <= self.TASK2_THRESH:
                    self.clean_task2()
                    self.state = 3
            elif self.state == 3:
                self.task3_complete = self.task3()

            self.rate.sleep()

    def stop_robot(self):
        # Stop robot with this shutdown hook
        self.twist_object.linear.x = 0.0
        self.twist_object.angular.z = 0.0
        self.cmdvel_pub.publish(self.twist_object)

    # Wait for n amount of seconds
    def wait_sec(self, n):
        for i in range(self.freq * n):
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('capstone', anonymous=True)

    # Choose simulation mode else the custom Cristian Valadez room setup mode
    sim_mode = False
    if sim_mode:
        capstone = Capstone(start_on_state=1, _sim_mode=True, wall_distance=0.5, wall_follow_speed=0.22,
                            collision_distance=0.8, task1_thresh=4.0, task2_thresh=0.8,
                            waypoints_fpath='/waypoints/waypoints.csv', pp_normalize=False,
                            pp_lookahead=0.8, pp_max_vel=0.22)
    else:
        capstone = Capstone(start_on_state=1, _sim_mode=False, wall_distance=0.2, wall_follow_speed=0.12,
                            collision_distance=0.6, task1_thresh=0.5, task2_thresh=-2.5,
                            waypoints_fpath='/waypoints/waypoints_custom.csv', pp_normalize=True,
                            pp_lookahead=0.08, pp_max_vel=0.1)


    def shutdownhook():
        rospy.loginfo("Shutdown time!")
        if not capstone.task3_complete:
            capstone.stop_robot()


    rospy.on_shutdown(shutdownhook)
    capstone.run_statemachine()
