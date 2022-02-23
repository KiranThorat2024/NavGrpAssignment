#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def movebase_client():
    # Create action client and wait for it to startup
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Create a goal with MoveBaseGoal constructor
    # Move along x and y axis
    # No rotation of mobile base frame w.r.t. map frame
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 2.5
    goal.target_pose.pose.position.y = 1.75
    goal.target_pose.pose.orientation.w = 1.0

    # Send goal to action server and wait for server to finish action
    client.send_goal(goal)
    wait = client.wait_for_result()

    # Check result arrival
    if wait:
        return client.get_result()
    else:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")

    return True


if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
