#! /usr/bin/env python3

import rospy
import actionlib
import time

# from sfm_diff_drive.msg import SFMDriveAction, SFMDriveGoal
from esc_move_base_msgs.msg import Goto2DAction, Goto2DGoal


def social_rrt_client():

    time.sleep(10)
    client = actionlib.SimpleActionClient("/pepper_goto_action", Goto2DAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = Goto2DGoal()

    goal.goal.x = -12
    goal.goal.y = 12

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult


if __name__ == "__main__":
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node("social_rrt_client_py")
        result = social_rrt_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
