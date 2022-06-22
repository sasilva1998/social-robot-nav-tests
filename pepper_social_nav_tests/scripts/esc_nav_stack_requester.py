#! /usr/bin/env python3

import rospy
import actionlib
import time
from esc_move_base_msgs.msg import Goto2DAction, Goto2DGoal
from sensor_msgs.msg import PointCloud2

def esc_nav_stack_client():

    time.sleep(10)
    client = actionlib.SimpleActionClient("/esc_goto_action", Goto2DAction)

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
        rospy.init_node("esc_nav_stack_client_py")
        rospy.wait_for_message("/pepper/camera/depth/points", PointCloud2)
        result = esc_nav_stack_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
