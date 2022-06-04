#! /usr/bin/env python3

import rospy
import actionlib
import time
from psmm_drive.msg import PSMMDriveAction, PSMMDriveGoal
from nav_msgs.msg import OccupancyGrid

def psmm_drive_client():

    time.sleep(10)
    client = actionlib.SimpleActionClient("psmm_drive_node", PSMMDriveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = PSMMDriveGoal()

    goal.goal.x = -12.0
    goal.goal.y = 12.5

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
        rospy.init_node("psmm_drive_client_py")
        rospy.wait_for_message("/projected_map", OccupancyGrid)
        result = psmm_drive_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
