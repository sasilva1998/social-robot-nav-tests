#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

class FakePosePublisher:

    def __init__(self):
        
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)


    def publish_origin(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.pose.position.x = 0
        pose_msg.pose.pose.position.y = 0
        pose_msg.pose.pose.position.z = 0

        pose_msg.pose.pose.orientation.x=0
        pose_msg.pose.pose.orientation.y=0
        pose_msg.pose.pose.orientation.z=0
        pose_msg.pose.pose.orientation.w=0

        self.pose_pub.publish(pose_msg)


if __name__ == "__main__":
    rospy.init_node("dummy_hololens_pose")
    dummy_pub = FakePosePublisher()

    while not rospy.is_shutdown():
        dummy_pub.publish_origin()
        rospy.sleep(2.5)
