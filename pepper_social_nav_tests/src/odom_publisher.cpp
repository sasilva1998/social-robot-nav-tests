#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time = ros::Time::now();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = msg->pose.pose.position.x;
    odom_trans.transform.translation.y = msg->pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;

    odom_trans.transform.rotation.x = msg->pose.pose.orientation.x;
    odom_trans.transform.rotation.y = msg->pose.pose.orientation.y;
    odom_trans.transform.rotation.z = msg->pose.pose.orientation.z;
    odom_trans.transform.rotation.w = msg->pose.pose.orientation.w;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;

    tf::TransformBroadcaster odom_broadcaster;

    ros::Subscriber sub = n.subscribe("/pepper/odom_groundtruth", 1000, odomCallback);

    ros::spin();

    return 0;
}
