// #include <mbf_client.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include <mbf_msgs/ExePathAction.h>
#include <fetch_move_base_msgs/Path2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>

#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float32.h>

// std_msgs::Float32 dist_tolerance = 0.2;
// std_msgs::Float32 angle_tolerance = 0.2;

void solutionPathCallback(const fetch_move_base_msgs::Path2D::ConstPtr &msg)
{
    actionlib::SimpleActionClient<mbf_msgs::ExePathAction> ac("/move_base_flex/exe_path", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); // will wait for infinite time
    ac.cancelAllGoals();

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    // actionlib_tutorials::FibonacciGoal goal;

    // geometry_msgs::Pose pose;

    geometry_msgs::PoseStamped pose;

    // geometry_msgs::PoseStamped poseStampedArray[sizeof(msg->waypoints)];

    nav_msgs::Path pathPoses;

    mbf_msgs::ExePathGoal goal;

    // ROS_INFO("Waypoints size: " << sizeof(msg->waypoints) << ".");

    ROS_INFO_STREAM("Size of waypoints: " << sizeof(msg->waypoints) / sizeof(msg->waypoints[0]) + 1 << ".");

    for (int i = 0; i < sizeof(msg->waypoints) / sizeof(msg->waypoints[0]) + 1; i++)
    {

        ROS_INFO_STREAM("X position added" << msg->waypoints[i].x << ".");
        ROS_INFO_STREAM("Y position added" << msg->waypoints[i].y << ".");
        pose.pose.position.x = msg->waypoints[i].x;
        pose.pose.position.y = msg->waypoints[i].y;

        tf2::Quaternion convertedQuaternion;

        convertedQuaternion.setRPY(0, 0, msg->waypoints[i].theta);

        convertedQuaternion = convertedQuaternion.normalize();

        pose.header.frame_id = "world";

        pose.pose.orientation.x = convertedQuaternion.getX();
        pose.pose.orientation.y = convertedQuaternion.getY();
        pose.pose.orientation.z = convertedQuaternion.getZ();
        pose.pose.orientation.w = convertedQuaternion.getW();

        pathPoses.header.frame_id = "world";
        pathPoses.poses.push_back(pose);

        // poseStamped.pose = pose;

        // poseStampedArray[i] = poseStamped;
    }
    // ROS_INFO_STREAM("Poses added" << poseStampedArray << ".");

    // goal.goal.path = pathPoses;

    // pathPoses.poses = poseStampedArray;

    goal.path = pathPoses;
    // goal.header.frame_id = "world";
    goal.dist_tolerance = 0.2;
    goal.angle_tolerance = 0.2;

    // goal.order = 20;
    ac.sendGoal(goal);

    // wait for the action to return
    // bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    // if (finished_before_timeout)
    // {
    //     actionlib::SimpleClientGoalState state = ac.getState();
    //     ROS_INFO("Action finished: %s", state.toString().c_str());
    // }
    // else
    //     ROS_INFO("Action did not finish before the time out.");
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "path_execute");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/fetch_move_base_planner/fetch_move_base_solution_path", 1000, solutionPathCallback);

    ros::spin();

    return 0;
}
