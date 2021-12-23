// #include <mbf_client.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

#include <fetch_move_base_msgs/Path2D.h>

void solutionPathCallback(const fetch_move_base_msgs::Path2D::ConstPtr &msg)
{
    actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); // will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    actionlib_tutorials::FibonacciGoal goal;
    goal.order = 20;
    ac.sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "path_execute");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/fetch_move_base_planner/fetch_move_base_solution_path", 1000, solutionPathCallback);

    ros::spin();

    return 0;
}
