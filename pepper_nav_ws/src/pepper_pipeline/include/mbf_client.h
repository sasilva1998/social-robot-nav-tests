#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

#include <fetch_move_base_msgs/Path2D.h>

void solutionPathCallback(const fetch_move_base_msgs::Path2D::ConstPtr &msg);
