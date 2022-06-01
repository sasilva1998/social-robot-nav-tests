#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

#include <pepper_move_base_msgs/Path2D.h>

void solutionPathCallback(const pepper_move_base_msgs::Path2D::ConstPtr &msg);
