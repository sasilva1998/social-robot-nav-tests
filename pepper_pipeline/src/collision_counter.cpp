#include "ros/ros.h"
#include <cstdlib>

// octomap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

// fcl
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/octree.h>
#include <fcl/traversal/traversal_node_octree.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>
#include <fcl/math/transform.h>
#include <fcl/collision.h>
#include <fcl/collision_node.h>
#include <fcl/distance.h>

#include <nav_msgs/Odometry.h>
#include <pedsim_msgs/AgentStates.h>
#include <std_msgs/Int32.h>

#include <stdio.h>

// ROS-Octomap interface
using octomap_msgs::GetOctomap;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_detector_node");
    ros::NodeHandle n;

    double fetch_base_radius_ = 0.20;
    double fetch_base_height_ = 1.22;

    std::shared_ptr<fcl::Cylinder> fetch_collision_solid_;
    std::shared_ptr<fcl::Cylinder> agent_collision_solid_;

    fetch_collision_solid_.reset(new fcl::Cylinder(fetch_base_radius_, fetch_base_height_));
    agent_collision_solid_.reset(new fcl::Cylinder(0.35, fetch_base_height_));

    GetOctomap::Request req;
    GetOctomap::Response resp;

    ros::Rate loop_rate(10);

    bool inCollision = false;
    int collisionCounter = 0;

    ros::Publisher collisionCounterPub = n.advertise<std_msgs::Int32>("collision_counter", 1000);

    fcl::CollisionRequest collision_request;
    fcl::CollisionResult collision_result_octomap;

    fcl::CollisionResult collision_result;

    while (ros::ok())
    {
        ros::service::call("/octomap_binary", req, resp);

        octomap::AbstractOcTree *abs_octree_ = octomap_msgs::msgToMap(resp.map);

        nav_msgs::OdometryConstPtr odomData = ros::topic::waitForMessage<nav_msgs::Odometry>("/pepper/odom_groundtruth");

        pedsim_msgs::AgentStatesConstPtr agentStates = ros::topic::waitForMessage<pedsim_msgs::AgentStates>("/pedsim_simulator/simulated_agents_overwritten");

        if (abs_octree_)
        {
            octomap::OcTree *octree_ = dynamic_cast<octomap::OcTree *>(abs_octree_);
            fcl::OcTree *tree_ = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(octree_));
            fcl::CollisionObject *tree_obj_ = new fcl::CollisionObject((std::shared_ptr<fcl::CollisionGeometry>(tree_)));

            fcl::Transform3f fetch_tf;
            fetch_tf.setIdentity();
            fetch_tf.setTranslation(fcl::Vec3f(odomData->pose.pose.position.x, odomData->pose.pose.position.y, fetch_base_height_ / 2.0));
            fcl::Quaternion3f qt0;
            qt0.fromEuler(0.0, 0.0, 0.0);
            fetch_tf.setQuatRotation(qt0);

            fcl::CollisionObject vehicle_co(fetch_collision_solid_, fetch_tf);

            fcl::collide(tree_obj_, &vehicle_co, collision_request, collision_result_octomap);

            if (collision_result_octomap.isCollision() & !inCollision)
            {
                collisionCounter++;
                inCollision = true;
            }
            else if (!collision_result_octomap.isCollision())
            {
                inCollision = false;
            }
        }

        // if (!abs_octree_ || !collision_result_octomap.isCollision())
        // {
        //     for (int i = 0; i < agentStates->agent_states.size(); i++)
        //     {
        //         fcl::Transform3f fetch_tf;
        //         fetch_tf.setIdentity();
        //         fetch_tf.setTranslation(fcl::Vec3f(odomData->pose.pose.position.x, odomData->pose.pose.position.y, fetch_base_height_ / 2.0));
        //         fcl::Quaternion3f qt0;
        //         qt0.fromEuler(0.0, 0.0, 0.0);
        //         fetch_tf.setQuatRotation(qt0);

        //         fcl::CollisionObject vehicle_co(fetch_collision_solid_, fetch_tf);

        //         pedsim_msgs::AgentState agentState = agentStates->agent_states[i];

        //         double dRobotAgent =
        //             std::sqrt(std::pow(agentState.pose.position.x - odomData->pose.pose.position.x, 2) +
        //                       std::pow(agentState.pose.position.y - odomData->pose.pose.position.y, 2));

        //         if (dRobotAgent < 2)
        //         {
        //             // FCL
        //             fcl::Transform3f agent_tf;
        //             agent_tf.setIdentity();
        //             agent_tf.setTranslation(
        //                 fcl::Vec3f(agentState.pose.position.x, agentState.pose.position.y, fetch_base_height_ / 2.0));
        //             fcl::Quaternion3f qt0;
        //             qt0.fromEuler(0.0, 0.0, 0.0);
        //             agent_tf.setQuatRotation(qt0);

        //             fcl::CollisionObject agent_co(agent_collision_solid_, agent_tf);
        //             fcl::collide(&agent_co, &vehicle_co, collision_request, collision_result);

        //             if (collision_result_octomap.isCollision() & !inCollision)
        //             {
        //                 collisionCounter++;
        //                 inCollision = true;
        //             }
        //             else if (!collision_result_octomap.isCollision())
        //             {
        //                 inCollision = false;
        //             }
        //         }
        //     }
        // }

        std_msgs::Int32 collisionCounterMsg;
        collisionCounterMsg.data = collisionCounter;

        collisionCounterPub.publish(collisionCounterMsg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}