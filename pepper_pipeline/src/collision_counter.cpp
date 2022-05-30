#include "ros/ros.h"
#include <cstdlib>

// octomap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap/OcTree.h>

// fcl
#include <fcl/fcl.h>
#include <fcl/collision.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/broadphase/broadphase_spatialhash.h>

#include "fcl/common/types.h"
#include "fcl/config.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/math/geometry-inl.h"

#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/collision_request.h"
#include "fcl/narrowphase/collision_result.h"

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

    std::shared_ptr<fcl::Cylinder<float>> fetch_collision_solid_;
    std::shared_ptr<fcl::Cylinder<float>> agent_collision_solid_;

    fetch_collision_solid_.reset(new fcl::Cylinder<float>(fetch_base_radius_, fetch_base_height_));
    agent_collision_solid_.reset(new fcl::Cylinder<float>(0.35, fetch_base_height_));

    GetOctomap::Request req;
    GetOctomap::Response resp;

    ros::Rate loop_rate(10);

    bool inCollision = false;
    int collisionCounter = 0;

    ros::Publisher collisionCounterPub = n.advertise<std_msgs::Int32>("collision_counter", 1000);

    fcl::CollisionRequestf collision_request;
    fcl::CollisionResultf collision_result_octomap;


    fcl::CollisionResult<float> collision_result;

    std:bool foundCollision = false;

    while (ros::ok())
    {
        ros::service::call("/octomap_binary", req, resp);

        octomap::AbstractOcTree *abs_octree_ = octomap_msgs::msgToMap(resp.map);

        nav_msgs::OdometryConstPtr odomData = ros::topic::waitForMessage<nav_msgs::Odometry>("/pepper/odom_groundtruth");

        pedsim_msgs::AgentStatesConstPtr agentStates = ros::topic::waitForMessage<pedsim_msgs::AgentStates>("/pedsim_simulator/simulated_agents_overwritten");

        if (abs_octree_)
        {

            octomap::OcTree *octree_ = dynamic_cast<octomap::OcTree *>(abs_octree_);
            
            fcl::OcTree<float> *tree_ = new fcl::OcTree<float>(std::shared_ptr<const octomap::OcTree>(octree_));

            fcl::CollisionObject<float> *tree_obj_= new fcl::CollisionObject<float>((std::shared_ptr<fcl::CollisionGeometry<float>>(tree_)));

            fcl::CollisionObject<float> vehicle_co(fetch_collision_solid_); 

            vehicle_co.setTranslation(fcl::Vector3f(odomData->pose.pose.position.x, odomData->pose.pose.position.y, fetch_base_height_ / 2.0));

            fcl::collide(tree_obj_, &vehicle_co, collision_request, collision_result_octomap);

            if (collision_result_octomap.isCollision() & !inCollision)
            {
                collisionCounter++;
                inCollision = true;
            }
            else if (!collision_result_octomap.isCollision() & !foundCollision)
            {
                inCollision = false;
            }
        }

        if (!abs_octree_ || !collision_result_octomap.isCollision())
        {

            foundCollision = false;

            for (int i = 0; i < agentStates->agent_states.size(); i++)
            {

                if (!foundCollision)
                {

                    fcl::CollisionObject<float> vehicle_co(fetch_collision_solid_);

                    vehicle_co.setTranslation(fcl::Vector3f(odomData->pose.pose.position.x, odomData->pose.pose.position.y, fetch_base_height_ / 2.0));

                    pedsim_msgs::AgentState agentState = agentStates->agent_states[i];

                    double dRobotAgent =
                        std::sqrt(std::pow(agentState.pose.position.x - odomData->pose.pose.position.x, 2) +
                                std::pow(agentState.pose.position.y - odomData->pose.pose.position.y, 2));

                    if (dRobotAgent < 2)
                    {
                        // FCL

                        fcl::CollisionObject<float> agent_co(agent_collision_solid_);

                        agent_co.setTranslation(fcl::Vector3f(agentState.pose.position.x, agentState.pose.position.y, fetch_base_height_ / 2.0));
                        fcl::collide(&agent_co, &vehicle_co, collision_request, collision_result);

                        if (collision_result.isCollision())
                        {
                            // collisionCounter++;
                            // inCollision = true;
                            foundCollision = true;
                        }
                        // else if (!collision_result.isCollision())
                        // {
                        //     inCollision = false;
                        // }
                    }
                }         
            }

            if (foundCollision & !inCollision)
            {
                collisionCounter++;
                inCollision = true;
            }
            else if (!foundCollision)
            {
                inCollision = false;
            }
            
        }

        std_msgs::Int32 collisionCounterMsg;
        collisionCounterMsg.data = collisionCounter;

        collisionCounterPub.publish(collisionCounterMsg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}