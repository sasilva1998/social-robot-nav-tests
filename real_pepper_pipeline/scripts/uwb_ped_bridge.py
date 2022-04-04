#!/usr/bin/env python

import rospy
from pedsim_msgs.msg import AgentStates, AgentState

# from geometry_msgs.msg import TransformStamped
from pozyx_bridge.msg import UwbTransformStampedArray


class AgentStatesBroadcaster(object):
    """Publishes Pozyx System tags position as pedsim AgentStates"""

    def __init__(self):

        self.agents_dict = {}
        self.agent_counter = 0

        # PUBLISHERS
        self.agents_pub = rospy.Publisher(
            "pedsim_simulator/simulated_agents", AgentStates, queue_size=10
        )

        # SUBSCRIBERS
        self.pozyx_sub = rospy.Subscriber(
            "uwb_sensor", UwbTransformStampedArray, self.agents_register_callback
        )

    def agents_register_callback(self, data):

        for i in data.transforms_array:
            if i.transform.child_frame_id[1:] not in self.agents_dict:
                self.agent_counter += 1
                new_agent = AgentState()
                new_agent.header.stamp = i.transform.header.stamp
                new_agent.header.frame_id = i.transform.header.frame_id
                new_agent.id = self.agent_counter

                new_agent.pose.position.x = i.transform.transform.translation.x
                new_agent.pose.position.y = i.transform.transform.translation.y
                new_agent.pose.position.z = i.transform.transform.translation.z

                new_agent.pose.orientation = i.transform.transform.rotation

                self.agents_dict[i.transform.child_frame_id] = new_agent
            else:
                agent_update = self.agents_dict[i.transform.child_frame_id]
                agent_update.header.stamp = i.transform.header.stamp
                agent_update.pose.position.x = i.transform.transform.translation.x
                agent_update.pose.position.y = i.transform.transform.translation.y
                agent_update.pose.position.z = i.transform.transform.translation.z

                agent_update.pose.orientation = i.transform.transform.rotation

                self.agents_dict[i.transform.child_frame_id] = agent_update

    def run(self):
        while not rospy.is_shutdown():
            agent_states = AgentStates()
            agent_states_list = []

            for i in self.agents_dict.items():
                agent_states_list.append(i[1])

            agent_states.header.stamp = rospy.Time.now()
            agent_states.header.frame_id = rospy.get_param("~frame_id", "world")
            agent_states.agent_states = agent_states_list
            self.agents_pub.publish(agent_states)
            rospy.sleep(0.05)


if __name__ == "__main__":
    rospy.init_node("pozyx_to_ped")
    ped_broadcaster = AgentStatesBroadcaster()
    ped_broadcaster.run()
