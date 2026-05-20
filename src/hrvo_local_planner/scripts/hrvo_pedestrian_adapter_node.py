#!/usr/bin/env python3
import math

import rospy
from pedsim_msgs.msg import AgentStates
from people_msgs.msg import People, Person


class HrvoPedestrianAdapterNode:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/pedsim_simulator/simulated_agents"
        )
        self.default_radius = float(rospy.get_param("~default_radius", 0.3))
        self.log_every_n = int(rospy.get_param("~log_every_n", 10))
        self.output_people_topic = rospy.get_param("~output_people_topic", "/people")
        self._msg_count = 0
        self._latest_hrvo_agents = []

        self.people_pub = rospy.Publisher(
            self.output_people_topic, People, queue_size=10
        )

        self.sub = rospy.Subscriber(
            self.input_topic, AgentStates, self._on_agent_states, queue_size=1
        )

        rospy.loginfo(
            "hrvo_pedestrian_adapter_node subscribed to %s, publishing %s (default_radius=%.2f)",
            self.input_topic,
            self.output_people_topic,
            self.default_radius,
        )

    def _on_agent_states(self, msg):
        self._msg_count += 1
        stamp = msg.header.stamp.to_sec()
        hrvo_agents = []
        people_msg = People()
        people_msg.header = msg.header

        for agent in msg.agent_states:
            vx = agent.twist.linear.x
            vy = agent.twist.linear.y
            heading = math.atan2(vy, vx) if (abs(vx) > 1e-6 or abs(vy) > 1e-6) else 0.0

            hrvo_agents.append(
                {
                    "agent_id": int(agent.id),
                    "x": float(agent.pose.position.x),
                    "y": float(agent.pose.position.y),
                    "vx": float(vx),
                    "vy": float(vy),
                    "radius": self.default_radius,
                    "heading": float(heading),
                    "timestamp": float(stamp),
                    "social_state": str(agent.social_state),
                }
            )

            person = Person()
            person.name = "ped_{}".format(int(agent.id))
            person.position.x = float(agent.pose.position.x)
            person.position.y = float(agent.pose.position.y)
            person.position.z = float(agent.pose.position.z)
            person.velocity.x = float(vx)
            person.velocity.y = float(vy)
            person.velocity.z = 0.0
            person.reliability = 1.0
            person.tags = [str(agent.social_state)]
            people_msg.people.append(person)

        self._latest_hrvo_agents = hrvo_agents
        self.people_pub.publish(people_msg)

        if self._msg_count % max(1, self.log_every_n) == 0:
            rospy.loginfo(
                "HRVO adapter got %d pedestrians @ %.3f",
                len(hrvo_agents),
                stamp,
            )
            for ped in hrvo_agents[:5]:
                rospy.loginfo(
                    "id=%d pos=(%.2f, %.2f) vel=(%.2f, %.2f) r=%.2f heading=%.2f social=%s",
                    ped["agent_id"],
                    ped["x"],
                    ped["y"],
                    ped["vx"],
                    ped["vy"],
                    ped["radius"],
                    ped["heading"],
                    ped["social_state"],
                )


def main():
    rospy.init_node("hrvo_pedestrian_adapter_node")
    HrvoPedestrianAdapterNode()
    rospy.spin()


if __name__ == "__main__":
    main()
