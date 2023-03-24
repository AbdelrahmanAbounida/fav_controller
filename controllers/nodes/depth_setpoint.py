#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import math
from controllers.msg import config_msg


class DepthSetpointNode:
    def __init__(self):
        rospy.init_node("depth_setpoint_publisher")

        self.start_time = rospy.get_time()

        # change these parameters to adjust setpoint(s)
        # either constant setpoint (setpoint_1), or jumping between two
        # setpoints
        self.constant = False  # constant or square wave?
        self.duration = 15  # in s
        self.setpoint_1 = -0.5
        self.setpoint_2 = -0.6
        self.oscillating_center = -0.35
        self.mode = 0
        self.frequency = 1
        self.amplitude = 0.175
        self.oscillating_center = -0.45

        self.config_sub = rospy.Subscriber("controllers_config", config_msg, self.setConfig)

        self.setpoint_pub = rospy.Publisher("depth_setpoint", Float64, queue_size=1)

    def get_setpoint(self):
        if self.mode == 0:
            setpoint = self.setpoint_1

        elif self.mode == 1:
            now = rospy.get_time()
            time = self.start_time - now

            i = time % (self.duration * 2)
            if i > (self.duration):
                setpoint = self.setpoint_1
            else:
                setpoint = self.setpoint_2
        elif self.mode == 2:
            setpoint = self.oscillating_center - self.amplitude * math.sin(
                rospy.get_time() - self.start_time * 2 * math.pi * self.frequency
            )

        self.publish_setpoint(setpoint)

    def publish_setpoint(self, setpoint):
        msg = Float64()
        msg.data = setpoint
        self.setpoint_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.get_setpoint()
            rate.sleep()

    def setConfig(self, message):
        self.mode = message.mode
        self.setpoint_1 = message.setpoint1
        self.setpoint_2 = message.setpoint2
        self.frequency = message.frequency
        self.amplitude = message.amplitude
        self.oscillating_center = message.osc_point

        # print(
        #     "mode: {}\nsetpoint1: {}\nsetpoint2: {}"
        #     "\nfrequency: {}\namplitude: {}"
        #     "\ncenter of oscillation: {}---".format(
        #         self.mode,
        #         self.setpoint_1,
        #         self.setpoint_2,
        #         self.frequency,
        #         self.amplitude,
        #         self.oscillating_center,
        #     )
        # )


def main():
    node = DepthSetpointNode()
    node.run()


if __name__ == "__main__":
    main()
