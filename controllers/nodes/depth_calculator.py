#!/usr/bin/env python
import rospy
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32MultiArray
from controllers.msg import config_msg


class DepthEstimatorNode:
    def __init__(self):
        self.__window = 10
        self.__filter = self.medianFilter()
        self.__filter.send(None)
        self.__atmosphere = 101325
        self.DEPTH_OFFSET = 0.05
        # Pressure offset in m
        rospy.init_node("depth_calculator")
        depth_pub = rospy.Publisher("depth", Float32MultiArray, queue_size=1)
        rospy.Subscriber("pressure", FluidPressure, self.pressure_callback, depth_pub)
        rospy.Subscriber("controllers_config", config_msg, self.setWindow)

        rospy.spin()

    def pressure_callback(self, pressure_msg, publisher):
        pascal_per_meter = 1.0e4
        # what kind of pressure data do we get? relative/absolute? What about
        # atmospheric pressure?
        rawDepth = (-pressure_msg.fluid_pressure + self.__atmosphere) / pascal_per_meter
        filterdDepth = self.__filter.send(rawDepth) + self.DEPTH_OFFSET
        depth_msg = Float32MultiArray()
        depth_msg.data = [rawDepth, filterdDepth]
        publisher.publish(depth_msg)

    def setWindow(self, message):
        self.__window = message.window
        print("window: {}".format(self.__window))

    def medianFilter(self):
        sampels = []
        filterd_pressure = 0
        while True:
            data = yield filterd_pressure
            sampels.append(data)
            while len(sampels) > self.__window:
                sampels.pop(0)
            total = sum(sampels)
            filterd_pressure = total / len(sampels)


def main():
    DepthEstimatorNode()


if __name__ == "__main__":
    main()
