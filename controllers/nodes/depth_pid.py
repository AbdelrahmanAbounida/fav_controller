#!/usr/bin/env python3
import rospy  # this is the python interface for ROS
from std_msgs.msg import Float64, Float32MultiArray
from controllers.msg import config_msg
from localization.msg import customPos
import numpy as np


class DepthController:
    def __init__(self):
        self.__p_gain = 1.2
        self.__i_gain = 0.2
        self.__d_gain = 0.2
        self.__p = 1.2
        self.__i = 0.2
        self.__d = 0.2
        self.__aw_gain = 0.0
        self.__setpoint = 0.0
        self.output_constraint = 1
        self.__integrator_active = True
        self.__controller_active = False
        self.__integral_lower_limit = 0.0
        self.__integral_upper_limit = 0.0
        self.__depth = 0.0
        self.__output = 0.0
        rospy.init_node("depth_controller")
        rospy.Subscriber("depth", Float32MultiArray, self.getDepth)
        rospy.Subscriber("target_position", customPos, self.__setSetpoint)
        self.vertical_thrust_pub = rospy.Publisher(
            "vertical_thrust", Float64, queue_size=1
        )

        self.control_effort = rospy.Publisher(
            "depth_control_effort", Float32MultiArray, queue_size=1
        )

    def run(self):
        rate = rospy.Rate(30)
        controller = self.__PID()
        while not rospy.is_shutdown():

            self.__output = next(controller)
            # output überwachen falls Bluerov zu tief oder flach taucht
            # controller abstellen.

            if self.__depth < -0.8 or self.__depth > -0.1:
                self.__output = 0

            msg = Float64()
            effort = Float32MultiArray()
            effort.data = [self.__p, self.__i, self.__d]

            msg.data = self.__output
            self.vertical_thrust_pub.publish(msg)
            self.control_effort.publish(effort)
            rate.sleep()

    def __setSetpoint(self, setpoint):
        setpoint.z = contstrain(setpoint.z, -0.1, -1.1)
        if np.abs(self.__setpoint - setpoint.z) > 0.02:
            self.__i = 0
            self.__setpoint = setpoint.z

    def getDepth(self, depth):

        self.__depth = depth.data[1]

    # def setConfig(self, message):
    #     self.__p_gain = message.pz_gain
    #     self.__i_gain = message.iz_gain
    #     self.__d_gain = message.dz_gain
    #     self.__aw_gain = message.aw_gain
    #     self.__integral_lower_limit = message.integral_lower_limit
    #     self.__integral_upper_limit = message.integral_upper_limit

    #     if self.__integrator_active is not message.integrator_active:
    #         self.__integrator_active = message.integrator_active
    #         self.resetController(False)
    #     if self.__controller_active is not message.controller_active:
    #         self.__controller_active = message.controller_active
    #         self.resetController(True)

    #     print(
    #         "p_gain: {}\ni_gain: {}\nd_gain: {}"
    #         "\nintegral_lower_limit: {}\nintegral_upper_limit: {}"
    #         "\nintegrator_active: {}\ncontroller_active: {}\n---".format(
    #             self.__p_gain,
    #             self.__i_gain,
    #             self.__d_gain,
    #             self.__integral_lower_limit,
    #             self.__integral_upper_limit,
    #             self.__integrator_active,
    #             self.__controller_active,
    #         )
    #     )

    def __PID(self):

        # initialize stored data
        ed_prev = 0
        t_prev = rospy.get_time()

        output = 0
        aw_out = 0
        while True:
            yield output
            time = rospy.get_time()

            # PID calculations
            e = self.__setpoint - self.__depth

            self.__p = self.__p_gain * e
            if time - t_prev > 0:
                self.__i = self.__i + self.__i_gain * (e - aw_out * self.__aw_gain) * (
                    time - t_prev)
                self.__d = self.__d_gain * (e - ed_prev) / (time - t_prev)

            if self.__integrator_active is False:
                self.__i = 0

            self.__i = contstrain(
                self.__i, self.__integral_upper_limit, self.__integral_lower_limit
            )

            output = self.__p + self.__i + self.__d
            aw_out = output
            output = contstrain(output, self.output_constraint, -self.output_constraint)
            aw_out -= output
            if self.__controller_active is False:
                output = 0

            # update stored data for next iteration
            ed_prev = e
            t_prev = time

    def resetController(self, all):
        self.__i = 0
        if all is True:
            self.__d = 0
            self.__p = 0
            self.__output = 0


def contstrain(value, upper, lower):
    if value < lower:
        value = lower
    if value > upper:
        value = upper
    return value


def main():
    print( rospy.get_param("/bluerov/stations"))
    node = DepthController()
    node.run()



if __name__ == "__main__":
    main()
