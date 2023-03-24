#!/usr/bin/env python3
import rospy  # this is the python interface for ROS
from std_msgs.msg import Float64, Float32MultiArray
from controllers.msg import config_msg
from finalproject.msg import status_msg
from geometry_msgs.msg import Vector3Stamped, PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import tf
# from path_follower.srv import GenerateWaypoints, GenerateWaypointsRequest


class Controller:

    def __init__(self):

        # Simulation settings
        # self.__p_gain = np.array([0.3, 0.3, 0.5, 1.2, 1.2, 0.1])*0.8

        # self.__i_gain = np.array([0.05, 0.05, 0.2, 0.2, 0.2, 0.05])*0.8

        # self.__d_gain = np.array([0.15, 0.15, 0.2, 0.2, 0.2, 0.2])*0.8

        # self.__p_slow_gain = np.array([0.3, 0.3, 1.5, 1.2, 1.2, 0.2])*0.8

        # self.__i_slow_gain = np.array([0.05, 0.05, 0.2, 0.2, 0.2, 0.05])*0.8

        # self.__d_slow_gain = np.array([0.15, 0.15, 0.2, 0.2, 0.2, 0.1])*0.8

        # self.__aw_gain = np.array([0, 0.25, 0.2, 0.2, 0.2, 0])*0.8

        # Real World settings
        self.__p_gain = np.array([0.3, 0.3, 1.8, 1.2, 1.2, 0.05])

        self.__i_gain = np.array([0.05, 0.05, 0.2, 0.2, 0.2, 0.0])

        self.__d_gain = np.array([0.15, 0.15, 0.00001, 0.2, 0.2, 0])

        self.__p_slow_gain = np.array([0.3, 0.3, 1.5, 1.2, 1.2, 0.3])

        self.__i_slow_gain = np.array([0.05, 0.05, 0.2, 0.2, 0.2, 0.05])

        self.__d_slow_gain = np.array([0.15, 0.15, 0.2, 0.2, 0.2, 0.1])

        self.__aw_gain = np.array([0, 0.25, 0.2, 0.2, 0.2, 0])

        self.__p = np.empty(6)
        self.__i = np.empty(6)
        self.__d = np.empty(6)

        self.ed_prev = np.empty(6)

        # output = np.empty(3)
        self.aw_out = np.empty(6)
        self.e = np.empty(6)
        self.ed_prev = np.empty(6)

        self.__aw_gain = np.empty(6)

        self.__setpoint = [0.3, 0.3, -0.5, 0, 0, 90]
        self.output_constraint = 1
        self.output_slow_constraint = 0.5
        self.__integrator_active = True
        self.__controller_active = False
        self.__integral_lower_limit = 0.0
        self.__integral_upper_limit = 0.0

        self.__pos = np.empty(6)

        self.__output = np.empty(6)

        self.slow = False

        self.station_reached = False
        self.point_reached = False

        rospy.init_node("controller")

        rospy.Subscriber("/bluerov/target_position",
                         PointStamped, self.__setSetpoint)

        rospy.Subscriber("/bluerov/target_angle",
                         PointStamped, self.__setAngle)

        rospy.Subscriber("/bluerov/visual_localization/pose",
                         PoseWithCovarianceStamped, self.getPos)

        rospy.Subscriber("/bluerov/localisation_front",
                         PoseWithCovarianceStamped, self.getfrontalPos)

        rospy.Subscriber("/controllers_config", config_msg, self.setConfig)

        rospy.Subscriber("/bluerov/status", status_msg, self.setStatus)

        self.thrust_pub = rospy.Publisher("thrust", Float64, queue_size=1)
        self.lateral_thrust_pub = rospy.Publisher("lateral_thrust",
                                                  Float64,
                                                  queue_size=1)
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                   Float64,
                                                   queue_size=1)
        self.roll_pub = rospy.Publisher("roll", Float64, queue_size=1)
        self.pitch_pub = rospy.Publisher("pitch", Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher("yaw", Float64, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/bluerov/status", status_msg, queue_size=1)
        self.angle_pub = rospy.Publisher(
            "/bluerov/angle", PointStamped, queue_size=1)

        self.x_control_effort = rospy.Publisher("x_control_effort",
                                                Float32MultiArray,
                                                queue_size=1)
        self.y_control_effort = rospy.Publisher("y_control_effort",
                                                Float32MultiArray,
                                                queue_size=1)
        self.z_control_effort = rospy.Publisher("z_control_effort",
                                                Float32MultiArray,
                                                queue_size=1)
        self.roll_control_effort = rospy.Publisher("roll_control_effort",
                                                   Float32MultiArray,
                                                   queue_size=1)
        self.pitch_control_effort = rospy.Publisher("pitch_control_effort",
                                                    Float32MultiArray,
                                                    queue_size=1)
        self.yaw_control_effort = rospy.Publisher("yaw_control_effort",
                                                  Float32MultiArray,
                                                  queue_size=1)
        self.ooB = False
        self.position = False
        self.rotM = np.zeros(shape=(3, 3))
        self.__rotOutput = np.empty(3)
        self.controlled = False
        self.statusPub()

        # self.generate_waypoints = rospy.ServiceProxy('generate_waypoint_service', GenerateWaypoints)
        # self.ready = false

    def setStatus(self, msg):
        self.slow = msg.slow
        self.resetController()

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.__PID()
            rate.sleep()

    def __publish(self):
        # self.__output = next(controller)
        # output überwachen falls Bluerov zu tief oder flach taucht
        # controller abstellen.
        for element in range(6):

            if element == 0:
                msg = Float64()
                effort = Float32MultiArray()
                effort.data = [
                    self.__p[element], self.__i[element], self.__d[element]
                ]
                msg.data = -self.__output[element]
                # rospy.logwarn(float(msg.data))
                # print(f'msg.data: {msg.data}')
                self.lateral_thrust_pub.publish(msg)
                self.x_control_effort.publish(effort)

            elif element == 1:
                msg = Float64()
                effort = Float32MultiArray()
                effort.data = [
                    self.__p[element], self.__i[element], self.__d[element]
                ]

                msg.data = self.__output[element]
                self.thrust_pub.publish(msg)
                self.y_control_effort.publish(effort)

            elif element == 2:
                if self.__pos[element] < -0.8 or self.__pos[element] > -0.1:
                    self.__output[element] = 0.0

                msg = Float64()
                effort = Float32MultiArray()
                effort.data = [
                    self.__p[element], self.__i[element], self.__d[element]
                ]

                msg.data = self.__output[2]
                #msg.data = self.__rotOutput[2]
                self.vertical_thrust_pub.publish(msg)
                self.z_control_effort.publish(effort)

            elif element == 3:
                msg = Float64()
                effort = Float32MultiArray()
                effort.data = [
                    self.__p[element], self.__i[element], self.__d[element]
                ]

                msg.data = self.__output[element]
                self.roll_pub.publish(msg)
                self.roll_control_effort.publish(effort)

            elif element == 4:
                msg = Float64()
                effort = Float32MultiArray()
                effort.data = [
                    self.__p[element], self.__i[element], self.__d[element]
                ]

                msg.data = self.__output[element]
                self.pitch_pub.publish(msg)
                self.pitch_control_effort.publish(effort)

            elif element == 5:
                msg = Float64()
                effort = Float32MultiArray()
                effort.data = [
                    self.__p[element], self.__i[element], self.__d[element]
                ]

                msg.data = self.__output[element]
                self.yaw_pub.publish(msg)
                self.yaw_control_effort.publish(effort)

    def __setSetpoint(self, setpoint):
        # if self.station_reached is True:
        #     self.__setpoint[5]=0
        self.controlled = False

        if np.abs(self.__setpoint[0] - setpoint.point.x) > 0.02:
            self.__i[0] = 0
            self.__setpoint[0] = setpoint.point.x
        self.__setpoint[0] = contstrain(self.__setpoint[0], 2, 0.01)

        if np.abs(self.__setpoint[1] - setpoint.point.y) > 0.02:
            self.__i[1] = 0
            self.__setpoint[1] = setpoint.point.y
        self.__setpoint[1] = contstrain(self.__setpoint[1], 4, 0.01)

        # setpoint.point.z = contstrain(setpoint.point.z, -0.8, -0.1)
        if np.abs(self.__setpoint[2] - setpoint.point.z) > 0.02:
            self.__i[2] = 0
            self.__setpoint[2] = setpoint.point.z

       #
       # print(self.__setpoint[0])

    def __setAngle(self, setpoint):

        self.__setpoint[3] = setpoint.point.x
        self.__setpoint[4] = setpoint.point.y
        self.__setpoint[5] = setpoint.point.z

   #
   # print(self.__setpoint[0])

    def statusPub(self):
        msg = status_msg()
        msg.station_reached = self.controlled
        self.station_reached = self.controlled
        msg.slow = self.slow
        self.status_pub.publish(msg)

    def getPos(self, pose):
        if self.station_reached is False:

            self.__pos[0] = pose.pose.pose.position.x

            self.__pos[1] = pose.pose.pose.position.y

            self.__pos[2] = pose.pose.pose.position.z

        quaternion = (pose.pose.pose.orientation.x,
                      pose.pose.pose.orientation.y,
                      pose.pose.pose.orientation.z,
                      pose.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.__pos[3] = euler[0] * 180 / np.pi
        self.__pos[4] = euler[1] * 180 / np.pi
        self.__pos[5] = np.abs(euler[2]) * 180 / np.pi
        msg = PointStamped()
        msg.point.x = self.__pos[3]
        msg.point.y = self.__pos[4]
        msg.point.x = self.__pos[5]
        self.angle_pub.publish(msg)
        # if self.__pos[5] < 0:
        #     self.__pos[5] = self.__pos[5] + 360
        # rospy.logwarn(self.__pos[5])

        self.rotM = quaternion_rotation_matrix(pose.pose.pose.orientation.x,
                                               pose.pose.pose.orientation.y,
                                               pose.pose.pose.orientation.z,
                                               pose.pose.pose.orientation.w)

        if abs(self.__pos[0]) > 2.9 or self.__pos[0] < 0:
            self.ooB = True
        elif abs(self.__pos[1]) > 3.9 or self.__pos[0] < 0:
            self.ooB = True
        elif abs(self.__pos[0]) < 2.9 and self.__pos[0] > 0:
            self.ooB = False
        elif abs(self.__pos[1]) < 3.9 and self.__pos[0] > 0:
            self.ooB = False

    def getfrontalPos(self, pose):
        if self.station_reached is True:
            # print(self.__[0])
            for element in "xyz":
                if element == "x":
                    self.__pos[0] = pose.pose.pose.position.x
                elif element == "y":
                    self.__pos[1] = pose.pose.pose.position.y
                elif element == "z":
                    self.__pos[2] = pose.pose.pose.position.z

    def setConfig(self, message):

        for element in "xyzrpa":
            if element == "x":
                self.__p_gain[0] = message.px_gain
                self.__i_gain[0] = message.ix_gain
                self.__d_gain[0] = message.dx_gain
                self.__aw_gain[0] = message.awx_gain
                self.__integral_lower_limit = message.integral_lower_limit
                self.__integral_upper_limit = message.integral_upper_limit

                if self.__integrator_active is not message.integrator_active:
                    self.__integrator_active = message.integrator_active
                    self.resetController(False)
                if self.__controller_active is not message.controller_active:
                    self.__controller_active = message.controller_active
                    self.resetController(True)

            elif element == "y":
                self.__p_gain[1] = message.py_gain
                self.__i_gain[1] = message.iy_gain
                self.__d_gain[1] = message.dy_gain
                self.__aw_gain[1] = message.awy_gain
                self.__integral_lower_limit = message.integral_lower_limit
                self.__integral_upper_limit = message.integral_upper_limit

                if self.__integrator_active is not message.integrator_active:
                    self.__integrator_active = message.integrator_active
                    self.resetController(False)
                if self.__controller_active is not message.controller_active:
                    self.__controller_active = message.controller_active
                    self.resetController(True)

            elif element == "z":
                self.__p_gain[2] = message.pz_gain
                self.__i_gain[2] = message.iz_gain
                self.__d_gain[2] = message.dz_gain
                self.__aw_gain[2] = message.awz_gain
                self.__integral_lower_limit = message.integral_lower_limit
                self.__integral_upper_limit = message.integral_upper_limit

                if self.__integrator_active is not message.integrator_active:
                    self.__integrator_active = message.integrator_active
                    self.resetController(False)
                if self.__controller_active is not message.controller_active:
                    self.__controller_active = message.controller_active
                    self.resetController(True)

            elif element == "r":
                self.__p_gain[3] = message.pr_gain
                self.__i_gain[3] = message.ir_gain
                self.__d_gain[3] = message.dr_gain
                self.__aw_gain[3] = message.awr_gain
                self.__integral_lower_limit = message.integral_lower_limit
                self.__integral_upper_limit = message.integral_upper_limit

            elif element == "p":
                self.__p_gain[4] = message.pp_gain
                self.__i_gain[4] = message.ip_gain
                self.__d_gain[4] = message.dp_gain
                self.__aw_gain[4] = message.awp_gain
                self.__integral_lower_limit = message.integral_lower_limit
                self.__integral_upper_limit = message.integral_upper_limit

            elif element == "a":
                self.__p_gain[5] = message.pya_gain
                self.__i_gain[5] = message.iya_gain
                self.__d_gain[5] = message.dya_gain
                self.__aw_gain[5] = message.awya_gain
                self.__integral_lower_limit = message.integral_lower_limit
                self.__integral_upper_limit = message.integral_upper_limit

            self.ready = message.ready
        # print(
        #     "p_gain: {}\ni_gain: {}\nd_gain: {}"
        #     "\nintegral_lower_limit: {}\nintegral_upper_limit: {}"
        #     "\nintegrator_active: {}\ncontroller_active: {}\n---".format(
        #         self.__p_gain,
        #         self.__i_gain,
        #         self.__d_gain,
        #         self.__integral_lower_limit,
        #         self.__integral_upper_limit,
        #         self.__integrator_active,
        #         self.__controller_active,
        #     )
        # )

    def __PID(self):
        # initialize stored data
       
        # if(self.ready):
        #     self.current_target_point= self.generate_waypoints(self.ready)
        #     self.resetController(True)
        #     self.ready = False
        # x, y, z, yaw
        # self.__setpoint[0] = 1
        # self.__setpoint[1] = 3.5
        # self.__setpoint[2] = -0.5
        # self.__setpoint[5] = 90
       # print(self.__setpoint[2])
        # if(self.slow):
        #     for element in range(6):
        #         time = rospy.get_time()

        #         # PID calculations
        #         e[element] = self.__setpoint[element] - self.__pos[element]

        #         self.__p[element] = self.__p_slow_gain[element] * e[element]
        #         if time - t_prev[element] > 0:
        #             self.__i[
        #                 element] = self.__i[element] + self.__i_slow_gain[element] * (
        #                     e[element] - aw_out[element] *
        #                     self.__aw_gain[element]) * (time - t_prev[element])

        #             self.__d[element] = self.__d_slow_gain[element] * (
        #                 e[element] - ed_prev[element]) / (time - t_prev[element])

        #         self.__i[element] = contstrain(self.__i[element],
        #                                         self.__integral_upper_limit,
        #                                         self.__integral_lower_limit)

        #         self.__output[element] = self.__p[element] + self.__i[
        #             element] + self.__d[element]
        #         aw_out[element] = self.__output[element]
        #         self.__output[element] = contstrain(self.__output[element],
        #                                             self.output_slow_constraint,
        #                                             -self.output_slow_constraint)
        #         aw_out[element] = aw_out[element] - self.__output[element]

        #         ed_prev[element] = e[element]
        #         t_prev[element] = time
        #     if np.abs(e[0]) <= 0.2 and np.abs(e[1]) <= 0.2 and np.abs(e[2]) <= 0.2:
        #         self.position = True
        #     if np.abs(e[5]) <= 2:
        #         self.position = False
        #     if self.ooB:
        #         self.__output[0] = 0
        #         self.__output[1] = 0
        #         self.__output[2] = 0
        #         self.__output[3] = 0
        #         self.__output[4] = 0
        #         self.__output[5] = 0
        #     if self.position is False:
        #         self.__output[3] = 0
        #         self.__output[4] = 0
        #         self.__output[5] = 0
        #     else:
        #         self.__output[1] = 0
        #         #self.__output[2] = 0
        #         self.__output[0] = 0
        #     self.__output[3] = 0
        #     self.__output[4] = 0
        #     self.__rotOutput = self.__output[0:3]
        #     self.__rotOutput = np.dot(self.rotM, self.__rotOutput)
        #     self.__publish()
        #     self.statusPub()

        # else:
        for element in range(6):
            time = rospy.get_time()
            ed_prev = np.empty(6)
            self.t_prev = np.tile(rospy.get_time(), 6)

        # output = np.empty(3)
        # aw_out = np.empty(6)
        # e = np.empty(6)
        # ed_prev = np.empty(6)

            # PID calculations
            self.e[element] = self.__setpoint[element] - self.__pos[element]
            # rospy.logwarn(e[element])

            self.__p[element] = self.__p_gain[element] * self.e[element]
            if time - self.t_prev[element] > 0:
                self.__i[
                    element] = self.__i[element] + self.__i_gain[element] * (
                        self.e[element] - self.aw_out[element] *
                        self.__aw_gain[element]) * (time - self.t_prev[element])
                # self.__i[element] = self.__i[element] + self.__i_gain[element] * (e[element]) * (
                #     time - t_prev[element])

                self.__d[element] = self.__d_gain[element] * (
                    self.e[element] - self.ed_prev[element]) / (time - self.t_prev[element])

            # if self.__integrator_active is False:
            #    self.__i[element] = 0

            #self.__i[element] = contstrain(self.__i[element],
                                        #    self.__integral_upper_limit,
                                        #    self.__integral_lower_limit)

            self.__output[element] = self.__p[element] + self.__i[
                element] + self.__d[element]
            self.aw_out[element] = self.__output[element]
            #print(self.__output[2])
            self.__output[element] = contstrain(self.__output[element],
                                                self.output_constraint,
                                                -self.output_constraint)
            self.aw_out[element] = self.aw_out[element] - self.__output[element]
            # print(self.__output[element])
            # if self.__controller_active is False:
            #   self.__output[element] = 0

            # update stored data for next iteration
            self.ed_prev[element] = self.e[element]
            self.t_prev[element] = time
        # print(e)
        # rospy.logwarn(e)

        # if np.abs(e[0]) <= 0.1 && np.abs(e[0]) <= 0.1 && np.abs(e[1]) <= 0.1 && np.abs(e[2]) <= 0.1 && np.abs(e[3]) <= 1 && np.abs(e[4]) <= 1 && np.abs(e[5]) <= 1:
        #     self.ready = true
        # if np.abs(self.e[0]) <= 0.2 and np.abs(self.e[1]) <= 0.5 and np.abs(self.e[2]) <= 0.2:
        #     self.position = True
        #     self.controlled = True
        #else:
            self.position = False
            self.controlled = False

        # if np.abs(e[5]) <= 2:
        #     self.position = False
        #     self.controlled = True

        if self.ooB:
            self.__output[0] = 0
            self.__output[1] = 0
            self.__output[2] = 0
            self.__output[3] = 0
            self.__output[4] = 0
            self.__output[5] = 0
        if self.position is False:
            self.__output[3] = 0
            self.__output[4] = 0
            self.__output[5] = 0
        else:
            self.__output[1] = 0
            # self.__output[2] = 0
            self.__output[3] = 0
        self.__output[3] = 0
        self.__output[4] = 0
        # rospy.logwarn(self.__pos[5])
        self.__rotOutput = self.__output[0:3]
        self.__rotOutput = np.dot(self.rotM, self.__rotOutput)
        self.__publish()
        self.statusPub()

    def resetController(self, all=True):
        self.__i = np.zeros(6)

        if all is True:
            self.__d = np.zeros(6)
            self.__p = np.zeros(6)
            self.__output = np.zeros(6)


def contstrain(value, upper, lower):
    if value < lower:
        value = lower
    if value > upper:
        value = upper
    return value


def quaternion_rotation_matrix(q0, q1, q2, q3):
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])

    return rot_matrix


def main():
    node = Controller()
    node.run()


if __name__ == "__main__":
    main()
