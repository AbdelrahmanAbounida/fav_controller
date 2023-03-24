#!/usr/bin/env python
from scipy.optimize import fsolve
import rospy
from nav_msgs.msg import Odometry
from fav_msgs.msg import RangeMeasurementArray
from std_msgs.msg import Float32MultiArray

from geometry_msgs.msg import Vector3Stamped, PointStamped


class Localizer:

    def __init__(self):
        rospy.init_node("localization_node")
        # camera offset
        self.CAMERA_OFFSET_X = 0
        self.CAMERA_OFFSET_Y = 0.2
        self.CAMERA_OFFSET_Z = 0

        # position tag 1

        self.TAG_POS_X = 0.5
        self.TAG_POS_Y = 3.35
        self.TAG_POS_Z = -0.5

        # current calculated camera position
        self.estimated_x = 0
        self.estimated_y = 0
        self.estimated_z = 0
        self.depth = 0
        # current actual position
        self.actual_x = 0
        self.actual_y = 0
        self.actual_z = 0

        self.fov = False  # to check if at least 3 tags are in the same camera view

        self.pos_publisher = rospy.Publisher(
            "calculated_position", PointStamped,
            queue_size=10)  # to publish the calculated position (x,y,z)

        self.pos_error_pub = rospy.Publisher(
            "pos_calculation_error", Vector3Stamped, queue_size=10
        )  # to publish the diff between the calculated position and the actual positoin

        self.odometry_sub = rospy.Subscriber(
            "ground_truth/state", Odometry, self.current_pos_error_callback
        )  # to calculate the current position error
        self.range_sub = rospy.Subscriber("ranges", RangeMeasurementArray,
                                          self.current_ranges_callback)
        self.depth_sub = rospy.Subscriber("depth", Float32MultiArray,
                                          self.depth_callback)

    def current_pos_error_callback(self, msg):
        """Calculate the calculated position error"""

        self.actual_x = msg.pose.pose.position.x
        self.actual_y = msg.pose.pose.position.y
        self.actual_z = msg.pose.pose.position.z

        position_error = Vector3Stamped()
        position_error.vector.x = abs(self.actual_x) - abs(self.estimated_x)
        position_error.vector.y = abs(self.actual_y) - abs(self.estimated_y)
        position_error.vector.z = abs(self.actual_z) - abs(self.depth)

        self.pos_error_pub.publish(position_error)

    def current_ranges_callback(self, data):
        """Estimate the current position"""

        def equations(positions, available_tags, combination):
            x, y = positions

            if combination == (0, 1):
                equ1 = ((x - self.TAG_POS_X)**2 + (y - self.TAG_POS_Y)**2 +
                        (self.depth - self.TAG_POS_Z)**2 -
                        available_tags["0"]**2)
                equ2 = ((x - self.TAG_POS_X - 0.6)**2 +
                        (y - self.TAG_POS_Y)**2 +
                        (self.depth - self.TAG_POS_Z)**2 -
                        available_tags["1"]**2)

            elif combination == (0, 3):
                equ1 = ((x - self.TAG_POS_X)**2 + (y - self.TAG_POS_Y)**2 +
                        (self.depth - self.TAG_POS_Z)**2 -
                        available_tags["0"]**2)
                equ2 = ((x - self.TAG_POS_X - 0.6)**2 +
                        (y - self.TAG_POS_Y)**2 +
                        (self.depth - self.TAG_POS_Z + 0.4)**2 -
                        available_tags["3"]**2)

            elif combination == (2, 1):
                equ1 = ((x - self.TAG_POS_X)**2 + (y - self.TAG_POS_Y)**2 +
                        (self.depth - self.TAG_POS_Z + 0.4)**2 -
                        available_tags["2"]**2)
                equ2 = ((x - self.TAG_POS_X - 0.6)**2 +
                        (y - self.TAG_POS_Y)**2 +
                        (self.depth - self.TAG_POS_Z)**2 -
                        available_tags["1"]**2)

            else:
                equ1 = ((x - self.TAG_POS_X)**2 + (y - self.TAG_POS_Y)**2 +
                        (self.depth - self.TAG_POS_Z + 0.4)**2 -
                        available_tags["2"]**2)
                equ2 = ((x - self.TAG_POS_X - 0.6)**2 +
                        (y - self.TAG_POS_Y)**2 +
                        (self.depth - self.TAG_POS_Z + 0.4)**2 -
                        available_tags["3"]**2)

            return (equ1, equ2)

        x_sum = 0
        y_sum = 0
        available_tags = {}
        solveable_tag_combinations = [(0, 1), (0, 3), (2, 1), (2, 3)]
        available_tag_combinations = []
        for i in data.measurements:
            available_tags.update({str(i.id-1): i.range})
        keys = available_tags.keys()
        for combination in solveable_tag_combinations:
            if str(combination[0]) in keys and str(combination[1]) in keys:
                available_tag_combinations.append(combination)

        for combination in available_tag_combinations:
            new_x, new_y = fsolve(equations,
                                  (self.estimated_x, self.estimated_y),
                                  args=(available_tags,
                                        combination))  # use pre
            x_sum += new_x
            y_sum += new_y
        if len(available_tag_combinations) > 0:
            self.estimated_x = x_sum / len(available_tag_combinations)
            self.estimated_y = y_sum / len(available_tag_combinations)
            self.fov = True

            self.estimated_x = self.estimated_x - self.CAMERA_OFFSET_X
            self.estimated_y = self.estimated_y - self.CAMERA_OFFSET_Y
            self.estimated_z = self.depth - self.CAMERA_OFFSET_Z
            # print(f"calculated_x: {self.estimated_x}, calculated_y: {self.estimated_y}, calculated_z: {self.estimated_z}")
            # print(f"current_X: {self.actual_x}, current_y: {self.actual_y}, current_z: {self.actual_z}")
            # print('===========================================')
            msg = PointStamped()
            msg.point.x = self.estimated_x
            msg.point.y = self.estimated_y
            msg.point.z = self.estimated_z

            msg.header.stamp = rospy.Time().now()
            msg.header.frame_id = 'map'
            self.pos_publisher.publish(msg)
        else:
            self.fov = False

    def depth_callback(self, depth):
        self.depth = depth.data[1]


if __name__ == "__main__":
    localizer = Localizer()
    rospy.spin()
