#!/usr/bin/env python3
import time 
import rospy
import numpy as np
from std_msgs.msg import Float64,Bool
from geometry_msgs.msg import Pose, Point, Quaternion # path Planning topoic
from geometry_msgs.msg import PoseWithCovarianceStamped # localization topic
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class YawAngleHolder:
    """  
    - right: 0
    - left: 180 / -170
    - forward: 90
    - backward: -90
    """
    def __init__(self):
        rospy.init_node('yaw_holder_node',anonymous=True)
        rospy.Subscriber('/bluerov/visual_localization/pose', PoseWithCovarianceStamped, self.current_yaw_callback) # current position from localization node
        rospy.Subscriber('/current_target_yaw', Float64, self.target_yaw_callback) # current position from localization node
        rospy.Subscriber('/allow_yaw', Bool, self.allow_yaw_callback) # current position from localization node


        self.current_yaw = 0
        self.target_yaw = 90

        self.yaw_KP = 0.1
        self.yaw_KD = 2

        self.last_error_yaw = 0
        self.current_time = 0
        self.last_time = 0

        self.allow = True

        self.yaw_controller_pub = rospy.Publisher('/bluerov/yaw',Float64,queue_size=10)

        self.euler_orientation = [0,0,0] # roll = pitch = yaw = 0.0

    def current_yaw_callback(self,msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.current_yaw = yaw
        # print(f"current_yaw:{yaw}")

    def target_yaw_callback(self,msg):
        """  current target yaw in degrees """
        self.target_yaw = msg.data
    
    def allow_yaw_callback(self,msg):
        self.allow = msg.data

    def hold(self,tolerance=0.05):
        if self.allow:
            target_rad = self.target_yaw*math.pi/180
            yaw_error = (target_rad-self.current_yaw)
            
            while abs(yaw_error) > tolerance:
                target_rad = self.target_yaw*math.pi/180
                yaw_error = (target_rad-self.current_yaw)
                self.yaw_controller_pub.publish(self.saturate_controller(self.yaw_KP * yaw_error))

            self.yaw_controller_pub.publish(0)

    def saturate_controller(self,cmd):

        if cmd < -0.5:
            return -0.5

        if cmd > 0.5:
            return 0.5

        elif cmd < 0 and cmd > -0.25:
            return -0.25

        elif cmd > 0 and cmd < 0.25:
            return 0.25
        
        return cmd

if __name__ == '__main__':
    angel_holder = YawAngleHolder()
    while not rospy.is_shutdown():
        angel_holder.hold()
