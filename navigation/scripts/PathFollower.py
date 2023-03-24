#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, Quaternion # path Planning topoic
from geometry_msgs.msg import PoseWithCovarianceStamped # localization topic
from path_generation.srv import GenerateWaypoints, GenerateWaypointsRequest
import time 


class PathFollower:
    def __init__(self):
        rospy.init_node('navigation_node',anonymous=True)
        rospy.Subscriber('/bluerov/visual_localization/pose', PoseWithCovarianceStamped, self.current_pos_callback) # current position from localization node
        rospy.Subscriber('/target_position', Pose, self.target_pos_callback) # target position from path generator publisher (use this in case u gonna handle path generation ur self)

        ###########################
        ## Controller Parameters ##
        ###########################
        self.x_controller_pub = rospy.Publisher('/bluerov/lateral_thrust',Float64,queue_size=10)
        self.y_controller_pub = rospy.Publisher('/bluerov/thrust',Float64,queue_size=10)
        self.z_controller_pub = rospy.Publisher('/bluerov/vertical_thrust',Float64,queue_size=10)

        # to publish the current target pos getting from path generation service to the web ui
        self.current_target_pos_pub = rospy.Publisher('/current_target_pos',Pose,queue_size=10)
        self.current_pos = Pose()
        self.target_pos = Pose()
        self.KP = 0.7 # Kp=0.5 square, 

        self.ready = True # this variable will tell the waypoint generator if the robot is ready to recieve the next point or it stills approaching current point
        self.current_target_point = Pose()
        self.rate = rospy.Rate(7)

        self.last_time = 0
        self.current_time = 0

        ###########################
        ## Scaling Parameters ##
        ###########################

        if rospy.has_param('/pool_width') and rospy.has_param('/map'):
            self.x_scale = rospy.get_param('/pool_width') / rospy.get_param('/map')['width']
        else:
            rospy.loginfo('Please check that pool width parameter already exist in the parameter server')
            
        if rospy.has_param('/pool_height') and rospy.has_param('/map'):
            self.y_scale = rospy.get_param('/pool_height') / rospy.get_param('/map')['height']
        else:
            rospy.loginfo('Please check that pool height parameter already exist in the parameter server')
    
    def current_pos_callback(self,msg):
        """ msg: geometry_msgs/PoseWithCovarianceStamped >> (header, Pose(pose,covariance)"""
        self.current_pos = msg.pose.pose
    
    def target_pos_callback(self,msg):
        # don't use this anymore
        self.target_pos = msg
    
    def follow(self,wait_time=3):
        generate_waypoints = rospy.ServiceProxy('generate_waypoint_service', GenerateWaypoints) # call waypoint generation service
        if self.ready:
            self.last_time = self.current_time
            self.current_target_point= generate_waypoints(self.ready).current_target_position
            print(self.current_target_point)

            self.current_target_pos_pub.publish(self.current_target_point) # to use in the webui

        self.current_time = time.time()
        ###############################################
        ########## Apply Controller Here ##############
        ###############################################

        error_x = self.current_pos.position.x  - self.current_target_point.position.x * self.x_scale # positive out >> move left 
        error_y = self.current_target_point.position.y * self.y_scale - self.current_pos.position.y  # positive out >> move forward 
        # error_z = self.current_target_point.position.z - self.current_pos.position.z 

        self.ready = self.update_controller_state(error_x,error_y) # or (self.current_time - self.last_time >= wait_time)

        if abs(error_x) < 0.2:
            error_x = 0

        if abs(error_y) < 0.2:
            error_y = 0
            
        # if abs(error_z) < 0.2:
        #     error_z = 0

        self.x_controller_pub.publish(self.saturate_controller(error_x*self.KP))
        self.y_controller_pub.publish(self.saturate_controller(error_y*self.KP))
        # self.z_controller_pub.publish(self.saturate_controller(error_z*self.KP))

        print(f"error_x: {error_x}")
        print(f"error_y: {error_y}")
        # print(f"error_z: {error_z}")
        print("=======================")
        print(f"target_pos:{self.current_target_point.position}")
        
    def saturate_controller(self,controller_output):
        "saturate the controller output to -0.7:0.7 to prevent fluctuation"

        if controller_output < -0.7:
            return -0.7
        elif controller_output > 0.7:
            return 0.7
        else:
            return controller_output
    
    def update_controller_state(self,error_x,error_y,tolerance=0.2,wait_time=5):
        """update ready state to switch to the next point depending on current error,
         tolerance: is the 3d error space radius"""

        return  abs(error_x) <= tolerance and abs(error_y) <= tolerance

if __name__ == '__main__':
    follower = PathFollower()
    rospy.wait_for_service('generate_waypoint_service') # wait for waypoints generator server to be ready
    while not rospy.is_shutdown():
        follower.follow()
        follower.rate.sleep()
