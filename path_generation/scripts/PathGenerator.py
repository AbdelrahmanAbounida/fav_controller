#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import matplotlib.pyplot as plt
from path_generation.srv import GenerateWaypoints,GenerateWaypointsResponse

class PathGenerator:
    def __init__(self):

        rospy.init_node('path_generator_node', anonymous=True)
        self.target_pos_publisher = rospy.Publisher('/target_position', Pose, queue_size=10) # to publish the target position to be followed using our algorithm 

        ###########################
        #### Current z-position ###
        ###########################
        self.z = -0.2
        self.waypoints = []
        self.load_waypoints() # load waypoitns from parameter server

        self.POOL_WIDTH = 1.8 # max x in meter
        self.POOL_LENGTH = 3 # max x in meter 
        self.POOL_DEPTH = 0.8 # max Z in meter

        self.current_target_x = 0
        self.current_target_y = 0
        self.current_target_z = 0


        self.rate = rospy.Rate(10)
        
        self.current_pos = 0 # current_target_pos
    

    def load_waypoints(self):
        """ load generated waypoints by path planning algorithm from parameter server """
        if rospy.has_param('shortest_path_waypoints'):
            self.waypoints = rospy.get_param('/shortest_path_waypoints')
        else:
            rospy.loginfo('There is no shortest_path_waypoints parameter in the parameter server')
            return None
        

    def publish_waypoints(self):
        ''' publish current waypoints, we can use this method in case we need to generate waypooints manually in navigation code, 
            by taking list of points from this topic and follow them one by one'''
        for pos in self.waypoints:
            self.target_pos_publisher.publish(pos)
            self.rate.sleep()   

    def handle_points_generation(self,req):
        if req.ready:
            print("responding")
            if self.current_pos < len(self.waypoints):
                # there is no published orientation for now
                current_target_position = Pose()

                pos = Point()
                pos.x = self.waypoints[self.current_pos][0] # waypoints are list of 2d tuples, except for station positions, are 3d tuple
                pos.y = self.waypoints[self.current_pos][1]
                
                if len(self.waypoints[self.current_pos]) == 3: # a station position
                    print(self.waypoints[self.current_pos])
                    pos.z = -5 # any distict number
                else:
                    pos.z = self.z

                current_target_position.position = pos 

                resp =  GenerateWaypointsResponse(current_target_position=current_target_position)
                self.current_pos += 1
                print(f"current_target_point: {resp}")
                return resp
            else:

                current_target_position = Pose()

                pos = Point()
                pos.x = self.waypoints[-1][0] # waypoints are list of 2d tuples
                pos.y = self.waypoints[-1][1]
                pos.z = self.z

                return GenerateWaypointsResponse(current_target_position=current_target_position)

    def main(self):
        rospy.Service('generate_waypoint_service', GenerateWaypoints, self.handle_points_generation)

if __name__ == "__main__":
    gen = PathGenerator()
    gen.main()
    rospy.spin()
