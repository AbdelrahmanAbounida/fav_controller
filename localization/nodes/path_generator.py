#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from localization.msg import customPos
from geometry_msgs.msg import Vector3Stamped, PointStamped
import numpy as np
import matplotlib.pyplot as plt

class PathGenerator:
    def __init__(self):

        rospy.init_node('path_generator_node', anonymous=True)
        rospy.Subscriber("/path_shape", String, self.target_path_shape_callback)  # to get the target path shape from the ui, ex. circle, square , spiral, sine.,,
        self.target_pos_publisher = rospy.Publisher('/target_position', PointStamped, queue_size=10) # to publish the target position to be followed using our algorithm 

        self.target_path = "Square" # initialize the target path to be square , u may edit manually if u don't want to use the Web_UI.

        self.POOL_WIDTH = 3 # max x in meter
        self.POOL_LENGTH = 4 # max x in meter 
        self.POOL_DEPTH = 0.8 # max Z in meter

        self.rate = rospy.Rate(10)


    def target_path_shape_callback(self,msg):
        self.target_path = msg.data

    

    def two_ways_trip(self,x,y,z):

        target_pos = PointStamped()
        for pos in list(zip(x,y,z)):
            target_pos.point.x = pos[0]
            target_pos.point.y = pos[1]
            target_pos.point.z = pos[2]
            yield target_pos

        for pos in list(zip(x[::-1],y[::-1],z[::-1])):
            target_pos.point.x = pos[0]
            target_pos.point.y = pos[1]
            target_pos.point.z = pos[2]
            yield target_pos
    

    def generate_shape(self,inf,x,y,z):
        ''' Main generator function for any shape'''
        if inf:
            while not rospy.is_shutdown():
                for pos in self.two_ways_trip(x,y,z):
                    self.target_pos_publisher.publish(pos)
                    self.rate.sleep()
        else:
            print("lets start")
            for pos in self.two_ways_trip(x,y,z):
                    self.target_pos_publisher.publish(pos)
                    self.rate.sleep()         
    

    def generate_square(self, num_of_points=100, inf=False):
        """ Square path in XY Plane"""
        min_x = 0.3
        max_x = self.POOL_WIDTH - 0.3
        min_y = 0.1
        max_y = self.POOL_LENGTH - 0.1
        
        x = np.concatenate((np.linspace(min_x,min_x,int(num_of_points/4)) ,np.linspace(min_x,self.POOL_WIDTH-0.3,int(num_of_points/4)),
                            np.linspace(max_x,max_x,int(num_of_points/4)),np.linspace(max_x,min_x,int(num_of_points/4))))

        y = np.concatenate((np.linspace(min_y,max_y,int(num_of_points/4)) ,np.linspace(max_y,max_y,int(num_of_points/4)),
                            np.linspace(max_y,min_y,int(num_of_points/4)),np.linspace(min_y,min_y,int(num_of_points/4))))
        
        z = np.linspace(0.1,0.1,num_of_points)
        
        self.generate_shape(inf,x,y,z)
        

    def generate_circle(self, num_of_points=60, radius=0.4,inf=False):
        """ Circle path in XZ Plane"""
        x0,z0 = 0.1, -0.4
        theta = np.linspace(0, 2 * np.pi, num_of_points)
        x = radius * np.cos(theta)
        z = radius * np.sin(theta)
        y = np.linspace(0.1,0.1,num_of_points)

        self.generate_shape(inf,x,y,z)
    
    def generate_sine(self, num_of_points=60,inf=False):
        """ Sine-wave path in YZ Plane"""
        y = np.linspace(0, self.POOL_WIDTH, num_of_points) 
        z = np.sin(np.pi * y)
        x = np.linspace(1.5,1.5,num_of_points)  # put any const value

        self.generate_shape(inf,x,y,z)
        

    def generate_spiral(self, num_of_points=60,inf=False):
        """ Spiral path in XYZ Space"""
        print(f"target path is: {self.target_path}")
        z = np.linspace(0.1, 20, 60) * 0.8/20 # mapping the output to 0.8, or simply replace 20 with 0.8
        x = np.sin(z)
        y = np.cos(z)

        self.generate_shape(inf,x,y,z)

    def main(self):
        while not rospy.is_shutdown():
            if self.target_path == "Square":
                self.generate_square()
            elif self.target_path == "Circle":
                self.generate_circle()
            elif self.target_path == "Sine":
                self.generate_sine()
            elif self.target_path == "Spiral":
                self.generate_spiral()
            else:
                continue
            self.rate.sleep()
    
    def test_pathes(self):
        pass



if __name__ == "__main__":
    gen = PathGenerator()
    gen.main()

