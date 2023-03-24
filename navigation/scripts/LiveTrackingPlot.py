#!/usr/bin/env python

from std_msgs.msg import String, Float64
from localization.msg import customPos
from drawnow import drawnow
import matplotlib.pyplot as plt
from IPython import display
from time import sleep
import numpy as np
import rospy


class LiveTrackingPlot:
    def __init__(self):
        rospy.init_node('live_plot_node',anonymous=True)
        rospy.Subscriber('/calculated_position', customPos, self.current_pos_callback) # current position from localizer
        rospy.Subscriber('/current_target_pos', customPos, self.current_target_pos_callback) # target position from path generator publisher (Don't use this anymore)
        rospy.Subscriber("/path_shape", String, self.target_path_shape_callback)
        
        self.current_pos_x = customPos()
        self.current_pos_y = customPos()
        self.current_pos_z = customPos()

        self.current_x = []
        self.current_y = []
        self.current_z = []

        self.target_x = []
        self.target_y = []
        self.target_z = []

        self.target_path_shape = "Sqaure"
        self.axes = self.generate_axes()
        self.POOL_WIDTH = 1.8 # max x in meter
        self.POOL_LENGTH = 3 # max x in meter 
        self.POOL_DEPTH = 0.8 # max Z in meter

        self.min_x,self.max_x = 0, self.POOL_WIDTH+0.3
        self.min_y,self.max_y = 0,self.POOL_LENGTH
        self.min_z,self.max_z = 0,-1

    
    def generate_axes(self):
        if self.target_path_shape == "Square":
            return "x-y"
        elif self.target_path_shape == "Sine":
            return "y-z"
        else:
            return "x-y"
        
    def current_pos_callback(self,msg):
        self.current_pos_x = msg.x
        self.current_pos_y = msg.y
        self.current_pos_z = msg.z
    
    def current_target_pos_callback(self,msg):
        self.current_x.append(self.current_pos_x)
        self.current_y.append(self.current_pos_y)
        self.current_z.append(self.current_pos_z)

        self.target_x.append(msg.x)
        self.target_y.append(msg.y)
        self.target_z.append(msg.z)

    
    def target_path_shape_callback(self,msg):
        self.target_path_shape = msg.data or "Square"

    def make_fig_x_y(self):
        plt.plot(self.current_x, self.current_y,'r',label='Current Position') 
        plt.plot(self.target_x, self.target_y,'b',label='Target Position') 

        plt.grid()
        plt.title("Square Shape Path Following")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend(loc="upper left")

    def make_fig_y_z(self):
        plt.plot(self.current_y, self.current_z,'r',label='Current Position') 
        plt.plot(self.target_y, self.target_z,'b',label='Target Position') 
        plt.grid()
        plt.title("Sine Shape Path Following")
        plt.xlabel("Y")
        plt.ylabel("Z")
        plt.legend(loc="upper left")
        

    def live_plot(self):
        # plt.plot(self.x,self.y,color='r')
        
        if self.axes == "x-y":
            plt.ion()  # enable interactivity
            fig = plt.figure()  # make a figure
            plt.grid()
            plt.title("Sqaure Shape Path Following")
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.legend()
            while not rospy.is_shutdown():
                sleep(0.05)
                drawnow(self.make_fig_x_y)

        if self.axes == "y-z":
            plt.ion()  # enable interactivity
            fig = plt.figure()  # make a figure
            plt.grid()
            plt.title("Sine Shape Path Following")
            plt.xlabel("Y")
            plt.ylabel("Z")
            plt.legend()
            while not rospy.is_shutdown():
                sleep(0.05)
                drawnow(self.make_fig_y_z)
   

if __name__ == '__main__':
    p = LiveTrackingPlot()
    p.live_plot()