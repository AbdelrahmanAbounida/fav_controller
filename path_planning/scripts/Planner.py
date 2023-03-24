#!/usr/bin/env python3
import rospy

class Planner:

    def __init__(self,cell_width,cell_height,map_width,map_height,start,goal):

        self.cell_width = cell_width
        self.cell_height = cell_height
        self.map_width = map_width
        self.map_height = map_height
        self.start = start
        self.goal = goal
        self.visited = []
        self.best_station_order_locations = None
        self.load_parameter_server()

    def load_parameter_server(self):
        # list of best_station_order
        if rospy.has_param('best_station_order_locations'):
            self.best_station_order_locations = rospy.get_param("/best_station_order_locations")
        else:
            rospy.loginfo("Please check that best_station_order parameter already exist in parameter server")

    def possible_directions(self,current_point):

        right = (current_point[0] + self.cell_width, current_point[1])
        left = (current_point[0] - self.cell_width, current_point[1])  
        up = (current_point[0] , current_point[1]- self.cell_height) 
        down = (current_point[0] , current_point[1] + self.cell_height) 

        up_right = (current_point[0] + self.cell_width, current_point[1]- self.cell_height)
        up_left = (current_point[0] - self.cell_width, current_point[1]- self.cell_height)
        down_right = (current_point[0] + self.cell_width, current_point[1] + self.cell_height)
        down_left = (current_point[0] - self.cell_width, current_point[1] + self.cell_height)

        # return valid directions
        all_directions = [right,left,up,down,up_right,up_left,down_right,down_left]
        valid_direction = [direction for direction in all_directions if self.valid_move(direction)]
        return valid_direction

    def valid_move(self,direction):
        x,y = direction
        
        if x >= self.map_width or x<0 or y >= self.map_height or y<0 or direction in self.visited:
            return None
        
        return direction

