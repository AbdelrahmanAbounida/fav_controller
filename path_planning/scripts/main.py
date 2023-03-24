#!/usr/bin/env python3
from BFS import BFS
from station_publisher.msg import Station, StationArray
from geometry_msgs.msg import Point, Quaternion, Pose
import rospy

class Main:

    def __init__(self):
        rospy.init_node("path_planning_node", anonymous=True)
        self.stations = None
        self.path_planning_algorithm = None
        self.map_width, self.map_height, self.rows, self.cols = (None,)*4
        self.start_point = None
        self.end_point = None
        self.load_parameter_server()
        self.shortest_path_list = self.generate_shortest_path()

    def load_parameter_server(self):
        # list of stations
        if rospy.has_param('best_station_order'):
            stations = rospy.get_param("/best_station_order")
            self.stations = self.create_stations(stations)
        else:
            rospy.loginfo("Please check that stations parameter already exist in parameter server")

        # path planning algorithm
        if rospy.has_param('/Path_Planning_Technique'):
            self.path_planning_algorithm = rospy.get_param("/Path_Planning_Technique")
        else:
            rospy.loginfo("Please check that Path_Planning_Technique parameter already exist in parameter server")

        # map parameters
        if rospy.has_param('map'):
            map_params = rospy.get_param('map')
            self.map_width = map_params['width']
            self.map_height = map_params['height']
            self.rows = map_params['rows']
            self.cols = map_params['cols']

            self.cell_width = self.map_width // self.rows
            self.cell_height = self.map_height // self.cols
        else:
            rospy.loginfo("Please check that map parameters already exist in parameter server")

        # start and end point
        if rospy.has_param('/start_point'):
            start = rospy.get_param("/start_point")
            s = Pose()

            position = Point()
            position.x = start["position"][0]
            position.y = start["position"][1]
            position.z = start["position"][2]
            s.position = position
            
            orientation = Quaternion()
            orientation.x = start["orientation"][0]
            orientation.y = start["orientation"][1]
            orientation.z = start["orientation"][2]
            orientation.w = start["orientation"][3]
            s.orientation = orientation
            
            self.start_point = s

        else:
            rospy.loginfo("Please check that start_point parameter already exist in parameter server")
        
        if rospy.has_param('/end_point'):
            end = rospy.get_param("/end_point")
            s = Pose()

            position = Point()
            position.x = end["position"][0]
            position.y = end["position"][1]
            position.z = end["position"][2]
            s.position = position

            orientation = Quaternion()
            orientation.x = end["orientation"][0]
            orientation.y = end["orientation"][1]
            orientation.z = end["orientation"][2]
            orientation.w = end["orientation"][3]
            s.orientation = orientation
            
            self.end_point = s
        else:
            rospy.loginfo("Please check that end_point parameter already exist in parameter server")
        
    def create_stations(self,stations):
        """ create list of stations from parameter server best_station_order """

        station_array = StationArray()

        for item in stations:
            station = Station()

            # 1- station name
            station.station_name = item["name"]

            # 2- station location
            pos = Point()
            pos.x = item["position"][0]
            pos.y = item["position"][1]
            pos.z = item["position"][2]

            orient = Quaternion()
            orient.x = item["orientation"][0]
            orient.y = item["orientation"][1]
            orient.z = item["orientation"][2]
            orient.w = item["orientation"][3]
                                             
            station_position = Pose()
            station_position.position = pos
            station_position.orientation = orient

            station.station_location = station_position

            # 3- station priority
            station.station_priority = item["priority"]

            # # 4- station id
            # station.id = item["id"]

            station_array.stations.append(station)
        
        return station_array.stations

    def generate_shortest_path(self):
        """generate the shortest_path from the start point to the last one"""

        shortest_path = []
        next_start = (self.start_point.position.x,self.start_point.position.y) # make the point as a 2d tuple as the algorithm assuming 2d space now
        current_goal = None
        if self.path_planning_algorithm == "BFS":

            for station in self.stations:
                current_goal = (station.station_location.position.x,station.station_location.position.y)
                planner = BFS(self.cell_width, self.cell_height, self.map_width, self.map_height, next_start, current_goal)

                current_path = planner.shortest_path()
                shortest_path+=current_path

                next_start = current_goal
            
            if not current_goal in shortest_path:
                shortest_path.append(current_goal+("station",))

        elif self.path_planning_algorithm == "DFS":

            for station in self.stations:
                current_goal = (station.station_location.position.x,station.station_location.position.y)
                planner = BFS(self.cell_width, self.cell_height, self.map_width, self.map_height, next_start, current_goal)

                current_path = planner.shortest_path()
                shortest_path+=current_path     
                next_start = current_goal


        elif self.path_planning_algorithm == "AStar":

            for station in self.stations:
                current_goal = (station.station_location.position.x,station.station_location.position.y)
                planner = BFS(self.cell_width, self.cell_height, self.map_width, self.map_height, next_start, current_goal)

                current_path = planner.shortest_path()
                shortest_path+=current_path 
                next_start = current_goal

        else:
            raise TypeError("You must specifiy the required Path Optimization Technique")        

        shortest_path.append((self.end_point.position.x,self.end_point.position.y))
        return shortest_path 
    
    def publish_shortest_path(self):
        """ adding shortest_path into the parameter server list  """
        rospy.set_param('shortest_path_waypoints', self.shortest_path_list)


if __name__ == '__main__':
    m = Main()
    m.publish_shortest_path()
