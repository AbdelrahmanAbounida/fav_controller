#!/usr/bin/env python3

from SimulatedAnnealingOptimizer import SimulatedAnnealing
from station_publisher.msg import Station, StationArray
from geometry_msgs.msg import Point, Quaternion, Pose
import rospy

class Main:
    def __init__(self):
        rospy.init_node("optimization_node", anonymous=True)
        self.stations = None
        self.optimization_algorithm = None
        self.load_parameter_server()
        self.order_optimizer = self.current_order_optimization_algorithm(self.stations,self.optimization_algorithm)
        self.best_order = self.best_station_orders()

    def load_parameter_server(self):
        if rospy.has_param('stations'):
            stations = rospy.get_param("/stations")
            self.stations = self.create_stations(stations)
        else:
            rospy.loginfo("Please check that stations parameter already exist in parameter server")

        if rospy.has_param('/Optimization_Algorithm'):
            self.optimization_algorithm = rospy.get_param("/Optimization_Algorithm")
        else:
            rospy.loginfo("Please check that Optimization_Algorithm parameter already exist in parameter server")
            

    def create_stations(self,stations):
        """ create list of stations from parameter server stations """

        station_array = StationArray()

        for item in stations:
            station = Station()

            # 1- station name
            station.station_name = item["station_name"]

            # 2- station location
            pos = Point()
            pos.x = item["map_pose"][0]
            pos.y = item["map_pose"][1]
            pos.z = item["map_pose"][2]

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
            station.station_priority = item["station_priority"]

            # # 4- station id
            # station.id = item["id"]

            station_array.stations.append(station)
        
        return station_array.stations

    def current_order_optimization_algorithm(self,stations,optimization_algorithm):

        if not stations or not optimization_algorithm:
            rospy.loginfo("Please check first that either stations or optimization_algorithm parameters already exist in the parameter server")
            return None

        if optimization_algorithm:

            if optimization_algorithm == "simulated_annealing":
                return SimulatedAnnealing(stations)

            elif optimization_algorithm == "ant_colony":
                return AntColony(stations)

            elif optimization_algorithm == "genetic_algorithm":
                return GeneticAlgorithm(stations)

            elif optimization_algorithm == "grey_wolf":
                return GreyWolf(stations)

            elif optimization_algorithm == "particle_swarm":
                return ParticleSwarm(stations)

            elif optimization_algorithm == "whale_optimization":
                return WhaleOptimization(stations)
            else:
                raise Exception("Given optimization algorithm is wrong")
        else:
            raise TypeError("You must specifiy the required Optimization Algorithm")
    
    def best_station_orders(self):
        """generate the best optimized order of stations that doesn't break some constraints such as stations priority"""

        if not self.order_optimizer:
            rospy.loginfo("Please check parameter server, as the algorithm can't generate a best order")
            return None

        optimizer = self.order_optimizer
        best_order = optimizer.optimize()
        return best_order

    def publish_best_station_order(self):
        """ adding best station order into the parameter server list  """
        if not self.best_order:
            rospy.loginfo("There is not best order, please check that if there is stations in the parameter server")
            return None
        best_station_order_list = [
        {'name':station.station_name,
        'position':[station.station_location.position.x,station.station_location.position.y,station.station_location.position.z],
        'orientation':[station.station_location.orientation.x,station.station_location.orientation.y,station.station_location.orientation.z,station.station_location.orientation.w],
        'priority':station.station_priority} for station in self.best_order]
        best_station_order_locations = [[station.station_location.position.x,station.station_location.position.y] for station in self.best_order]


        rospy.set_param('best_station_order', best_station_order_list)
        rospy.set_param('best_station_order_locations', best_station_order_locations)



if __name__ == '__main__':
    m = Main()
    m.publish_best_station_order()
