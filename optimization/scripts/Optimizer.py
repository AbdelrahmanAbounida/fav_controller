#!/usr/bin/env python3
import rospy
import math
import random
from station_publisher.msg import Station, StationArray


class Optimizer:
    def __init__(self):
        pass
    
    def generate_feasible_solution(self,stations):
        "return an ordered list of stations according to their priority"

        # return sorted(self.stations, key=lambda x: x.station_priority == 3, reverse=True)
        priority_4 = list(filter(lambda station: station.station_priority  == 4, stations)) # start station
        priority_0 = list(filter(lambda station: station.station_priority  == 0, stations)) # end station

        priority_3 = list(filter(lambda station: station.station_priority  == 3, stations))
        priority_2 = list(filter(lambda station: station.station_priority  == 2, stations))
        priority_1 = list(filter(lambda station: station.station_priority  == 1, stations))

        random.shuffle(priority_3)
        random.shuffle(priority_2)
        random.shuffle(priority_1)

        return priority_4 + priority_3 + priority_2 + priority_1 + priority_0
    

    def _constraints(func):
        """ check if the given station order is feasible and doesn't break any of the 4 constraints """

        def wrapper(self,stations):
            """ stations: list of stations ordered in an optimized way
            n: total number of stations # might be != len(stations)  
            """

            station_names = [] # for constraint1,2
            pathes = ['']*(len(stations)-1) # for constraint3
            priorities = [] # for constraint4

            ################################
            ### constraint1, constraint2 
            ################################
            # Each path must be visited and only once ex: (a > b > a > c) will break the constraint >> Each station should appear one time in the list
            for index,station in enumerate(stations):
                station_names.append(station.station_name)

                if index and index != len(stations)-1:
                    pathes[index-1] = pathes[index-1] + station.station_name
                    pathes[index] = station.station_name + ' > '
                
                elif index:
                    pathes[index-1] = pathes[index-1] + station.station_name

                else:
                    pathes[index] = station.station_name + ' > '


            if len(set(station_names)) != len(station_names):
                raise Exception("Constraint1 / Constraint2 has been broken")

            ################################
            ### constraint3
            ################################
            # no back tracking or go back is allowed. ex: path A > B (departure of A should be before arrival of B)
            # ui - uj + nX_ij <= n-1 , where ui: the departure time of i station, uj: arrival time of j station
            for path in pathes:
                i,j    = path.split(' > ')
                ui,uj  = station_names.index(i)+1,station_names.index(j)+1
                if ui-uj + len(stations) * pathes.count(path) > len(stations)-1:
                    print(path)
                    raise Exception("Constraint3 has been broken")


            ################################
            ### constraint4
            ################################
            # order of station priorities must be followed.. ex: station A with prority 3 must come first before B with priority 2

            for i in range(len(stations)-1):
                station_i = stations[i].station_priority
                station_j = stations[i+1].station_priority

                priorities.append(station_j - station_i)

            # if priorities != sorted(priorities):
            #     raise Exception("Constraint4 has been broken")

            # if the given stations list is feasible and passed all constraints
            return func(self,stations)

        return wrapper


    @_constraints
    def cost_function(self,stations):
        cost = 0
        for i in range(len(stations)-1):
            departure_loc = stations[i].station_location.position
            arrival_loc = stations[i+1].station_location.position
            d_ij = math.sqrt( math.pow(arrival_loc.x - departure_loc.x, 2) + math.pow(arrival_loc.y - departure_loc.y, 2) )
            cost+= d_ij
        return cost

