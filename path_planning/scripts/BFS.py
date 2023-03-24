#!/usr/bin/env python3

from Planner import Planner
from queue import Queue
class BFS(Planner):

    def __init__(self,cell_width,cell_height,map_width,map_height,start,goal,tolerance=50):

        super().__init__(cell_width,cell_height,map_width,map_height,start,goal)
        self.visited = []
        self.tolerance = tolerance

    def shortest_path(self):
        path = Queue()
        path.put([self.start])
        self.visited.append(self.start) 
        i = 1
        while path.qsize():
            #1- get current point 
            current_path = path.get()
            
            #2- get all possible directions of the last point in the current path
            current_point = current_path[-1]
            possible_directions = self.possible_directions(current_point)
            if i == 1:
                i = 2
            #3- add possible directions to the list
            for direction in possible_directions:

                p = current_path + [direction]
                self.visited.append(direction) # add direction to visied grid

                if abs(direction[0] - self.goal[0]) <= self.tolerance and abs(direction[1] - self.goal[1]) <= self.tolerance:
                    if direction == self.goal:
                        p[-1] = p[-1]+('station',) # make this point distinct
                    else:
                        p.append((self.goal[0],self.goal[1],'station'))
                    
                    return p  # shortest path

                # if direction == self.goal:
                #     return p 
                if not p in self.best_station_order_locations:
                    path.put(p)
            
            #print(possible_directions)
        
        #print(f"No available path between these points, {self.start}, {self.goal}")
        return []

