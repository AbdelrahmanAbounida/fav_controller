#!/usr/bin/env python3

from Planner import Planner


class AStar(Planner):

    def __init__(self,cell_width,cell_height,map_width,map_height,start,goal):

        super().__init__(cell_width,cell_height,map_width,map_height,start,goal)
        self.visited = []