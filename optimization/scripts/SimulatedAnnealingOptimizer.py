#!/usr/bin/env python3
import rospy

from Optimizer import Optimizer

class SimulatedAnnealing(Optimizer):
    def __init__(self,stations,T0=1000,Tf=0.1,iterations=300,nT=2):
        """ 
        T0: Initial Temperature
        Tf: Final Temperature
        iterations: number of iterations
        nT: num of iterations per temperature
        """
        self.stations = stations
        self.x_cur = self.generate_feasible_solution(self.stations) # initial solution
        self.T0 = T0
        self.Tf = Tf
        self.i_max = iterations
        self.best_solution = self.x_cur
        self.best_cost = self.cost_function(self.x_cur)
        self.nT = nT
    
    def optimize(self):
        # 1- initialization
        beta = (self.Tf - self.T0) / self.i_max
        T_cur = self.T0
        i_cur = 0

        while(i_cur < self.i_max and T_cur < self.Tf):
            # 2- Generate new feasible solution
            x_new = self.generate_feasible_solution(self.stations)

            # 3- compute change in energy state
            current_cost = self.cost_function(self.x_cur)
            new_cost = self.cost_function(self.x_new)

            delta_e = new_cost - current_cost

            # 4- check if the solution is better that current solution
            if delta_e < 0:
                self.x_cur = x_new

            # 5- accept worse solution if we get a random value
            else:
                r = random.random()
                p = math.exp(-1*delta_e / T_cur)

                if p > r:
                    x_cur = x_new
                    current_cost = new_cost
            
            # 6- update best solution reached
            if current_cost < self.best_cost:
                self.best_cost = current_cost
                self.best_solution = x_cur
            
            # 7- update temperature
            T_cur = self.Tf - beta * i_cur
            i_cur+=1

        return self.best_solution
    
