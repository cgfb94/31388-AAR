import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
import time

from Robot import Robot
from Plotter import Plotter

class Simulator():
    def __init__(self):
        self.sim_log        = []
        self.robots         = []
        self.sim_plot       = Plotter()

    def add_robot(self, name, color, wheel_seperation, wheel_diameter, sample_time = 0.01, start_pose = [0, 0, 0]):
        self.robots.append(Robot(name, color, wheel_seperation, wheel_diameter, sample_time, start_pose))

    def simulate(self, sim_time):
        now = time.time()
        while time.time() - now < sim_time:
            log_time = time.time() - now
            # update pose of each robot individually
            for robot in self.robots:
                pass
        
            time.sleep(0.01)

    def live_sim(self, sim_time):
        
        for robot in self.robots:
            self.sim_plot.add_robot(robot)
                  
        now = time.time()
        while time.time() - now < sim_time:
            Plotter.pos_ax.cla()
            # iterate through robots in sim, get thier poses
            for robot in self.robots:
                try:
                    next(robot.go_to_pose([15,15, 1]))
                except StopIteration:
                    x_pos, y_pos, theta = robot.get_pose()
                    self.sim_plot.update_data(robot.name, x_pos, y_pos, theta % (2*np.pi), time.time() - now, robot.r_speed[-1], robot.l_speed[-1])
                    input('Press ENTER to exit')
                    return
                x_pos, y_pos, theta = robot.get_pose()
                self.sim_plot.update_data(robot.name, x_pos, y_pos, theta % (2*np.pi), time.time() - now, robot.r_speed[-1], robot.l_speed[-1])

            # draw on the canvas with the updated data    
            self.sim_plot.update_canvas()
            time.sleep(0.01)
        # keep the plotting window open until user closes it    
        input('Press ENTER to exit')
        return
        
    def go_to_pose(self, robot, goal_pos):
        robot.goal_pos = goal_pos
        while   abs(robot.goal_pos[0] - robot.xi[0]) > 0.1 or \
                abs(robot.goal_pos[1] - robot.xi[1]) > 0.1 or \
                abs(robot.goal_pos[2] - robot.xi[2]) > 1:
            phi_dot = robot.wheel_speeds()
            new_pose = robot.update_pose(phi_dot)
            yield new_pose
