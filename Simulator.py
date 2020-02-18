import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
import time

from Robot import Robot
from Plotter import Plotter

class Simulator():
    '''Simulate the movements of robots in arena'''
    def __init__(self, time_step = 0.01):
        self.sim_log        = []
        self.robots         = {}
        self.sim_plot       = Plotter()
        self.time_step      = time_step

    def add_robot(
        self, name, color, wheel_seperation, 
        wheel_diameter, sample_time = 0.01, start_pose = [0, 0, 0]):
        '''Add a robot to be simulated'''

        self.robots[name] = (Robot(
            name, color, wheel_seperation, wheel_diameter, 
            sample_time, start_pose))


    def live_sim(self, sim_time):
        '''Carry out a simulation for a set amount of time and plot'''

        # add the robots in the sim to the plot
        for _ , robot in self.robots.items():
            self.sim_plot.add_robot(robot)
            robot.set_command(robot.go_to_pose([10,10, 1]))
                  
        now = time.time()
        while time.time() - now < sim_time:
            # clear the pos_ax to remove previous arrow
            Plotter.pos_ax.cla()
            # iterate through robots in sim, get thier poses
            for _ , robot in self.robots.items():
                try:
                    # this is where we instruct the robot
                    next(robot.command)
                except StopIteration:
                    # if the simulation stops freeze the plot
                    x_pos, y_pos, theta = robot.get_pose()
                    self.sim_plot.update_data(
                        robot.name, x_pos, y_pos, theta % (2*np.pi), 
                        robot.actual_sim_time, robot.r_speed[-1], 
                        robot.l_speed[-1])
                    
                
                x_pos, y_pos, theta = robot.get_pose()
                self.sim_plot.update_data(
                    robot.name, x_pos, y_pos, theta % (2*np.pi), 
                    robot.actual_sim_time, robot.r_speed[-1], robot.l_speed[-1])

            # draw on the canvas with the updated data    
            self.sim_plot.update_canvas()
            time.sleep(0.01)

        # keep the plotting window open until user closes it    
        input('Press ENTER to exit')
        return

    def test_sim(self, sim_time):
        '''Carry out a simulation for a set amount of time and plot'''

        # add the robots in the sim to the plot
        for _ , robot in self.robots.items():
            self.sim_plot.add_robot(robot)
                
        now = time.time()
        
        while time.time() - now < sim_time:
            # clear the pos_ax to remove previous arrow
            Plotter.pos_ax.cla()
            # iterate through robots in sim, get thier poses
            for _ , robot in self.robots.items():
                x_pos, y_pos, theta = robot.get_pose()
                self.sim_plot.update_data(
                    robot.name, x_pos, y_pos, theta % (2*np.pi), 
                    time.time() - now, robot.r_speed[-1], robot.l_speed[-1])

            # draw on the canvas with the updated data    
            self.sim_plot.update_canvas()

        # keep the plotting window open until user closes it    
        input('Press ENTER to exit')
        return

    def set_commands(self, robot_name, commands):
        '''TODO give the robot a sequence of commands to follow'''
        self.robots[robot_name].commands = commands

    def sim_missions(self, sim_time):
        '''TODO Run the mission given in the commands'''

        # add the robots in the sim to the plot
        for _ , robot in self.robots.items():
            self.sim_plot.add_robot(robot)
                
        now = time.time()
        
        while time.time() - now < sim_time:
            # clear the position axis so new arrow can be plotted
            Plotter.pos_ax.cla()
            Plotter.pos_ax.grid()

            # iterate through robots in sim, get thier poses
            for _, robot in self.robots.items():
                robot.run()
                x_pos, y_pos, theta = robot.get_pose()
                self.sim_plot.update_data(
                    robot.name, x_pos, y_pos, theta % (2*np.pi), 
                    time.time() - now, robot.r_speed[-1], robot.l_speed[-1])
                time.sleep(self.time_step)

            # draw on the canvas with the updated data    
            self.sim_plot.update_canvas()

        # keep the plotting window open until user closes it    
        input('Press ENTER to exit')
        return



if __name__ == "__main__":
    print("Running Robot.py")
