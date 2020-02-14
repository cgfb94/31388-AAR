import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

from Robot import Robot
from Plotter import Plotter

class Simulator():
    def __init__(self):
        self.pose_history   = []
        self.sim_df         = pd.DataFrame()
        self.line1          = []
        self.sim_plot       = Plotter()

        #self.sim_plot.run_animation()


    def simulate(self, Robot, velocities, sim_time):
        now = time.time()
        while time.time() - now < sim_time:
            log_time = time.time() - now
            x_pos, y_pos, theta_pos = Robot.update_pose(velocities)
            self.pose_history.append([log_time, x_pos, y_pos, theta_pos])
            time.sleep(0.01)
    
    def gen_data_frame(self):
        self.sim_df = pd.DataFrame(self.pose_history)

    def plot_simulation(self):
        self.sim_df.plot()
        plt.show()

    def live_sim(self, Robot, velocities, sim_time):
        
        now = time.time()
        while time.time() - now < sim_time:
            x_pos, y_pos, theta = Robot.update_pose(velocities)
            #self.live_plotter(x_pos, y_pos, theta)
            self.sim_plot.update_data(x_pos, y_pos, theta % (2*np.pi), time.time() - now)
            time.sleep(0.01)
        input('Press ENTER to exit')
        

    def live_plotter(self, x, y, theta, identifier='', sim_time = 0.1):
        line1 = self.line1
        if line1==[]:
            # this is the call to matplotlib that allows dynamic plotting
            plt.ion()
            fig = plt.figure(figsize=(6,6))
            self.ax = fig.add_subplot(111)
            # create a variable for the line so we can later update it
            self.ax.scatter(x, y)        
            #update plot label/title
            plt.ylabel('Y Label')
            plt.title('Title: {}'.format(identifier))
            plt.show()
            self.line1 = [1]
        
        # after the figure, axis, and line are created, we only need to update the y-data
        self.ax.scatter(x, y, c='r')
        # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
        plt.pause(sim_time)

if __name__ == "__main__":
    pass