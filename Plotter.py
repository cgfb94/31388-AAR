import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as image
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import time

class Plotter():

    def __init__(self, x_pos = 0, y_pos = 0, theta_pos = 0):

        # constants
        self.dpi = 72; self.imageSize = (47,47)
        # read in our png file
        self.im = image.imread('smr.jpg')
       

        self.x_pos = [x_pos]
        self.y_pos = [y_pos]
        self.theta_pos = [theta_pos]
        self.sim_time = [0]

        # define a 2x2 grid to hold plots
        self.fig, self.ax = plt.subplots(2,2, dpi=self.dpi)
        plt.ion()

        # axes for position, speed and angle of robot
        self.pos_ax = plt.subplot(122)
        self.speed_ax = plt.subplot(221)
        self.ang_ax = plt.subplot(223)

        self.pos_ax.set_xlim([-20,20])
        self.pos_ax.set_ylim([-20,20])

        self.lspeed, = self.speed_ax.plot([], [], 'b-', lw=2)
        self.rspeed, = self.speed_ax.plot([], [], 'r-', lw=2)
        self.angle, = self.ang_ax.plot([], [], 'g', lw=2)

        self.speed_ax.set_xlim([0,50])
        self.speed_ax.set_ylim([-np.pi,np.pi])

        self.ang_ax.set_ylim([0,2*np.pi])
        self.ang_ax.set_xlabel("Time (s)")
        self.ang_ax.set_ylabel("Angle (rad)")

        self.robot_pos, = self.pos_ax.plot([], [], 'g-', lw=2)
        self.robot_star, = self.pos_ax.plot([], [], 'g*')

    def update_data(self, x, y, theta, time):
        self.x_pos.append(x)
        self.y_pos.append(y)
        self.theta_pos.append(theta)
        self.sim_time.append(time)

        self.pos_ax.cla()
        self.pos_ax.patch.set_alpha(0)
        self.robot_pos, = self.pos_ax.plot([], [], 'g-', lw=2)
        self.robot_star, = self.pos_ax.plot([], [], 'g*')
        self.pos_ax.set_xlim([-20,20])
        self.pos_ax.set_ylim([-20,20])

        self.robot_pos.set_data(self.x_pos[-10:], self.y_pos[-10:])
        self.robot_star.set_data(self.x_pos[-1], self.y_pos[-1])
        if time:
            self.ang_ax.set_xlim([0,time])
        
        self.angle.set_data(self.sim_time, self.theta_pos)

        # plot image of robot on latest position
        im = OffsetImage(self.im, zoom=72/self.dpi)
        im.image.axes = self.pos_ax
        #ab = AnnotationBbox(im, (self.x_pos[-1], self.y_pos[-1]), frameon=False, pad=0.0)
        
        #self.pos_ax.add_artist(ab)
        bbox_props = dict(boxstyle="rarrow,pad=0.3", fc="cyan", ec="b", lw=2)
        t = self.pos_ax.text(self.x_pos[-1], self.y_pos[-1], "Robot", ha="center", va="center", rotation=(self.theta_pos[-1]*(180/np.pi)),
            size=5,
            bbox=bbox_props)

        plt.pause(0.1)
        self.fig.canvas.draw()
        
        plt.show()

if __name__ == "__main__":
    pass