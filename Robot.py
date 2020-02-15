import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
import time
import math as math

class Robot():
    def __init__(self, name, color, wheel_seperation, wheel_diameter, sample_time = 0.01, start_pose = [0, 0, 0]):
        self.name               = name 
        self.color              = color
        self.wheel_seperation   = wheel_seperation
        self.wheel_diameter     = wheel_diameter    # array of [r,l] diameters
        self.sample_time        = sample_time
        self.xi                 = start_pose        # pose of robot
        self.goal_pos           = 0
        self.pose_history       = [] 
        self.command_list       = []
        self.l_speed            = []
        self.r_speed            = []

    def set_commands(self, commands):
        self.commands = commands

    def run_mission(self):
        for command in self.commands:
            self._run(command)

    def _run(self, command):
        pass

    def get_pose(self):
        return self.xi 

    def update_pose(self, phi_dot):
        ''' xi          - pose [x,y,theta] (m,m,rad)
            w_disp      - (m)
            w_diam      - (m)
            phi_dot     - angular speed (rad/sec)
            time_step   - sampling speed (seconds)
        '''
        xi_new = self.xi + self.derivative(phi_dot) * self.sample_time
        self.xi = xi_new
        self.pose_history.append(xi_new)
        self.r_speed.append(phi_dot[0]*self.sample_time)
        self.l_speed.append(phi_dot[1]*self.sample_time)
        return xi_new 

    def derivative(self, phi_dot):
        ''' xi          - pose [x,y,theta] (m,m,rad)
            w_disp      - (m)
            w_diam      - (m)
            phi_dot     - angular speed [0,1] (rad/s)
        '''
        _, _, theta     = self.xi
        r1, r2          = self.wheel_diameter
        l               = self.wheel_seperation

        rotation = np.array(    [[np.cos(theta), np.sin(theta), 0],
                                [-np.sin(theta), np.cos(theta), 0],
                                [      0      ,      0        , 1]])

        rotation_inv = np.transpose(rotation)
        constrains = np.array([[1, 0, self.wheel_seperation/2],
                            [1, 0, -self.wheel_seperation/2],
                            [0, 1, 0]])
        input_speed = np.transpose(np.array( [r1*phi_dot[0], 
                                              r2*phi_dot[1],
                                              0]))

        xi_dot = np.dot(np.dot(rotation_inv, np.linalg.inv(constrains)),input_speed)
        return xi_dot
    
    def drive(self, distance, speed):
        start_pose = self.xi    # does it copy or assign?
        driven_dist = 0
        phi_dot = [0,0]
        phi_dot[0]  = 2 * speed / self.wheel_diameter[0]
        phi_dot[1]  = 2 * speed / self.wheel_diameter[1]
        while driven_dist <= distance:
            new_pose = self.update_pose(phi_dot)
            driven_dist += np.sqrt(new_pose[0]**2 + new_pose[1]**2)

    def polar_coord(self):
        delta_x = self.goal_pos[0] - self.xi[0]
        delta_y = self.goal_pos[1] - self.xi[1]
        rho = np.sqrt(delta_x**2+delta_y**2)
        alpha = -self.xi[2] + math.atan2(delta_y,delta_x)
        beta = -math.atan2(delta_y,delta_x)
        return (rho, alpha, beta)

    def v_w(self):
        rho, alpha, beta = self.polar_coord()
        kp = 3
        ka = 8
        kb = -1.5
        v = kp * rho
        w = ka * alpha + kb * beta
        return (v,w)
    
    def wheel_speeds(self):
        v, w = self.v_w()

        phi_dot = [0,0]
        phi_dot[0] = 1 / self.wheel_diameter[0] / 2 * (v + w * self.wheel_seperation / 2)
        phi_dot[1] = 1 / self.wheel_diameter[1] / 2 * (v - w * self.wheel_seperation / 2)
        
        return phi_dot

    def go_to_pose(self, goal_pos):
        self.goal_pos = goal_pos
        while   abs(self.goal_pos[0] - self.xi[0]) > 0.1 or \
                abs(self.goal_pos[1] - self.xi[1]) > 0.1 or \
                abs(self.goal_pos[2] - self.xi[2]) > 1:
            phi_dot = self.wheel_speeds()
            new_pose = self.update_pose(phi_dot)
            yield new_pose