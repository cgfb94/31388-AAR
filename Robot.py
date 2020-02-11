import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
import time

class Robot():
    def __init__(self, wheel_seperation, wheel_diameter, sample_time = 0.01, start_pose = [0, 0, 0]):
        self.wheel_seperation   = wheel_seperation
        self.wheel_diameter     = wheel_diameter    # array of [r,l] diameters
        self.sample_time        = sample_time
        self.xi                 = start_pose

    def euler(self, phi_dot):
        ''' xi          - pose [x,y,theta] (m,m,rad)
            w_disp      - (m)
            w_diam      - (m)
            phi_dot     - angular speed (rad/sec)
            time_step   - sampling speed (seconds)
        '''
        xi_new = self.xi + self.derivative(phi_dot) * self.sample_time

        self.xi = xi_new
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

        rotation_inv = np.array(    [[np.cos(theta), -np.sin(theta), 0],
                                    [np.sin(theta), -np.cos(theta), 0],
                                    [      0      ,      0        , 1]])

        rotation_vec = np.transpose(np.array( [r1*phi_dot[0]/2 + r2*phi_dot[1]/2, 
                                                0                 ,
                                    	        r1*phi_dot[0]/2*l - r2*phi_dot[1]/2*l]))

        xi_dot = np.dot(rotation_inv, rotation_vec)
        return xi_dot