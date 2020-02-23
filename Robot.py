import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
import time
import math as math

class Robot():
    '''Robot stores pose and history as well as physical parameters'''
    def __init__(
            self, name, color, wheel_seperation, wheel_diameter, 
            sample_time = 0.01, start_pose = [0, 0, 0]):

        self.name               = name 
        self.color              = color
        self.wheel_seperation   = wheel_seperation
        self.wheel_diameter     = wheel_diameter    # array of [r,l] diameters
        self.sample_time        = sample_time
        self.xi                 = start_pose        # pose of robot [x,y,theta]
        self.goal_pos           = 0
        self.pose_history       = [] 
        self.commands           = []
        self.l_speed            = []
        self.r_speed            = []
        self.actual_sim_time    = 0.0


    def set_command(self, func):
        ''' Set the current comand to a method in the class'''
        self.command = func

    def run(self):
        '''Runs the first command in the commands then removes it'''

        # check that there are commands left then set it as current command
        if len(self.commands):
            self.set_command(self.commands[0])

        # request the next variable from the command (generator)
        try:
            next(self.command)
            return 

        # catch the end of the generator and move to next command
        except StopIteration:
            if len(self.commands):
                self.commands.pop(0)
                if len(self.commands):
                    self.command = self.commands[0]
                else:
                    return
            else:
                return

    @property
    def pose(self):
        '''Returns the current pose (xi) [x, y, theta]'''
        return self.xi 

    def update_pose(self, phi_dot):
        ''' Calculate new pose given a wheel speed

            Ex3 - Kinematics - Q7

            xi          - pose [x,y,theta] (m,m,rad)
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
        self.actual_sim_time += self.sample_time
        return xi_new 

    def derivative(self, phi_dot):
        ''' Find the acceleration of the wheels

            xi          - pose [x,y,theta] (m,m,rad)
            phi_dot     - angular speed [0,1] (rad/s)
        '''
        _, _, theta     = self.xi
        r1, r2          = self.wheel_diameter

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

        xi_dot = np.dot(np.dot(rotation_inv, np.linalg.inv(constrains)),
                                input_speed)
        return xi_dot
    
    def forward(self, distance, speed):
        '''Drives the robot forward at a speed for set dist

            Ex3 - Kinematics - Q9   
        '''
        start_pose = self.xi    

        driven_dist = 0
        phi_dot = [0,0]
        phi_dot[0]  = (2 * speed / self.wheel_diameter[0]) 
        phi_dot[1]  = (2 * speed / self.wheel_diameter[1]) 
        
        while driven_dist <= distance:
            new_pose = self.update_pose(phi_dot)
            driven_dist = (np.sqrt(
                            (new_pose[0] - start_pose[0])**2 + (
                                new_pose[1] - start_pose[1])**2)) 
                    
            yield new_pose

    def turn(self, angle, theta_dot):
        '''Turns the robot around its centre at a given rotational velocity
            (angle in radian, theta_dot in radian/s)
            
            Ex3 - Kinematics - Q10
        '''
        prev_pose = self.xi

        turned_angle = 0
        phi_dot = [0,0]
        pos_angle_vel = self.wheel_seperation * theta_dot / self.wheel_diameter[0]
        phi_dot[0] =  pos_angle_vel if angle >= 0 else -pos_angle_vel # right wheel
        phi_dot[1] = -pos_angle_vel if angle >= 0 else  pos_angle_vel # left wheel
        
        while turned_angle < angle if angle >= 0 else turned_angle > angle:
            new_pose = self.update_pose(phi_dot)
            turned_angle += new_pose[2] - prev_pose[2]
            prev_pose = new_pose
            #print(f"{turned_angle} {angle}")
            yield new_pose

    def polar_coord(self):
        '''Returns the polar representation of displacemnt from ideal'''
        delta_x = self.goal_pos[0] - self.xi[0]
        delta_y = self.goal_pos[1] - self.xi[1]
        rho = np.sqrt(delta_x**2+delta_y**2)
        alpha = -self.xi[2] + math.atan2(delta_y,delta_x)
        beta = -math.atan2(delta_y,delta_x)
        return (rho, alpha, beta)

    def v_w(self):
        ''' Returns the Linear speed (v) and angular speed (w)'''
        rho, alpha, beta = self.polar_coord()
        kp = 3
        ka = 8
        kb = -1.5
        v = kp * rho
        w = ka * alpha + kb * beta
        return (v,w)
    
    def wheel_speeds(self):
        '''Convert the linear speeds to radial wheel speeds'''
        v, w = self.v_w()

        phi_dot = [0,0]
        phi_dot[0] = (
        1 / self.wheel_diameter[0] / 2 * (v + w * self.wheel_seperation / 2))

        phi_dot[1] = (
        1 / self.wheel_diameter[1]  / 2 * (v - w * self.wheel_seperation / 2))
        
        return phi_dot

    def go_to_pose(self, goal_pos):
        '''Yields the new pose which takes the robot towards goal'''
        self.goal_pos = goal_pos

        while   abs(self.goal_pos[0] - self.xi[0]) > 0.01 or \
                abs(self.goal_pos[1] - self.xi[1]) > 0.01 or \
                abs(self.goal_pos[2] - self.xi[2]) > 0.02:

            phi_dot = self.wheel_speeds()
            new_pose = self.update_pose(phi_dot)

            if phi_dot[0]*self.sample_time < 0.005 and \
                phi_dot[1]*self.sample_time < 0.005:
                break

            yield new_pose

if __name__ == "__main__":
    print("Running Robot.py")