import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
import time

from Simulator import Simulator

def simulator():
    '''Creates a Simulator instance creates robots and runs sim'''
    rob_sim = Simulator()
    rob_sim.add_robot("carla", 'g', 0.26, [0.035, 0.035], 0.01, [0, 0, 0])

    #rob_sim.robots["carla"].set_commands([rob_sim.robots["carla"].go_to_pose([10, 10, 1])])
    #rob_sim.add_robot("nikki", 'r', 0.26, [0.035, 0.035], 0.01, [8, 12,-np.pi])
    #rob_sim.add_robot("marc", 'b', 0.5, [0.5, 0.5], 0.01, [-6, -8, np.pi])
    rob_sim.test_sim(60)

if __name__ == '__main__':
    simulator()