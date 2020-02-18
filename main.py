import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
import time

from Simulator import Simulator

def simulator():
    '''Creates a Simulator instance creates robots and runs sim'''

    rob_sim = Simulator()

    # add carla robot to sim and set mission
    rob_sim.add_robot("carla", 'g', 0.26, [0.035, 0.035], 0.01, [0, 0, 0])
    carla = rob_sim.robots["carla"]
    rob_sim.set_commands(
        "carla", [
            carla.forward(10,10), 
            carla.forward(15,-10),
            carla.go_to_pose([5,5,-1]), 
            carla.go_to_pose([-10,-10, 1])])

    # add nikki robot to sim and set mission
    rob_sim.add_robot("nikki", 'r', 0.26, [0.035, 0.035], 0.01, [8, 12,-np.pi])
    nikki = rob_sim.robots["nikki"]
    rob_sim.set_commands(
        "nikki", [
            nikki.forward(1,1)
        ])

    # run the simulation 
    rob_sim.sim_missions(500)

if __name__ == '__main__':
    simulator()