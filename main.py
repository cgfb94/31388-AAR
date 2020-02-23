import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
import time

from Simulator import Simulator

def simulator():
    '''Creates a Simulator instance creates robots and runs sim'''

    rob_sim = Simulator()

    # add carla robot to sim and set mission
    rob_sim.add_robot("r0", 'tab:blue', 0.26, [0.035, 0.035], 0.01, [0, 0, 0])
    r0 = rob_sim.robots["r0"]
    rob_sim.set_commands(
        "r0", [ 
            r0.go_to_pose([0.5,0,0])
        ])

    rob_sim.add_robot("r1", 'tab:orange', 0.26, [0.035, 0.035], 0.01, [0, 0, 0])
    r1 = rob_sim.robots["r1"]
    rob_sim.set_commands(
        "r1", [ 
            r1.go_to_pose([0.5,0.5,0])
        ])

    rob_sim.add_robot("r2", 'tab:green', 0.26, [0.035, 0.035], 0.01, [0, 0, 0])
    r2 = rob_sim.robots["r2"]
    rob_sim.set_commands(
        "r2", [ 
            r2.go_to_pose([0.5,-0.5,-np.pi/2])
        ])
    
    rob_sim.add_robot("r3", 'tab:red', 0.26, [0.035, 0.035], 0.01, [0, 0, 0])
    r3 = rob_sim.robots["r3"]
    rob_sim.set_commands(
        "r3", [ 
            r3.go_to_pose([0,0.5,-np.pi])
        ])

    rob_sim.add_robot("r4", 'tab:purple', 0.26, [0.035, 0.035], 0.01, [0, 0, 0])
    r4 = rob_sim.robots["r4"]
    rob_sim.set_commands(
        "r4", [ 
            r4.go_to_pose([0, 0, np.pi/2])
        ])

    rob_sim.add_robot("r5", 'tab:brown', 0.26, [0.035, 0.035], 0.01, [0, 0, 0])
    r5 = rob_sim.robots["r5"]
    rob_sim.set_commands(
        "r5", [ 
            r5.go_to_pose([0, -0.5, 0])
        ])

    rob_sim.add_robot("r6", 'tab:pink', 0.26, [0.035, 0.035], 0.01, [0, 0, 0])
    r6 = rob_sim.robots["r6"]
    rob_sim.set_commands(
        "r6", [ 
            r6.go_to_pose([-0.5, 0.5, 0])
        ])

    rob_sim.add_robot("r7", 'tab:olive', 0.26, [0.035, 0.035], 0.01, [0, 0, 0])
    r7 = rob_sim.robots["r7"]
    rob_sim.set_commands(
        "r7", [ 
            r7.go_to_pose([-0.5, 0, 0])
        ])

    rob_sim.add_robot("r8", 'tab:cyan', 0.26, [0.035, 0.035], 0.01, [0, 0, 0])
    r8 = rob_sim.robots["r8"]
    rob_sim.set_commands(
        "r8", [ 
            r8.go_to_pose([-0.5, -0.5, -np.pi])
        ])




    # # add nikki robot to sim and set mission
    # rob_sim.add_robot("nikki", 'r', 0.26, [0.035, 0.035], 0.01, [8, 12,-np.pi])
    # nikki = rob_sim.robots["nikki"]
    # rob_sim.set_commands(
    #     "nikki", [
    #         nikki.forward(10,15),
    #         nikki.go_to_pose([15,-5,-np.pi]),
    #         nikki.forward(5, -25)
    #     ])

    # # add marc robot to sim and set mission
    # rob_sim.add_robot("marc", 'b', 0.26, [0.035, 0.035], 0.01, [-8, 6,np.pi])
    # marc = rob_sim.robots["marc"]
    # rob_sim.set_commands(
    #     "marc", [
    #         marc.forward(10,-15),
    #         marc.go_to_pose([15,-5,-np.pi]),
    #         marc.forward(5, -25),
    #         marc.go_to_pose([-15,-5,-np.pi]),
    #     ])

    # run the simulation with the given missions
    rob_sim.sim_missions(500)

if __name__ == '__main__':
    simulator()