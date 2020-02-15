import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
import time

from Robot import Robot
from Simulator import Simulator

def simulator():
    rob_sim = Simulator()
    rob_sim.add_robot("carla", 'g', 0.26, [0.035, 0.035], 0.01, [0, 0, 0])
    rob_sim.add_robot("nikki", 'r', 0.26, [0.035, 0.035], 0.01, [8, 12, -np.pi])
    rob_sim.add_robot("marc", 'b', 0.26, [0.035, 0.035], 0.01, [-6, -8, np.pi])
    rob_sim.live_sim(60)

if __name__ == '__main__':
    simulator()