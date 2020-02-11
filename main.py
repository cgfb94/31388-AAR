import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
import time

from Robot import Robot
from Simulator import Simulator

def simulator():
    carla = Robot( 0.26, [0.035, 0.035], 0.01, [0, 0, 0])
    rob_sim = Simulator()
    rob_sim.live_sim(carla, [0, 5000.86], 100)
    #rob_sim.simulate(carla, [2.86, -2.86], 2)
    #print(rob_sim.pose_history)

if __name__ == '__main__':
    simulator()
