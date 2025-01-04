import matplotlib.pyplot as plt
from time import sleep, time
import numpy as np
import hebi

def torqueParams():
    
    effort_actual, velocity_actual, position_actual = np.zeros((1,group.size)), np.zeros((1,group.size)), np.zeros((1,group.size)) 
    safeTorque = True        

    return effort_actual, velocity_actual, position_actual, safeTorque

