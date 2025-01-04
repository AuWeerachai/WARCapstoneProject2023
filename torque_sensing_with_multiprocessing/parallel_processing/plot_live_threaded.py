import matplotlib.pyplot as plt
from plotting_import import *
from time import sleep, time
import numpy as np
import hebi

# Set up the loop to update the plot
startTime = time()
duration = 1
currentTime = time() - startTime
gui_running = True


#Multiprocessing parameters
if __name__ == '__main__':                  
    queue = mp.Queue()
    lock = mp.Lock()

    #torque_produce = mp.Process(target=torque_producer, args=(queue, lock, gui_running, 7))
 
    #Multiprocessing Start
    #torque_produce.start()
    #torque_consume = mp.Process(target=torque_consumer, args=(queue, lock, startTime))
    #torque_consume.daemon = True

    #torque_consume.start()
    #torque_produce.join()


    #to stop - see if there is a keyboard input w/ broadcast
    #pay attention to what is run by consumer or producer, everything has a perfect copy of namespace
        #consider finding ID for produce and consume, then write loop for if ID == 'ID'