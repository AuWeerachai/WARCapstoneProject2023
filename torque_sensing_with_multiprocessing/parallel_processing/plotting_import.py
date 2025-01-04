import matplotlib.pyplot as plt
import multiprocessing as mp
from time import sleep, time
import numpy as np
import hebi
import os

#Populate Hebi
lookup = hebi.Lookup()
sleep(2.0)

family_name = "Arm"
module_names = ["m1","m2","m3", "m5", "m6", "m7", "m8"]
group = lookup.get_group_from_names([family_name], module_names)

if group is None:
    print('Group not found: Did you forget to set the module family and name above?')
    exit(1)

#Define HEBI
num_joints = group.size
group_feedback = hebi.GroupFeedback(num_joints)
if group.get_next_feedback(reuse_fbk=group_feedback) is None:
    print('Error getting feedback.')
    exit(1)


while True:
    sleep(.1)
    group.get_next_feedback(reuse_fbk=group_feedback)
    torque_arr = hebi.GroupFeedback(num_joints).effort
    print(torque_arr)


def live_plot_torque_params():
    # Create the figure and line object for the torque data
    fig, ax = plt.subplots()
    result = [ax.plot([], [], 'o-')[0] for _ in range(3)]       #changed n
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Torque [N-m]')
    return result, ax


def torque_producer(queue, lock, run_production, num_joints):
    with lock:                                                              #Synchronize access to the console
        print('Starting producer => {}'.format(os.getpid()))                #initalize anything else in with lock loop
    # Place values in the Queue
    while run_production:
        torque_arr = [1, 1, 2]
        queue.put(torque_arr)
        print(queue.get())
    # Synchronize access to the console
    with lock:
        print('Producer {} exiting...'.format(os.getpid()))


 
# The consumer function takes data off of the Queue
def torque_consumer(queue, lock, startTime):
    # Synchronize access to the console
    with lock:
        print('Starting consumer => {}'.format(os.getpid()))
        motor_list, ax = live_plot_torque_params()
    # Run indefinitely
    while True:
        sleep(.1)
        # If the queue is empty, queue.get() will block until the queue has data
        queue_value = queue.get()
        currentTime = time() - startTime

        # Plot the values in the queue
        live_plot_torque_loop(currentTime, motor_list, ax, queue_value)




def live_plot_torque_loop(elapsed_time, motor_list, ax, efforts):
    # Simulate measuring torque in a while loop
    for index, motor in enumerate(motor_list):
        motor.set_xdata(np.append(motor.get_xdata(), elapsed_time))
        motor.set_ydata(np.append(motor.get_ydata(), efforts[index]))

    ax.legend(motor_list, ['m1', 'm2', 'm3'])

    # Update the plot
    if elapsed_time < 2:
        plt.xlim([0, elapsed_time])
    else:
        plt.xlim([elapsed_time-2, elapsed_time]) 
    plt.ylim([-10, 10])
    plt.draw()
    plt.pause(0.0001)
