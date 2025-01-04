import os
import multiprocessing as mp
import numpy as np
import matplotlib.pyplot as plt
from time import sleep, time

# Line parameters for live plot
def live_plot_torque_params():
    plt.ion()
    fig = plt.figure(figsize=(13,6))
    ax = fig.add_subplot(111)

    motor_list = []
    for i in range(8):
        motor_list.append(ax.plot([], [], label="m{}".format(i+1))[0])

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Torque (N-m)')
    ax.set_title('Live Torque Plot')
    plt.legend()
    plt.show()

    return motor_list, ax

# The consumer function takes data off of the Queue
def consumer(queue, lock, startTime, motor_list, ax):
    # Synchronize access to the console
    with lock:
        print('Starting consumer => {}'.format(os.getpid()))
    # Run indefinitely
    while True:
        sleep(.1)
        # If the queue is empty, queue.get() will block until the queue has data
        queue_value = queue.get()
        currentTime = time() - startTime
        
        # Plot the values in the queue
        live_plot_torque_loop(currentTime, motor_list, ax, queue_value)

# The producer function puts data onto the Queue
def producer(queue, lock, input_arr, stop_flag):
    with lock:
        print('Starting producer => {}'.format(os.getpid()))  # Prints unique ID for process
    # Place values in the Queue
    while not stop_flag.is_set():
        sleep(.1)
        queue.put(input_arr)
    # Synchronize access to the console
    with lock:
        print('Producer {} exiting...'.format(os.getpid()))

# Function for updating the live plot
def live_plot_torque_loop(elapsed_time, motor_list, ax, efforts):
    # Simulate measuring torque in a while loop
    for index, motor in enumerate(motor_list):
        motor.set_xdata(np.append(motor.get_xdata(), elapsed_time))
        motor.set_ydata(np.append(motor.get_ydata(), efforts[index]))

    ax.legend(motor_list, ['m1', 'm2', 'm3', 'm4', 'm5', 'm6', 'm7', 'm8'])

    # Update the plot
    plt.xlim([0, elapsed_time])
    plt.ylim([-10, 10])
    plt.draw()
    plt.pause(0.0001)

# Multiprocessing parameters
if __name__ == '__main__':
    queue = mp.Queue()
    lock = mp.Lock()
    stop_flag = mp.Event()

    motor_list, ax = live_plot_torque_params()

    # Multiprocessing Start
    produce = mp.Process(target=producer, args=(queue, lock, np.array([1,2,3,4,-2,-3,-4,5]), stop_flag))
    consume = mp.Process(target=consumer, args=(queue, lock, time(), motor_list, ax))
    consume.daemon = True

    produce.start()
    consume.start()

    # # Run the plot while the producer is still running
    # while produce.is_alive():
    #     try:
    #         plt.pause(0.1)
    #     except KeyboardInterrupt:
    #         print('Plot interrupted...')
    #         stop_flag.set()
    #         break

    # Join the processes
    produce.join()

    # Close the plot
    plt.show()
