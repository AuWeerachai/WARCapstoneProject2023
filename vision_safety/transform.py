'''
from: https://automaticaddison.com/how-to-convert-camera-pixels-to-robot-base-frame-coordinates/
'''

import numpy as np
import matplotlib.pyplot as plt

import pyrealsense2 as rs

def main():

    newL = []
    finalL = []

    with open('coordinate.txt','r') as read_file:
        for line in read_file:
            line = line[2:-3]
            line = line.split(", ")
            for i in line:
                newL.append(float(i))
            finalL.append(newL)
            newL = []

    for i in finalL:
        print(f'({-i[0]}, {0.20 + i[1]})')

# rs.rs2_project_point_to_pixel

if __name__ == '__main__':
    main()