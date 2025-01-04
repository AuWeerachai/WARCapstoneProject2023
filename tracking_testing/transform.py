'''
from: https://automaticaddison.com/how-to-convert-camera-pixels-to-robot-base-frame-coordinates/
'''

import numpy as np
import matplotlib.pyplot as plt

def main():

    newL = []
    finalL = []

    with open('coordinate.txt','r') as read_file:
        for line in read_file:
            line = line[1:-2]
            line = line.split(", ")
            for i in line:
                newL.append(float(i))
            finalL.append(newL)
            newL = []

    for i in finalL:
        print(f'({0.27 - i[1]}, {-i[0] + 0.07})')

if __name__ == '__main__':
    main()