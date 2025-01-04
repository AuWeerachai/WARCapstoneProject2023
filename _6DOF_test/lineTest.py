
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np

def line2D(a, b, n=5, graph=False):
    # creates n points evenly spaced along a line from point a to b
    # in 2 dimensions
    result = a
    x_split = (b[0,0] - a[0,0]) / n
    y_split = (b[0,1] - a[0,1]) / n
    for i in range(n):
        new_point = np.array([[a[0,0] + x_split * (i+1), a[0,1] + y_split * (i+1)]])
        result = np.append(result, new_point, axis=0)
    if graph:
        result = np.asarray(result)
        # print(result)
        x, y = np.split(result, 2, axis=1)
        fig = plt.figure()
        ax = fig.add_subplot()
        ax.plot(x, y, color='blue', linestyle='dashed', linewidth=2, alpha=0.5)
        ax.scatter(x, y, color='green', marker='o', s=40)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        plt.show()
    return result

def line3D(a, b, n, graph=False):
    # creates n points evenly spaced along a line from point a to b
    # in 3 dimensions
    result = a
    x_split = (b[0,0] - a[0,0]) / n
    y_split = (b[0,1] - a[0,1]) / n
    z_split = (b[0,2] - a[0,2]) / n
    for i in range(n):
        new_point = np.matrix([[a[0,0] + x_split * (i+1), a[0,1] + y_split * (i+1), a[0,2] + z_split * (i+1)]])
        result = np.append(result, new_point, axis=0)

    if graph:
        result = np.asarray(result)
        # print(result)
        x, y, z = np.split(result, 3, axis=1)
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.plot3D(x, y, z, color='blue', linestyle='dashed', linewidth=2, alpha=0.5)
        ax.scatter(x, y, z, color='green', marker='o', s=40)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')           
        plt.show()
    return result

def curve3D(a, b, n, maxH, graph):
    A = (maxH - a[0,2])
    A = 0.5
    a = np.float64(a)
    b = np.float64(b)
    diff = b-a
    dx = diff[0,0]
    dz = diff[0,2]
    ratio = dz/A
    ratio = ratio
    scale = -0.3183098864122*np.arcsin(ratio) + 1
    f = 1/(2*dx)
    k = 2*np.pi*f*scale
    x = np.linspace(a[0,0],b[0,0],n)
    y = np.transpose(np.matrix(np.linspace(a[0,1],b[0,1],n)))
    z = A*np.sin((x - a[0,0])*k) + a[0,2]
    x = np.transpose(np.matrix(x))
    z = np.transpose(np.matrix(z))
    result = np.append(x,y,axis = 1)
    result = np.append(result,z,axis = 1)
    if graph:
        result = np.asarray(result)
        x, y, z = np.split(result, 3, axis=1)
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.plot3D(x, y, z, color='blue', linestyle='dashed', linewidth=2, alpha=0.5)
        ax.scatter([a[0,0],b[0,0]], [a[0,1],b[0,1]], [a[0,2],b[0,2]], color='green', marker='o', s=40)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        plt.show()
    return result

if __name__ == '__main__':
    p1 = np.matrix([0, 0, 0])
    p2 = np.matrix([0.92, 0.69, 4.2])
    # points2d = line2D(p1, p2, n=5, graph=True)
    # p3 = np.array([1.0, 2.0, 3.0])
    # p4 = np.array([4.0, 5.0, 6.0])
    points3d = line3D(p1, p2, 10, False)
    print(points3d)
