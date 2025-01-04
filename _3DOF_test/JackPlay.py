import numpy as np
import matplotlib.pyplot as plt

maxH = 20
tol = 0.00001
x1,z1 = 1,2
x2,z2 = -3,6

def plotpath(scale,A,dx,p1,p2,plotter =  True):
    f = 1/(2*dx)
    x = np.linspace(p1[0],p2[0],50)
    k = 2*np.pi*f*scale
    path = A*np.sin((x - p1[0])*k) + p1[1]
    if(plotter):
        fig = plt.figure()
        ax = fig.add_subplot()
        ax.plot(x, path, color='blue', linestyle='dashed', linewidth=2, alpha=0.5)
        ax.scatter([p1[0],p2[0]], [p1[1],p2[1]], color='black', marker='o', s=40)
        ax.set_xlabel('x')
        ax.set_ylabel('z')
        plt.show()
    return path

def plotPoly(x,y,plotter = True):
    fig = plt.figure()
    ax = fig.add_subplot()
    guess = np.polyfit(x,y,(np.shape(x)[0]-1))
    p = np.poly1d(guess)
    xp = np.linspace(-1, 1, 80)
    tang = -0.3183098864122*np.arcsin(xp) + 1
    if(plotter):
        plt.plot(xp, p(xp), '-')
        ax.plot(xp, tang, color='red', linestyle='dashed', linewidth=2)
        ax.scatter(x, y, color='green', marker='o', s=40)
        ax.set_xlabel('delz/A')
        ax.set_ylabel('Scale')
        plt.show()
    return p

def findSols(rat,tolerance,A,dx,p1,p2):
    fc = 1/(2*dx)
    scalec = 1/2
    adj = 0.1
    zf = rat*A + z1
    loop = True
    while(loop):
        kc = 2*np.pi*fc*scalec
        curveEnd = A*np.sin((p2[0] - p1[0])*kc) + p1[1]
        diff = curveEnd - zf
        if(abs(diff) < tolerance):
            loop = False
        elif(diff < 0):
            scalec = scalec - adj
            adj = adj*0.1
        else:
            scalec = scalec + adj
    return scalec, diff

def popAnalycal(a,dx,p1,p2):
    rat = np.array([-1,1])
    sa = np.array([3/2,1/2])
    for i in range(98):
        r = -0.98 + 0.02*i
        point = findSols(r,tol,a,dx,p1,p2)[0]
        sa = np.append(sa,point)
        rat = np.append(rat,r)
    return rat,sa

def findSinPath(p1,p2,plots = True):
    A = (maxH - p1[1])
    dx = p2[0]-p1[0]
    dz = p2[1]-p1[1]
    ratio = dz/A
    dzOa,s = popAnalycal(A,dx,p1,p2)
    eq = plotPoly(dzOa,s,plots)
    path = plotpath(eq(ratio),A,dx,p1,p2,plots)
    return path


a = np.matrix([[1,2,3,4],[2,3,4,5],[3,4,5,6],[4,5,6,7]])
print(a)
b = a[1:,]
c = -a[1,:]
print(c)
d = np.insert(a,2,c,axis = 0)
print(d)