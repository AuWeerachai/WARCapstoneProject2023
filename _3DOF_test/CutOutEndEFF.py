import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
import time 
from mpl_toolkits import mplot3d
from lines import line3D, curve3D

startT = time.time()
def AN1(theta):
    mat = sp.Matrix([[sp.cos(theta), -sp.sin(theta),0,0],
                        [sp.sin(theta),sp.cos(theta),0,0],
                        [0,0,1,0.05],
                        [0,0,0,1]])
    return mat


def A12(theta):
    mat = sp.Matrix([[sp.cos(theta), 0, -sp.sin(theta), 0],
                        [0,1,0,-0.07],
                        [sp.sin(theta),0,sp.cos(theta),0.07],
                        [0,0,0,1]])
    return mat

def A23(theta):
    mat = sp.Matrix([[sp.cos(theta), 0, -sp.sin(theta), 0.33],
                        [0,1,0,-0.07],
                        [sp.sin(theta),0,sp.cos(theta),0],
                        [0,0,0,1]])
    return mat

def A34():
    mat = sp.Matrix([[1, 0, 0, 0.33],
                        [0,1,0,-0.07],
                        [0,0,1,0],
                        [0,0,0,1]])
    return mat

    

def jacobi(b,thetas):
    r1 = []
    r2 = []
    r3 = []
    for i in range(0,sp.shape(thetas)[0]):
        var = sp.diff(b[0],thetas[i])
        r1.append(var)
    for i in range(0,sp.shape(thetas)[0]):
        var = sp.diff(b[1],thetas[i])
        r2.append(var)
    for i in range(0,sp.shape(thetas)[0]):
        var = sp.diff(b[2],thetas[i])
        r3.append(var)
    r1 = sp.Matrix([r1])
    r2 = sp.Matrix([r2])
    r3 = sp.Matrix([r3])

    Jn = sp.Matrix([[r1],[r2],[r3]])
    return (Jn)


def AttndPos(theta1, theta2, theta3):
    mat1 = AN1(theta1)
    mat2 = A12(theta2)
    mat3 = A23(theta3)
    mat4 = A34()

    AIE = mat1*mat2*mat3*mat4
    AIE.row_del(3)

    # a = AIE.col(0)
    # o = AIE.col(1)
    # n = AIE.col(2)
    p = AIE.col(3)
    
    # f = sp.Matrix([[a],[o],[n],[p]])

    # AIE.col_del(3)
    # CIE = AIE


    return (p) 

def bigJ(angv,p):
    # J1 = jacobi(a,angv)
    # J2 = jacobi(o,angv)
    # J3 = jacobi(n,angv)
    J4 = jacobi(p,angv)
    # J = sp.Matrix([[J1],[J2],[J3],[J4]])
    return J4

def tonumpy(m):
    numrows = sp.shape(m)[0]
    numcols =sp.shape(m)[1]
    nummy = np.zeros([numrows, numcols])
    for i in range(0,numcols):
        for j in range(0,numrows):
            nummy[j,i] = m[j,i]

    return nummy

#define our actuator angles as a vector of symbols
def calcX(thetaActual,XYZ):
    tht1,tht2,tht3 = sp.symbols('tht1 tht2 tht3', real = True)
    angs = sp.Matrix([[tht1],[tht2],[tht3]])
    #Determine forward Kinematics Parameters
    Forwards = AttndPos(tht1,tht2,tht3)
    # avec = Forwards[0]
    # ovec = Forwards[1]
    # nvec = Forwards[2]
    # pvec = Forwards[3]
    fmoment = Forwards
    # CIEmoment = Forwards[5]
    #Find the Jacobian of the matrix symbolically
    Jsym = bigJ(angs,fmoment)
    #Start Subbing in the real values from feedback
    ang1 = thetaActual[0,0]
    ang2 = thetaActual[1,0]
    ang3 = thetaActual[2,0]
    J = Jsym.subs(tht1,ang1)
    J = J.subs(tht2, ang2)
    J = J.subs(tht3,ang3)
    fold = fmoment.subs(tht1,ang1)
    fold = fold.subs(tht2,ang2)
    fold = fold.subs(tht3,ang3)
    # print(np.shape(fold))
    # fnew = fold
    # print(fnew)
    fnew = np.zeros([3,1])
    fnew[0]= XYZ[0]
    fnew[1]= XYZ[1]
    fnew[2]= XYZ[2]
    #put into numpy for easier calcs
    J = tonumpy(J)
    fold = tonumpy(fold)
    fnew = tonumpy(fnew)
    # do the actual calcs of X
    invJ = np.linalg.inv(J)
    X = np.matmul(invJ,(fnew - fold))
    return X



def checkforexit(tht,cmnd):
    check = abs(tht - cmnd)
    return (check[0,0]<tol and check[1,0]<tol and check[2,0]<tol)

def allcalcs(XYZ,start):
    cmnd = start
    loop = True
    # print(cmnd)
    anglepath = np.transpose(cmnd)
    # anglepath = np.matrix([0,0,0])
    while loop:
        X = calcX(cmnd,XYZ)
        thtnew = cmnd + X
        # logger = np.transpose(thtnew)
        # anglepath = np.append(anglepath,logger,axis = 0)
        if ((time.time() - startT ) > 5):
            print("Calcs no worky")
            return
        if checkforexit(thtnew,cmnd):
            loop = False
        
        else:
            cmnd = thtnew

    # anglepath = np.transpose(anglepath)
    return thtnew


def calculatedXYZ(finalangles):
    f = AttndPos(finalangles[0,0],finalangles[1,0],finalangles[2,0])
    f = np.matrix([[f[0]],[f[1]],[f[2]]])
    return f

def toXYZforPlot(allAng):
    XYZs = np.matrix([[0],[0],[0]])
    for i in range(np.shape(allAng)[1]):
        col = calculatedXYZ(allAng[:,i])
        XYZs = np.append(XYZs,col,1)
    XYZs = np.delete(XYZs,(0),1)    
    return XYZs
    


def armMovementPlot(angs,Curve = True):
    ToPlot = toXYZforPlot(angs)

    x = ToPlot[0]
    y = ToPlot[1]   
    z = ToPlot[2]

    xf = ToPlot[0,-1]
    yf = ToPlot[1,-1]
    zf = ToPlot[2,-1]

    fig = plt.figure(figsize = (10,10))
    ax = plt.axes(projection='3d')
    ax.grid()

    ax.scatter(ToPlot[0,0], ToPlot[1,0], ToPlot[2,0], c = 'r', s = 35)
    ax.scatter(0, 0, 0, c = 'black', s = 35)
    ax.scatter(x, y, z, c = 'y', s = 25, alpha = 0.8)
    ax.scatter(xf, yf, zf, c = 'b', s = 35)
    ax.scatter(desiredXYZ[0,0], desiredXYZ[1,0], desiredXYZ[2,0], c = 'b', s = 35)
    ax.set_title('Path of Arm')
    # Set axes label
    ax.set_xlabel('x', labelpad=20)
    ax.set_ylabel('y', labelpad=20)
    ax.set_zlabel('z', labelpad=20)
    if (Curve):
        desiredPath = curve3D(np.transpose(calculatedXYZ(Startangles)),np.transpose(desiredXYZ),NumberP, MaxA, False)
    else:
        desiredPath = line3D(np.transpose(calculatedXYZ(Startangles)),np.transpose(desiredXYZ),NumberP,False)
    x1, y1, z1 = np.split(desiredPath, 3, axis=1)
    ax.scatter(x1, y1, z1, color='green', marker='o', s=40)
    plt.show()
    return
  
def StraightLine(startA,end):
    start = np.transpose(calculatedXYZ(startA))
    linPoints = np.transpose(line3D(start,np.transpose(end),NumberP, False))
    size = np.shape(linPoints)[1]
    allAngs = startA
    for i in range(size-1):
        moves = allcalcs(linPoints[:,i+1], startA)
        allAngs = np.append(allAngs,moves,axis = 1)
        startA = moves[:,-1]
    return allAngs

def SmoothCurve(startA,end):
    start = np.transpose(calculatedXYZ(startA))
    linPoints = np.transpose(curve3D(start,np.transpose(end),NumberP, MaxA, False))
    size = np.shape(linPoints)[1]
    allAngs = np.matrix([[0],[0],[0]])
    for i in range(size-1):
        moves = allcalcs(linPoints[:,i+1], startA)
        allAngs = np.append(allAngs,moves,axis = 1)
        startA = moves[:,-1]
    allAngs = np.delete(allAngs, (0), axis=1)
    return allAngs

tol = 0.007
NumberP = 20
MaxA = 0.2
Startangles = np.matrix([[1.4],[1.14],[-2.5]])
desiredXYZ = np.matrix([[0.05], [0.25], [0.1]])
# pos = (SmoothCurve(Startangles, desiredXYZ))
# # pos = (StraightLine(Startangles, desiredXYZ))
# end = time.time()
# duration = end - startT
# print(duration,"seconds")
# newXYZ = calculatedXYZ(pos[:,-1])
# diff = desiredXYZ - newXYZ
# print(diff)
# armMovementPlot(pos,True)

sumtime = 0
sumdist = 0
sumx = 0
sumy = 0
sumz = 0

for i in range(10):
    startT = time.time()
    # pos = (SmoothCurve(Startangles, desiredXYZ))
    pos = (StraightLine(Startangles, desiredXYZ))
    end = time.time()
    duration = end - startT
    sumtime = sumtime + duration
    print(duration,"seconds")
    newXYZ = calculatedXYZ(pos[:,-1])
    diff = desiredXYZ - newXYZ
    diffx = np.float64(diff[0,0])
    diffy = np.float64(diff[1,0])
    diffz = np.float64(diff[2,0])
    dist = np.sqrt(diffx*diffx+diffy*diffy+diffz*diffz)
    sumx = sumx + diffx
    sumy = sumy + diffy
    sumz = sumz + diffz
    sumdist = sumdist + dist
    # print(diff)+

avetime = sumtime/10
avex = sumx/10
avey = sumy/10
avez = sumz/10
avedist = sumdist/10
print("average distance from target: ", avedist)
print("average Time for rexecution is:", avetime)