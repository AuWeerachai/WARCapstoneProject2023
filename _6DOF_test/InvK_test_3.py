import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from mpl_toolkits import mplot3d
import time 
from lines import line3D, curve3D

L1 =0.1079 #could also be 0.04
L2 = 0

def AN1(theta):
    mat = sp.Matrix([[sp.cos(theta), sp.sin(theta),0,0],
                        [-sp.sin(theta),sp.cos(theta),0,0],
                        [0,0,1,0.1079],
                        [0,0,0,1]])
    return mat


def A12(theta):
    mat = sp.Matrix([[sp.cos(theta), 0, sp.sin(theta), 0],
                        [0,1,0,0],
                        [-sp.sin(theta),0,sp.cos(theta),0.07],
                        [0,0,0,1]])
    return mat

def A23(theta):
    mat = sp.Matrix([[1, 0, 0, 0.105],
                        [0,sp.cos(theta),-sp.sin(theta),0],
                        [sp.sin(theta),0,sp.cos(theta),0],
                        [0,0,0,1]])
    return mat

def A34(theta):
    mat = sp.Matrix([[sp.cos(theta), 0, -sp.sin(theta), 0.33],
                        [0,1,0,0],
                        [-sp.sin(theta),0,sp.cos(theta),0],
                        [0,0,0,1]])
    return mat


def A45(theta):
    mat = sp.Matrix([[sp.cos(theta), 0, -sp.sin(theta), 0.33],
                        [0,1,0,-0.115],
                        [sp.sin(theta),0,sp.cos(theta),0],
                        [0,0,0,1]])
    return mat

def A56(theta):
    mat = sp.Matrix([[1, 0, 0, 0.15],
                        [0,sp.cos(theta),-sp.sin(theta),-0.07],
                        [0,sp.sin(theta),sp.cos(theta),0],
                        [0,0,0,1]])
    return mat

def AttndPos(thetas):
    mat1 = AN1(thetas[0,0])
    mat2 = A12(thetas[1,0])
    mat3 = A23(thetas[2,0])
    mat4 = A34(thetas[3,0])
    mat5 = A45(thetas[4,0])
    mat6 = A56(thetas[5,0])

    AIE = mat1*mat2*mat3*mat4*mat5*mat6
    AIE.row_del(3)

    a = AIE.col(0)
    o = AIE.col(1)
    n = AIE.col(2)
    p = AIE.col(3)
    
    f = sp.Matrix([[a],[o],[n],[p]])

    AIE.col_del(3)
    CIE = AIE


    return (a,o,n,p,f,CIE) 

def tonumpy(m):
    numrows = sp.shape(m)[0]
    numcols =sp.shape(m)[1]
    nummy = np.zeros([numrows, numcols])
    for i in range(0,numcols):
        for j in range(0,numrows):
            nummy[j,i] = m[j,i]

    return nummy

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

def bigJ(angv,a,o,n,p):
    J1 = jacobi(a,angv)
    J2 = jacobi(o,angv)
    J3 = jacobi(n,angv)
    J4 = jacobi(p,angv)
    J = sp.Matrix([[J1],[J2],[J3],[J4]])
    return J

def calcX_fdes(thetaActual,fdesired):
    tht1,tht2,tht3,tht4,tht5,tht6 = sp.symbols('tht1 tht2 tht3 tht4 tht5 tht6', real = True)
    angs = sp.Matrix([[tht1],[tht2],[tht3],[tht4],[tht5],[tht6]])
    #Determine forward Kinematics Parameters
    Forwards = AttndPos(angs)
    avec = Forwards[0]
    ovec = Forwards[1]
    nvec = Forwards[2]
    pvec = Forwards[3]
    fmoment = Forwards[4]
    CIEmoment = Forwards[5]
    #Find the Jacobian of the matrix symbolically (analytically)
    Jsym = bigJ(angs,avec,ovec,nvec,pvec)
    #Start Subbing in the real values from feedback
    ang1 = thetaActual[0,0]
    ang2 = thetaActual[1,0]
    ang3 = thetaActual[2,0]
    ang4 = thetaActual[3,0]
    ang5 = thetaActual[4,0]
    ang6 = thetaActual[5,0]
    J = Jsym.subs(tht1,ang1)
    J = J.subs(tht2, ang2)
    J = J.subs(tht3,ang3)
    J = J.subs(tht4,ang4)
    J = J.subs(tht5, ang5)
    J = J.subs(tht6,ang6)
    fold = fmoment.subs(tht1,ang1)
    fold = fold.subs(tht2,ang2)
    fold = fold.subs(tht3,ang3)
    fold = fold.subs(tht4,ang4)
    fold = fold.subs(tht5,ang5)
    fold = fold.subs(tht6,ang6)

    fdes = np.zeros([12,1])
    fdes[0]= fdesired[0]
    fdes[1]= fdesired[1]
    fdes[2]= fdesired[2]
    fdes[3]= fdesired[3]
    fdes[4]= fdesired[4]
    fdes[5]= fdesired[5]
    fdes[6]= fdesired[6]
    fdes[7]= fdesired[7]
    fdes[8]= fdesired[8]
    fdes[9]= fdesired[9]
    fdes[10]= fdesired[10]
    fdes[11]= fdesired[11]
    #put into numpy for easier calcs
    J = tonumpy(J)
    fold = tonumpy(fold)
    fdes = tonumpy(fdes)
    # do the actual calcs of X
    invJ = np.linalg.pinv(J)
    X = np.matmul(invJ,(fdes - fold))
    return X


def calcX_XYZ(thetaActual,XYZdesired):

    tht1,tht2,tht3,tht4,tht5,tht6 = sp.symbols('tht1 tht2 tht3 tht4 tht5 tht6', real = True)
    angs = sp.Matrix([[tht1],[tht2],[tht3],[tht4],[tht5],[tht6]])
    #Determine forward Kinematics Parameters
    pvec = AttndPos(angs)[3]

    #Find the Jacobian of the matrix symbolically (analytically)
    J4 = jacobi(pvec,angs)
    Jsym = sp.Matrix([J4])

    #Start Subbing in the real values from feedback
    ang1 = thetaActual[0,0]
    ang2 = thetaActual[1,0]
    ang3 = thetaActual[2,0]
    ang4 = thetaActual[3,0]
    ang5 = thetaActual[4,0]
    ang6 = thetaActual[5,0]
    J = Jsym.subs(tht1,ang1)
    J = J.subs(tht2, ang2)
    J = J.subs(tht3,ang3)
    J = J.subs(tht4,ang4)
    J = J.subs(tht5, ang5)
    J = J.subs(tht6,ang6)
    fold = pvec.subs(tht1,ang1)
    fold = fold.subs(tht2,ang2)
    fold = fold.subs(tht3,ang3)
    fold = fold.subs(tht4,ang4)
    fold = fold.subs(tht5,ang5)
    fold = fold.subs(tht6,ang6)

    fdes = np.zeros([3,1])
    fdes[0]= XYZdesired[0]
    fdes[1]= XYZdesired[1]
    fdes[2]= XYZdesired[2]

    #put into numpy for easier calcs
    J = tonumpy(J)
    fold = tonumpy(fold)
    fdes = tonumpy(fdes)
    # do the actual calcs of X
    invJ = np.linalg.pinv(J)
    X = np.matmul(invJ,(fdes - fold))
    return X

def checkforexit(tht,cmnd,tol):
    check = abs(tht - cmnd)
    return (check[0,0]<tol and check[1,0]<tol and check[2,0]<tol and check[3,0]<tol and check[4,0]<tol and check[5,0]<tol)

def allcalcs(fdes,start,tol):
    cmnd = start
    loop = True
    # anglepath = np.matrix([0,0,0,0,0,0])
    anglepath = np.transpose(start)
    while loop:
        if(np.shape(fdes)[0]==12):
            X = calcX_fdes(cmnd,fdes)
        elif(np.shape(fdes)[0]==3):
            X = calcX_XYZ(cmnd,fdes)
        else:
            print("Desired f matrix has impossible dimensions")

        thtnew = cmnd + X
        logger = np.transpose(thtnew)
        anglepath = np.append(anglepath,logger,axis = 0)
        if checkforexit(thtnew,cmnd,tol):
            loop = False
        
        else:
            cmnd = thtnew
    #anglepath = np.delete(anglepath, (0), axis=0) #uncomment this
    anglepath = np.transpose(anglepath)
    # print("Done Calculating Path")
    return anglepath
    #return np.transpose(logger) #uncomment this to return only the "correct" points

def calculatedXYZ(finalangles):
    f = AttndPos(finalangles)[4]
    #print(f)
    f = np.matrix([[f[9]],[f[10]],[f[11]]])
    return f



def StraightLine(startA,end,tol,numP):
    start = np.transpose(calculatedXYZ(startA))
    #  print(start)
    if(np.shape(end)[0]<12):
        linPoints = np.transpose(line3D(start,np.transpose(end),numP, False))
    else:
        linPoints = np.transpose(line3D(start,np.transpose(end[-3:,:]),numP, False))
        # print(linPoints[:,1])
    size = np.shape(linPoints)[1]
    allAngs = startA
    for i in range(size-1):
        moves = allcalcs(linPoints[:,i+1], startA,tol)
        allAngs = np.append(allAngs,moves,axis = 1)
        startA = moves[:,-1]   
    return allAngs

def toXYZforPlot(allAng):
    XYZs = np.matrix([[0],[0],[0]])
    for i in range(np.shape(allAng)[1]):
        col = calculatedXYZ(allAng[:,i])
        XYZs = np.append(XYZs,col,1)
    XYZs = np.delete(XYZs,(0),1)    
    return XYZs

def armMovementPlot(angs):
    ToPlot = toXYZforPlot(angs)

    x = ToPlot[0]
    y = ToPlot[1]   
    z = ToPlot[2]

    # xf = ToPlot[0,-1]
    # yf = ToPlot[1,-1]
    # zf = ToPlot[2,-1]

    fig = plt.figure(figsize = (10,10))
    ax = plt.axes(projection='3d')
    ax.grid()
    # alphas = np.linspace(0.2, 1, np.shape(x)[1])
    colors = cm.rainbow(np.linspace(0, 1, np.shape(x)[1]))
    ax.scatter(ToPlot[0,0], ToPlot[1,0], ToPlot[2,0], c = 'r', s = 35, marker = "1")
    ax.scatter(0, 0, 0, c = 'black', s = 35)
    ax.scatter(x, y, z, c = colors, s = 25, alpha = 1)
    ax.scatter(desiredXYZ[0,0], desiredXYZ[1,0], desiredXYZ[2,0], c = 'g', s = 35, marker = "1")
    ax.set_title('Path of Arm')
    
    # Set axes label
    ax.set_xlabel('x', labelpad=20)
    ax.set_ylabel('y', labelpad=20)
    ax.set_zlabel('z', labelpad=20)
    plt.show()
    return

def addGimbal(ang, XYZ, tol, numP):
    origPos = StraightLine(ang, XYZ,tol,numP)
    newPos = origPos
    newRow = -1*newPos[1,:]
    newPos = np.insert(newPos,2,newRow,axis = 0)
    ## Add pi/2 to gimbal 5 to correct it a bit
    # for col in range(np.shape(newPos)[1]):
    #     newPos[3,col]=newPos[3,col]-(0.5*np.pi)  
    for col in range(np.shape(newPos)[1]):
        newPos[4,col]=newPos[4,col]*1
    for col in range(np.shape(newPos)[1]):
        newPos[5,col]=newPos[5,col]*1  
    print(origPos)
    print(newPos)
    return newPos, origPos

if __name__ == '__main__':
    start = time.time()
    Tol = 0.01
    NumberP = 5
    h = 0.8
    Startangles = np.matrix([[0.1],[0.1],[0.1],[0.1],[0.1],[0.1]])
    desiredXYZ = np.matrix([[0.832], [-0.048], [0.188]])
    #fdesired = np.matrix([[0],[1],[0],[1],[0],[0],[0],[0],[1],[0.832], [-0.048], [0.188]])
    fdesired = np.matrix([[1],[0],[0],[0],[1],[0],[0],[0],[-1],[0.832], [-0.048], [0.188]]) 
    finalPos = StraightLine(Startangles,fdesired,Tol,NumberP) #IGNORE
    # pos = allcalcs(fdesired, Startangles, Tol)
    #pos = allcalcs(desiredXYZ, Startangles, Tol)
    #print(np.shape(pos))
    armMovementPlot(finalPos)
  
    end = time.time()

    print(np.shape(finalPos))
    duration = end - start
    print(duration,"seconds")


