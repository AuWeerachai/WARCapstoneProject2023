import numpy as np
import sympy as sp
from Jacobian import FirstJ
from lines import line3D, curve3D

def calcX(thetaActual,desiredXYZ,J,F):

    # tht1,tht2,tht3,tht4,tht5,tht6 = sp.symbols('tht1 tht2 tht3 tht4 tht5 tht6', real = True)
    # angs = sp.Matrix([[tht1],[tht2],[tht3],[tht4],[tht5],[tht6]])
    #Determine forward Kinematics Parameters
    avec = F[0]
    ovec = F[1]
    nvec = F[2]
    pvec = F[3]
    fmoment = F[4]
    ang1 = thetaActual[0,0]
    ang2 = thetaActual[1,0]
    ang3 = thetaActual[2,0]
    ang4 = thetaActual[3,0]
    ang5 = thetaActual[4,0]
    ang6 = thetaActual[5,0]
    J = J.subs(tht1,ang1)
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

    fnew = np.zeros([12,1])
    # fnew[0]= 0
    # fnew[1]= 0
    # fnew[2]= -1
    # fnew[3]= 0
    # fnew[4]= 1
    # fnew[5]= 0
    # fnew[6]= 1
    # fnew[7]= 0
    # fnew[8]= 0
    fnew[0]= fold[0]
    fnew[1]= fold[1]
    fnew[2]= fold[2]
    fnew[3]= fold[3]
    fnew[4]= fold[4]
    fnew[5]= fold[5]
    fnew[6]= fold[6]
    fnew[7]= fold[7]
    fnew[8]= fold[8]
    fnew[9]= desiredXYZ[0]
    fnew[10]= desiredXYZ[1]
    fnew[11]= desiredXYZ[2]
    #put into numpy for easier calcs
    J = tonumpy(J)
    fold = tonumpy(fold)
    fnew = tonumpy(fnew)
    # do the actual calcs of X
    invJ = np.linalg.pinv(J)
    X = np.matmul(invJ,(fnew - fold))
    return X

def checkforexit(tht,cmnd,tol):
    check = abs(tht - cmnd)
    return (check[0,0]<tol and check[1,0]<tol and check[2,0]<tol and check[3,0]<tol and check[4,0]<tol and check[5,0]<tol)

def allcalcs(XYZ,start,tol,J):
    cmnd = start
    loop = True
    anglepath = np.transpose(start)
    while loop:
        X = calcX(cmnd,XYZ,J)
        thtnew = cmnd + X
        logger = np.transpose(thtnew)
        anglepath = np.append(anglepath,logger,axis = 0)
        if checkforexit(thtnew,cmnd,tol):
            loop = False
        
        else:
            cmnd = thtnew
    anglepath = np.delete(anglepath, (0), axis=0)
    anglepath = np.transpose(anglepath)
    return np.transpose(logger)

def StraightLine(startA,end,tol,numP):
    start = np.transpose(calculatedXYZ(startA))
    #  print(start)
    linPoints = np.transpose(line3D(start,np.transpose(end),numP, False))
    # print(linPoints[:,1])
    size = np.shape(linPoints)[1]
    allAngs = startA
    J = FirstJ()[0]
    F = FirstJ()[1]
    for i in range(size-1):
        moves = allcalcs(linPoints[:,i+1], startA,tol,J,F)
        allAngs = np.append(allAngs,moves,axis = 1)
        startA = moves[:,-1]
        print("Next Subpoint")
    return allAngs