#!/usr/bin/env python3

import hebi
import math
from time import sleep, time
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt

## =========== Kinematics Functions =========== #

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
    mat = sp.Matrix([[1, 0, 0, 0.37],
                        [0,1,0,-0.08],
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

    a = AIE.col(0)
    o = AIE.col(1)
    n = AIE.col(2)
    p = AIE.col(3)
    
    f = sp.Matrix([[a],[o],[n],[p]])

    AIE.col_del(3)
    CIE = AIE


    return (a,o,n,p,f,CIE) 

def bigJ(angv,a,o,n,p):
    J1 = jacobi(a,angv)
    J2 = jacobi(o,angv)
    J3 = jacobi(n,angv)
    J4 = jacobi(p,angv)
    J = sp.Matrix([[J1],[J2],[J3],[J4]])
    return J

def tonumpy(m):
    numrows = sp.shape(m)[0]
    numcols =sp.shape(m)[1]
    nummy = np.zeros([numrows, numcols])
    for i in range(0,numcols):
        for j in range(0,numrows):
            nummy[j,i] = m[j,i]

    return nummy

#define our actuator angles as a vector of symbols
def calcX(thetaActual,desiredXYZ):

    tht1,tht2,tht3 = sp.symbols('tht1 tht2 tht3', real = True)
    angs = sp.Matrix([[tht1],[tht2],[tht3]])
    #Determine forward Kinematics Parameters
    Forwards = AttndPos(tht1,tht2,tht3)
    avec = Forwards[0]
    ovec = Forwards[1]
    nvec = Forwards[2]
    pvec = Forwards[3]
    fmoment = Forwards[4]
    CIEmoment = Forwards[5]
    #Find the Jacobian of the matrix symbolically
    Jsym = bigJ(angs,avec,ovec,nvec,pvec)
    #Start Subbing in the real values from feedback
    ang1 = thetaActual[0,0]
    ang2 = thetaActual[1,0]
    ang3 = thetaActual[2,0]
    angs = np.matrix([[ang1],[ang2],[ang3]])
    J = Jsym.subs(tht1,ang1)
    J = J.subs(tht2, ang2)
    J = J.subs(tht3,ang3)
    fold = fmoment.subs(tht1,ang1)
    fold = fold.subs(tht2,ang2)
    fold = fold.subs(tht3,ang3)
    fnew = np.zeros([12,1])
    fnew[0]=fold[0]
    fnew[1]=fold[1]
    fnew[2]=fold[2]
    fnew[3]=fold[3]
    fnew[4]=fold[4]
    fnew[5]=fold[5]
    fnew[6]=fold[6]
    fnew[7]=fold[7]
    fnew[8]=fold[8]
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


def checkforexit(tht,cmnd):
    check = abs(tht - cmnd)
    return (check[0,0]<tol and check[1,0]<tol and check[2,0]<tol)

def allcalcs(XYZ, initialAngles):
    cmnd = initialAngles
    loop = True

    anglepath = np.matrix([0,0,0])
    while loop:
        X = calcX(cmnd,XYZ)
        thtnew = cmnd + X
        logger = np.transpose(thtnew)
        anglepath = np.append(anglepath,logger,axis = 0)
        if checkforexit(thtnew,cmnd):
            loop = False
        
        else:
            cmnd = thtnew

    anglepath = np.delete(anglepath, (0), axis=0)
    anglepath = np.transpose(anglepath)
    print("Done Calculating Path")
    return anglepath 


def calculatedXYZ(finalangles):
    f = AttndPos(finalangles[0,0],finalangles[1,0],finalangles[2,0])[4]
    f = np.matrix([[f[9]],[f[10]],[f[11]]])
    return f

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

    xf = ToPlot[0,-1]
    yf = ToPlot[1,-1]
    zf = ToPlot[2,-1]

    fig = plt.figure(figsize = (10,10))
    ax = plt.axes(projection='3d')
    ax.grid()

    ax.scatter(ToPlot[0,0], ToPlot[1,0], ToPlot[2,0], c = 'r', s = 35)
    ax.scatter(0, 0, 0, c = 'black', s = 35)
    ax.scatter(x, y, z, c = 'y', s = 25, alpha = 0.8)
    ax.scatter(xf, yf, zf, c = 'g', s = 35)
    ax.scatter(desiredXYZ[0,0], desiredXYZ[1,0], desiredXYZ[2,0], c = 'b', s = 35)
    ax.set_title('Path of Arm')
# Set axes label
    ax.set_xlabel('x', labelpad=20)
    ax.set_ylabel('y', labelpad=20)
    ax.set_zlabel('z', labelpad=20)
    plt.show()
    return

#  ===================== Kinematics Functions end ==================================================# 

def getTrajectory(startAngles, endXYZ, max_waypoints):
    pos = allcalcs(endXYZ,startAngles)
    print("===Theta Waypoints Matrix  ====")
    print(pos)
    # newXYZ = calculatedXYZ(pos[:,-1])
    # diff = desiredXYZ - newXYZ
    # print(diff)
    num_joints = np.shape(pos)[0]  
    num_waypoints = np.shape(pos)[1]
    if(num_waypoints > max_waypoints):
        n = num_waypoints // max_waypoints
        pos = pos[:,0::n]
        print(f"+++++++++Take Every {n}th Value+++++++++")
        print(pos)
        num_waypoints = np.shape(pos)[1]
    print("Number of Joints:")
    print(num_joints)
    print("Number of Waypoints: ")
    print(num_waypoints)
    vel = np.empty((num_joints,num_waypoints)) #defines velocity matrix
    acc = np.empty((num_joints,num_waypoints)) #defines velocity matrix
    vel[:,0] = acc[:,0] = 0.0 #make start velocity/acceleration 0
    vel[:,-1] = acc[:,-1] = 0.0  #make end velocity/acceleration 0
    vel[:,1:-1] = acc[:,1:-1] = np.nan
    time = np.linspace(0.0, 1*num_waypoints, num_waypoints) 
    trajectory = hebi.trajectory.create_trajectory(time, pos, vel, acc)
    # print("First Set of Gimbal Angles: ")
    # print(trajectory.get_state(0))
    lastAngles = pos[:,-1]

    #return trajectory object, number of waypoints/joints, and last set of angles in the big waypoints matrix
    return trajectory,num_joints,num_waypoints, lastAngles 

def runTrajectory(trajectory,num_joints,num_waypoints,isFinal):
    cmd = hebi.GroupCommand(num_joints)
    period = 0.01 #affects execution rate
    duration = trajectory.duration
    
    pos_cmd = np.array(num_joints, dtype=np.float64)
    vel_cmd = np.array(num_joints, dtype=np.float64)

    posfinal_cmd = 0 #matrices to hold final position
    velfinal_cmd = 0
    accfinal_cmd = 0
    t = 0

    while (t < duration):
        pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)
        posfinal_cmd, velfinal_cmd, accfinal_cmd = trajectory.get_state(t)
        print("=======")
        print(num_waypoints)
        print(t)
        print(pos_cmd)
        print(vel_cmd)
        print(acc_cmd)
        print("=======")
        cmd.position = pos_cmd
        cmd.velocity = vel_cmd
        group.send_command(cmd)
        t = t + period
        sleep(period) #helps it not execute too fast
    
    loop = isFinal
    while(loop):
        print("=======")
        print("HOLDING FINAL POSITION")
        print(posfinal_cmd)
        print(velfinal_cmd)
        print(accfinal_cmd)
        print("=======")
        cmd.position = posfinal_cmd
        cmd.velocity = velfinal_cmd
        group.send_command(cmd)

    return True

# #====================== SETUP HEBI AND FEEDBACK =============================
lookup = hebi.Lookup()
dir(hebi)
# Wait 2 seconds for the module list to populate
sleep(2.0)

#code for smaller 3DOF arm
family_name = "Test"
mod1_name = "T1"
mod2_name = "T2"
mod3_name = "T3"

group = lookup.get_group_from_names([family_name], [mod1_name,mod2_name, mod3_name])

if group is None:
    print('Group not found: Did you forget to set the module family and name above?')
    exit(1)

print('Found group on network with {0} modules.'.format(group.size))
group.feedback_frequency = 50.0

group_feedback = hebi.GroupFeedback(group.size)

#======= Get Initial Theta Position Feedback =======
group.send_feedback_request()
group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
angles = group_feedback.position
#print(angles)
#===========================================================


#============= USING FEEDBACK AND FUNCTIONS TO DO CALCS =========

tol = 0.001 #how close do you want calculations to converge
Startangles = np.matrix([[angles[0]],[angles[1]],[angles[2]]])

Point1 = np.matrix([[0.05], [0.25], [0.1]])
Point2 = np.matrix([[0.05], [0.35], [0.1]])
Point3 = np.matrix([[0.15], [0.25], [0.1]])
Point4 = np.matrix([[0.15], [0.35], [0.1]])


max_waypoints = 30
trajCalcValues = getTrajectory(Startangles,Point1, max_waypoints) #go from current pos to Point 1
trajectory = trajCalcValues[0]
num_joints = trajCalcValues[1]
num_waypoints = trajCalcValues[2]

Startangles_2 = trajCalcValues[3]
max_waypoints = 30
trajCalcValues_2 = getTrajectory(Startangles_2, Point4, max_waypoints) #go from current pos to Point 4
trajectory_2 = trajCalcValues_2[0]
num_joints_2 = trajCalcValues_2[1]
num_waypoints_2 = trajCalcValues_2[2]
Startangles_3 = trajCalcValues_2[3]

isFinalTrajectory = False
runTrajectory(trajectory, num_joints, num_waypoints, isFinalTrajectory)

isFinalTrajectory = True #will end after second point
runTrajectory(trajectory_2, num_joints_2, num_waypoints_2, isFinalTrajectory)















