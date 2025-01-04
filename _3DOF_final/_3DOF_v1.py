#!/usr/bin/env python3

import hebi
import math
from time import sleep, time
import numpy as np
import sympy as sp

# jacks calcs begin here =========== #


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
    mat1 = AN1(theta1) #most updated test code for 3DOF arm
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
    # print(check)
    return (check[0,0]<tol and check[1,0]<tol and check[2,0]<tol)

def allcalcs(XYZ):
    thtfeedback = np.matrix([[0.0],[0.0],[0.0]])
    cmnd = thtfeedback
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
    return(anglepath)


def calculatedXYZ(finalangles):
    f = AttndPos(finalangles[0,0],finalangles[1,0],finalangles[2,0])[4]
    f = np.matrix([[f[9]],[f[10]],[f[11]]])
    return f


tol = 0.005
#XYZ = np.matrix([[0.306721177788881],[0.559823752684030],[0.217720653401772]]) #this works
#XYZ = np.matrix([[0.239964935729273],[0.512854877765793],[0.495439048202929]]) #THIS WORKS

#XYZ = np.matrix([[0.358752941629404], [0.162561288676887], [0.684273545618736]]) #Works after cutting it in half
#XYZ = np.matrix([[-0.179365427914854], [-0.350702087869525], [0.590052297787412]]) #works after cutting into thirds
XYZ = np.matrix([[-0.554904582093652], [0.0327905559698084], [0.398464347287072]])

pos = allcalcs(XYZ)
final = pos[:,(np.shape(pos)[1]-1)]
newXYZ = calculatedXYZ(final)
diff = XYZ - newXYZ


print("the difference between desired XYZ and predicted XYZ is:")
print(diff)


#  Jacks calcs ends ==================================================# 



# #====================== GET INITIAL FEEDBACK =============================
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

#======= Get feedback =======
group.send_feedback_request()
group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
angles = group_feedback.position
#print(angles)
#===========================================================





# Populate variable 'current_position' with position feedback

# Position, velocity, and acceleration waypoints.
# Each column is a separate waypoint.
# Each row is a different joint.



# Set positions, and first waypoint as current location1
pos[0,0] = angles[0]
pos[1,0] = angles[1]
pos[2,0] = angles[2]


startingLocation = AttndPos(angles[0], angles[1], angles[2])[3]
print("Starting Location (XYZ): ")
print(startingLocation)

# pos[:,1] = 0
# pos[:,2] = [0, 2.72, -1.2]
# pos[:,3] = [-1.5, 2.72, -1.2]
# pos[:,4] = [-3.3, 2.72, -1.2]
# pos[:,5] = [0, 2.72, -1.2]
# pos[:,6] = 0.0

# #take only every nth waypoint (theta values)
pos = pos[:,0::4]
print("+++++++++Take Every nth Value+++++++++")
print(pos)

num_joints = np.shape(pos)[0]
num_waypoints = np.shape(pos)[1]
print("Number of Joints:")
print(num_joints)
print("Number of Waypoints: ")
print(num_waypoints)

# pos = np.empty((num_joints,num_waypoints))
vel = np.empty((num_joints,num_waypoints))
#print(np.shape(vel))
acc = np.empty((num_joints,num_waypoints))

# Set first and last waypoint values to 0.0
vel[:,0] = acc[:,0] = 0.0
vel[:,-1] = acc[:,-1] = 0.0
# Set all other values to NaN
vel[:,1:-1] = acc[:,1:-1] = np.nan

print("===Position matrix after configuring first waypoint to current location ====")
print(pos)

# The times to reach each waypoint (in seconds)
time = np.linspace(0.0, 5*num_waypoints, num_waypoints)

# Define trajectory
trajectory = hebi.trajectory.create_trajectory(time, pos, vel, acc)
print("First Waypoint: ")
print(trajectory.get_state(0))


# Follow the trajectory
cmd = hebi.GroupCommand(num_joints)
period = 0.01 #affects execution rate
duration = trajectory.duration

pos_cmd = np.array(num_joints, dtype=np.float64)
vel_cmd = np.array(num_joints, dtype=np.float64)

t = 0.0


### Following code actually moves the arm

while (t < duration):
  pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)

  print("=======")
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












