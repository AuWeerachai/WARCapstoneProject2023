import hebi
import math
from time import sleep, time
import numpy as np
import sympy as sp
import time 
from lines import line3D
from Calculations_SinglePoint import StraightLine, addGimbal

def getTrajectory(startAngles, endXYZ, waypoints, speed): 
    pos = addGimbal(startAngles,endXYZ, tol, waypoints)   #0.01 rad tolerance, 5 cuts to straight line trajectory
    # print("===Theta Waypoints Matrix  ====")
    # print(pos)
    # newXYZ = calculatedXYZ(pos[:,-1])
    # diff = desiredXYZ - newXYZ
    # print(diff)
    num_joints = np.shape(pos)[0]  
    num_waypoints = np.shape(pos)[1]
    # if(num_waypoints > max_waypoints):  
    #     n = num_waypoints // max_waypoints
    #     pos = pos[:,0::n]
    #     # print(f"+++++++++Take Every {n}th Value+++++++++")
    #     # print(pos)
    #     num_waypoints = np.shape(pos)[1]
    # print("Number of Joints:")
    # print(num_joints)
    # print("Number of Waypoints: ")
    # print(num_waypoints)
    vel = np.empty((num_joints,num_waypoints)) #defines velocity matrix
    acc = np.empty((num_joints,num_waypoints)) #defines velocity matrix
    vel[:,0] = acc[:,0] = 0.0 #make start velocity/acceleration 0
    vel[:,-1] = acc[:,-1] = 0.0  #make end velocity/acceleration 0
    vel[:,1:-1] = acc[:,1:-1] = np.nan
    time = np.linspace(0.0, speed*num_waypoints, num_waypoints) 
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

#Code for 6DOF arm
family_name = "Arm"
mod1_name = "m1"
mod2_name = "m2"
mod3_name = "m3"
mod5_name = "m5"
mod6_name = "m6"
mod7_name = "m7" #send negative value of 6
mod8_name = "m8"

group = lookup.get_group_from_names([family_name], [mod8_name, mod6_name, mod7_name, mod5_name, mod3_name, mod2_name, mod1_name])

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

tol = 0.01 #how close do you want calculations to converge

offset_m6_times = -1
offset_m3_times = -1
offset_m2_times = -1
offset_m5_add = 1.57

Startangles = np.matrix([[angles[0]],[angles[1]*offset_m6_times],[angles[3]+offset_m5_add],[angles[4]*offset_m3_times],[angles[5]*offset_m2_times],[angles[6]]])

# Point1 = np.matrix([[0.5], [0.3], [0]]) #WORKS

Point1 = np.matrix([[0.4], [0.4], [0.3]]) 
#Point1 = np.matrix([[0], [0.5], [0.5]]) 



waypoints = 5  #number of waypoints between start point and next point (not necessary to update every single time)
speed = 0.4 #seconds per waypoint (also not necessary to update every single time)

trajCalcValues = getTrajectory(Startangles, Point1, waypoints, speed) #current pos to Point 10
trajectory = trajCalcValues[0]
num_joints = trajCalcValues[1] #since numjoints is always the same, no need to redeclare every single time
num_waypoints = trajCalcValues[2]


isFinalTrajectory = True
runTrajectory(trajectory, num_joints, num_waypoints, isFinalTrajectory)
# runTrajectory(trajectory_2, num_joints, num_waypoints_2, isFinalTrajectory)
# runTrajectory(trajectory_3, num_joints, num_waypoints_3, isFinalTrajectory)
# runTrajectory(trajectory_4, num_joints, num_waypoints_4, isFinalTrajectory)
# runTrajectory(trajectory_5, num_joints, num_waypoints_5, isFinalTrajectory)
# isFinalTrajectory = True #next trajectory is the last; will stop after running
# runTrajectory(trajectory_6, num_joints, num_waypoints_6, isFinalTrajectory)