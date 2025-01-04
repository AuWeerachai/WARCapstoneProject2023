import hebi
import math
from time import sleep, time
import numpy as np
import sympy as sp
import time 

class RobotArm:
    def __init__(self, window=None):
        ##====================== SETUP HEBI AND FEEDBACK =============================
    
        self.start = time.time()
        self.window = window

        lookup = hebi.Lookup()
        dir(hebi)
        # Wait 2 seconds for the module list to populate
        sleep(2.0)

        #Code for 6DOF arm
        # family_name = "Arm"
        mod1_name = "m1"
        mod2_name = "m2"
        mod3_name = "m3"
        mod5_name = "m5"
        mod6_name = "m6"
        mod7_name = "m7" #send negative value of 6
        mod8_name = "m8"
        family_nameG = "Gripper"
        mod4_name = "m4"
        # self.group = lookup.get_group_from_names([family_name], [mod8_name, mod6_name, mod7_name, mod5_name, mod3_name, mod2_name, mod1_name])
        
        # Distinguish which arm it is
        family_name = "Arm1"
        self.group = lookup.get_group_from_names([family_name], [mod8_name, mod6_name, mod7_name, mod5_name, mod3_name, mod2_name, mod1_name])
        self.fam_nam = family_name
        print("Arm 1 FOund")
        if self.group is None:
            family_name = "Arm2"
            self.group = lookup.get_group_from_names([family_name], [mod8_name, mod6_name, mod7_name, mod5_name, mod3_name, mod2_name, mod1_name])
            self.fam_nam = family_name
            print("Arm 2 FOund")
            # window.camera_offset_x.set(-0.01)
            # window.camera_offset_y.set(0.2)
        
        self.Gripper = lookup.get_group_from_names([family_nameG], [mod4_name])
        if self.group is None:
            print('Group not found: Did you forget to set the module family and name above?')
            raise RuntimeError

        if self.Gripper is None:
            print('Gripper not found: Did you forget to set the module family and name above?')
            raise RuntimeError

        print('Found group on network with {0} modules.'.format(self.group.size))
        self.group.feedback_frequency = 50.0
        self.Gripper.feedback_frequency = 50.0

        group_feedback = hebi.GroupFeedback(self.group.size)
        self.Gripper_feedback = hebi.GroupFeedback(self.Gripper.size)

        self.dangerousTorque = False

        if(family_name == "Arm2"):
            self.gripper_release = -0.25
            self.gripper_hold = 0.25
        else:
            self.gripper_release = -1.40
            self.gripper_hold =   -0.76

        #======= Get Initial Theta Position Feedback =======
        self.group.send_feedback_request()
        group_feedback = self.group.get_next_feedback(reuse_fbk=group_feedback)

        self.angles = group_feedback.position
        #print(angles)
        #===========================================================


        #============= USING FEEDBACK AND FUNCTIONS TO DO CALCS =========

        self.tol = 0.01 #how close do you want calculations to converge

        self.offset_m6_times = -1
        self.offset_m3_times = -1
        self.offset_m2_times = -1
        self.offset_m5_add = 1.57

    def execute_path(self, path, desired_waypoints=5, speed=0.4, slow_speed=1):
        # provide a list of points as vertical matrices
        Startangles = np.matrix([[self.angles[0]],[self.angles[1]*self.offset_m6_times],[self.angles[3]+self.offset_m5_add],[self.angles[4]*self.offset_m3_times],[self.angles[5]*self.offset_m2_times],[self.angles[6]]])
        trajCalcValues = self.getHomeTrajectory(Startangles,slow_speed)
        trajectory = [trajCalcValues[0]]
        num_joints = trajCalcValues[1]
        for point in path:
            start_angles = trajCalcValues[3]
            trajCalcValues = self.getTrajectory(start_angles, point, desired_waypoints, slow_speed)
            trajectory.append(trajCalcValues[0])
            print(time.time()-self.start)
        final_traj = self.getHomeTrajectory(start_angles, slow_speed)
        for traj in trajectory:
            self.runTrajectory(traj, num_joints, desired_waypoints, False)
        self.runTrajectory(final_traj, num_joints, desired_waypoints, True)
        

    def demo_path(self):
        if self.window is not None:
            round_x = self.window.coordinates[0][0]
            round_y = self.window.coordinates[0][1]
            dest_x = self.window.destination[0].get()
            dest_y = self.window.destination[1].get()
        else:
            round_x = 0.4
            round_y = 0.4
            dest_x = 0.09
            dest_y = -0.54

        #Remember to offset the feedback -> calc
        Startangles = np.matrix([[self.angles[0]],[self.angles[1]*self.offset_m6_times],[self.angles[3]+self.offset_m5_add],[self.angles[4]*self.offset_m3_times],[self.angles[5]*self.offset_m2_times],[self.angles[6]]])

        round_z = 0.4

        Point1 = np.matrix([[0.565], [-0.184], [0.331]]) 
        Point2 = np.matrix([[round_x], [round_y], [round_z]]) 
        Point3 = np.matrix([[round_x], [round_y], [0.09]]) 
        Point4 = np.matrix([[round_x], [round_y], [round_z]]) 
        Point5 = np.matrix([[0.565], [-0.184], [0.331]]) 
        Point6 = np.matrix([[dest_x], [dest_y], [round_z]]) 
        Point7 = np.matrix([[dest_x], [dest_y], [0.095]])
        Point71 = np.matrix([[dest_x], [dest_y], [0.087]])
        Point8 = np.matrix([[dest_x], [dest_y], [round_z]]) 
        Point9 = np.matrix([[0.565], [-0.184], [0.331]])  




        desired_waypoints = 5  #numbewait(2,values[0],values[1],Release = False)r of waypoints between start point and next point (not necessary to update every single time)
        speed = self.window.arm_speed.get() #seconds per waypoint (also not necessary to update every single time)
        slowSpeed = 1


        # INITIAL -> HOME (NO ROUND)
        trajCalcValues = self.getHomeTrajectory(Startangles,slowSpeed)
        trajectory1 = trajCalcValues[0]
        num_joints = trajCalcValues[1] #since numjoints is always the same, no need to redeclare every single time
        num_waypoints = desired_waypoints
        print(time.time()-self.start)
        # trajCalcValues = getTrajectory(Startangles, Point1, desired_waypoints, speed) 
        # trajectory1 = trajCalcValues[0]
        # num_joints = trajCalcValues[1] #since numjoints is always the same, no need to redeclare every single time
        # num_waypoints = trajCalcValues[2]
        # print(time.time()-start)

        # HOME -> Point 2 (Reach first round)
        Startangles = trajCalcValues[3]
        trajCalcValues = self.getTrajectory(Startangles, Point2, desired_waypoints, speed) 
        trajectory2 = trajCalcValues[0]
        print(time.time()-self.start)


        # Point 2 -> Point 3 (reach down)
        Startangles = trajCalcValues[3]
        trajCalcValues = self.getTrajectory(Startangles, Point3, desired_waypoints, speed) 
        trajectory3 = trajCalcValues[0]
        print(time.time()-self.start)

        # Point 3 -> Point 4 (pick up)
        Startangles = trajCalcValues[3]
        trajCalcValues = self.getTrajectory(Startangles, Point4, desired_waypoints, speed) 
        trajectory4 = trajCalcValues[0] 
        print(time.time()-self.start)

        # Point 4 -> HOME (WITH ROUND)
        Startangles = trajCalcValues[3]
        # trajCalcValues = getHomeTrajectory(Startangles,speed)
        trajCalcValues = self.getTrajectory(Startangles, Point5, desired_waypoints, speed) 
        trajectory5 = trajCalcValues[0]
        print(time.time()-self.start)

        # HOME -> Point 6 (Reach second round)
        Startangles = trajCalcValues[3]
        trajCalcValues = self.getTrajectory(Startangles, Point6, desired_waypoints, speed) 
        trajectory6 = trajCalcValues[0]
        print(time.time()-self.start)

        #Point 6 -> Point 7 (reach down) 
        Startangles = trajCalcValues[3]
        trajCalcValues = self.getTrajectory(Startangles, Point7, desired_waypoints, speed) 
        trajectory7 = trajCalcValues[0]
        print(time.time()-self.start)

        #bump Down
        Startangles = trajCalcValues[3]
        trajCalcValues = self.getTrajectory(Startangles, Point71, desired_waypoints, speed) 
        trajectory71 = trajCalcValues[0]
        print(time.time()-self.start)

        #Point 7 -> Point 8 (come up) 
        Startangles = trajCalcValues[3]
        trajCalcValues = self.getTrajectory(Startangles, Point8, desired_waypoints, speed) 
        trajectory8 = trajCalcValues[0]
        print(time.time()-self.start)

        #Point 8 -> HOME (NO ROUND)
        Startangles = trajCalcValues[3]
        trajCalcValues = self.getTrajectory(Startangles, Point9, desired_waypoints, speed) 
        #trajCalcValues = getHomeTrajectory(Startangles, slowSpeed) 
        trajectory9 = trajCalcValues[0]
        print(time.time()-self.start)



        # blocking = input("Enter Something")



        isFinalTrajectory = False
        unwinding_flag = True
        self.runTrajectory(trajectory1, num_joints, num_waypoints, isFinalTrajectory, self.group, unwinding_flag, self.dangerousTorque)
        unwinding_flag = False
        values = self.runTrajectory(trajectory2, num_joints, num_waypoints, isFinalTrajectory, self.group, unwinding_flag, self.dangerousTorque)
        self.wait(1,values[0],values[1],Release = True)
        values = self.runTrajectory(trajectory3, num_joints, num_waypoints, isFinalTrajectory, self.group, unwinding_flag, self.dangerousTorque)
        self.wait(1,values[0],values[1],Release = False)
        self.runTrajectory(trajectory4, num_joints, num_waypoints, isFinalTrajectory, self.group, unwinding_flag, self.dangerousTorque)
        self.runTrajectory(trajectory5, num_joints, num_waypoints, isFinalTrajectory, self.group, unwinding_flag, self.dangerousTorque)
        self.runTrajectory(trajectory6, num_joints, num_waypoints, isFinalTrajectory, self.group, unwinding_flag, self.dangerousTorque)
        values = self.runTrajectory(trajectory7, num_joints, num_waypoints, isFinalTrajectory, self.group, unwinding_flag, self.dangerousTorque)
        self.wait(1,values[0],values[1],Release = True)
        values = self.runTrajectory(trajectory71, num_joints, num_waypoints, isFinalTrajectory, self.group, unwinding_flag, self.dangerousTorque)
        values = self.runTrajectory(trajectory8, num_joints, num_waypoints, isFinalTrajectory, self.group, unwinding_flag, self.dangerousTorque)
        isFinalTrajectory = False
        self.runTrajectory(trajectory9, num_joints, num_waypoints, isFinalTrajectory, self.group, unwinding_flag, self.dangerousTorque)

        
    def getHomeTrajectory(self, startAngles, speed):  
        startAngs = np.copy(startAngles)
        secondShoulder = -1*startAngs[1,:]
        startAngs = np.insert(startAngs,2,secondShoulder,axis = 0)
        # startAngs = np.transpose(startAngs)
        homeAngs = np.transpose(np.matrix([0,1.047,-1.047,-1.57,-1.047,-1.57,0]))
        # print(startAngs)
        # print(homeAngs)
        waypoints = np.append(startAngs,homeAngs,axis=1)
        # print(waypoints)
        num_joints = np.shape(waypoints)[0]  
        num_waypoints = np.shape(waypoints)[1] 

        vel = np.empty((num_joints,num_waypoints)) #defines velocity matrix
        acc = np.empty((num_joints,num_waypoints)) #defines velocity matrix
        vel[:,0] = acc[:,0] = 0.0 #make start velocity/acceleration 0
        vel[:,-1] = acc[:,-1] = 0.0  #make end velocity/acceleration 0
        vel[:,1:-1] = acc[:,1:-1] = np.nan
        time = np.linspace(0.0, speed*num_waypoints, num_waypoints) 
        trajectory = hebi.trajectory.create_trajectory(time, waypoints, vel, acc)

    # home angles but  usable calculation form
        lastAngles =  np.transpose(np.matrix([0,-1.047,0,1.047,1.57,0]))

        #return trajectory object, number of waypoints/joints, and last set of angles in the (Originally calculated) big waypoints matrix
        # print("Return Last Angles: ")
        # print(lastAngles)
        return trajectory,num_joints,num_waypoints,lastAngles




    def getTrajectory(self,startAngles, endXYZ, waypoints, speed):  
        
        calcPos = StraightLine(startAngles,endXYZ, self.tol, waypoints)
   
        # print("1")
        # print(calcPos)
        lastAngles = calcPos[:,-1]

    #convert calculations to usable feedback
        feedbackPos = np.copy(calcPos) #MAKE SURE TO USE COPY AND NOT THE EQUAL SIGN OR IT WILL MUTATE THE ORIGINAL ARRAY
        feedbackPos[1,:] = feedbackPos[1,:]*(-1)  #row 2
        feedbackPos[2,:] = feedbackPos[2,:]-(0.5*np.pi) #row 3
        feedbackPos[3,:] = feedbackPos[3,:]*(-1)
        feedbackPos[4,:] = feedbackPos[4,:]*(-1)

        #Make M2 point straight down depending on if it is Arm 1 or Arm 2
        #print(self.window.m2_offset.get())
        if self.fam_nam == "Arm2":
            # feedbackPos[4,:] = feedbackPos[4,:] + self.window.m2_offset.get()
            feedbackPos[4,:] = feedbackPos[4,:] + 0.33
        else:
            feedbackPos[4,:] = feedbackPos[4,:] + 0.15

        newRow = -1*feedbackPos[1,:]
        feedbackPos = np.insert(feedbackPos,2,newRow,axis = 0)



        # if(self.group.family_name == "Arm1"):
        #     print("THIS IS ARM1")
        # elif(self.group.family_name == "Arm2"):
        #     print("THIS IS ARM2")

        num_joints = np.shape(feedbackPos)[0]  
        num_waypoints = np.shape(feedbackPos)[1]
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
        trajectory = hebi.trajectory.create_trajectory(time, feedbackPos, vel, acc)
        # print("First Set of Gimbal Angles: ")
        # print(trajectory.get_state(0))
        
        #return trajectory object, number of waypoints/joints, and last set of angles in the (Originally calculated) big waypoints matrix
        # print("Return Last Angles: ")
        # print(lastAngles)
        return trajectory,num_joints,num_waypoints, lastAngles

    def runTrajectory(self,trajectory,num_joints,num_waypoints, isFinal, group, unwinding_flag, dangerousTorque):
        cmd = hebi.GroupCommand(num_joints)
        period = 0.01 #affects execution rate
        duration = trajectory.duration
        
        pos_cmd = np.array(num_joints, dtype=np.float64)
        vel_cmd = np.array(num_joints, dtype=np.float64)

        posfinal_cmd = 0 #matrices to hold final position
        velfinal_cmd = 0
        accfinal_cmd = 0
        t = 0

        group_feedback = hebi.GroupFeedback(num_joints)
        rolling_window = static_calc_params()

        while (t < duration):
            pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)
            posfinal_cmd, velfinal_cmd, accfinal_cmd = trajectory.get_state(t)
            cmd.position = pos_cmd
            cmd.velocity = vel_cmd
            self.group.send_command(cmd)
            t = t + period
            sleep(period)

            group.get_next_feedback(reuse_fbk=group_feedback)
            theta = group_feedback.effort

            dangerousTorque, rolling_window = static_torque(theta, rolling_window, t, unwinding_flag, dangerousTorque)
            if dangerousTorque:
                while True:
                    cmd.position = posfinal_cmd
                    cmd.velocity = velfinal_cmd
                    self.group.send_command(cmd)
                break
        
        loop = isFinal
        while(loop):
            cmd.position = posfinal_cmd
            cmd.velocity = velfinal_cmd
            self.group.send_command(cmd)

        return pos_cmd,vel_cmd

    def manipulateGripper(self,rad):
        pos_cmd = np.ones(1, dtype=np.float64)
        pos_cmd = rad*pos_cmd
        cmd = hebi.GroupCommand(1)
        cmd.position = pos_cmd
        self.Gripper.send_command(cmd)
        return

    def wait(self,duration,pos,vel, Release = True):
        loop = True
        begin = time.time()
        pos_cmd = np.array(7, dtype=np.float64)
        vel_cmd = np.array(7, dtype=np.float64)
        pos_cmd, vel_cmd = pos,vel
        cmd = hebi.GroupCommand(7)
        while(loop):
            cmd.position = pos_cmd
            cmd.velocity = vel_cmd
            self.group.send_command(cmd)
            if(time.time()-begin > (duration-2)):
                if Release:
                    self.manipulateGripper(self.gripper_release)
                else:
                    self.manipulateGripper(self.gripper_hold)
            if (time.time()-begin > duration):
                loop = False
        return True


if __name__ == '__main__':
    from lines import line3D
    from Calculations_Final import StraightLine #, CalcToFeedback
    newArm = RobotArm()
else:
    from .lines import line3D
    from .Calculations_Final import StraightLine #, CalcToFeedback
    from .static_calc import *
