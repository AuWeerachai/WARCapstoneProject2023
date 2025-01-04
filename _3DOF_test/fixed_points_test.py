import hebi
import math
from time import sleep, time
import numpy as np
import sympy as sp

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
num_joints = 3
num_waypoints = 10
print(num_joints)


pos = np.empty((num_joints,num_waypoints))
vel = np.empty((num_joints,num_waypoints))
acc = np.empty((num_joints,num_waypoints))

# Set first and last waypoint values to 0.0
vel[:,0] = acc[:,0] = 0.0
vel[:,-1] = acc[:,-1] = 0.0
# Set all other values to NaN
vel[:,1:-1] = acc[:,1:-1] = np.nan

# Set positions
pos[:,0] = angles[:]
pos[:,1] = 0
pos[:,2] = [-1.15, 1.31, -1.16]
pos[:,3] = [-2.85, 1.41, -1.20]
pos[:,4] = [-4.27, 1.43, -1.49]
pos[:,5] = [-4.18, 1.16, -2.01]
pos[:,6] = [-4.27, 1.43, -1.49]
pos[:,7] = [-3.70, 1.42, -1.67]
pos[:,8] = [-3.73, 1, -1.93]
pos[:,9] = 0.0

# The times to reach each waypoint (in seconds)
time = np.linspace(0.0, 30, num_waypoints)

# Define trajectory
trajectory = hebi.trajectory.create_trajectory(time, pos, vel, acc)


# Follow the trajectory
cmd = hebi.GroupCommand(num_joints)
period = 0.01
duration = trajectory.duration

pos_cmd = np.array(num_joints, dtype=np.float64)
vel_cmd = np.array(num_joints, dtype=np.float64)

t = 0.0

while (t < duration):
  pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)

  print("=======")
  print(pos_cmd)
  print(vel_cmd)
  print(acc_cmd)
  print("=======")

  cmd.position = pos_cmd
  cmd.velocity = vel_cmd
  group.send_command(cmd)

  t = t + period
  sleep(period)