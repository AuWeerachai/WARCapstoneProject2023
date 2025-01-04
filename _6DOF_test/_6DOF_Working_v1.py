#!/usr/bin/env python3

import hebi
from math import pi, sin
from time import sleep, time

lookup = hebi.Lookup()
dir(hebi)
# Wait 2 seconds for the module list to populate
sleep(2.0)



family_name = "Arm"
mod1_name = "m1"
mod2_name = "m2"
mod3_name = "m3"
#mod4_name = "mod4"
mod5_name = "m5"
mod6_name = "m6"
#mod7_name = "m7"
mod8_name = "m8"


group = lookup.get_group_from_names([family_name], [mod1_name,mod2_name,mod3_name,mod5_name,mod6_name, mod8_name])


if group is None:
    print('Group not found: Did you forget to set the module family and name above?')
    exit(1)

print('Found group on network with {0} modules.'.format(group.size))
group.feedback_frequency = 50.0

group_feedback = hebi.GroupFeedback(group.size)

# # ==================  Load the kinematics from an HRDF file, (Load the RobotModel for a 6-DoF Arm) ==========================
try:
    arm = hebi.robot_model.import_from_hrdf("origArm.hrdf")
except:
    print("Could not load HRDF.")
    exit(1)

print('Robot model loaded from HRDF has {0} degrees of freedom.'.format(arm.dof_count))



# # ================== CREATING ROBOT MODEL MANUALLY WITHOUT USING HRDF FILES ==========================
# model_from_code = hebi.robot_model.RobotModel()
# print(model_from_code.add_actuator('X5-9'))


# print(model_from_code.add_bracket('X5-HeavyBracket', 'left-outside'))

# model_from_code.add_actuator('X5-9')

# model_from_code.add_link('X5', 0.325, pi)
# model_from_code.add_actuator('X5-4')
# model_from_code.add_link('X5', 0.325, 0.0)

# # Method 2: Create a simple kinematic description of the arm in code
# print('Robot model generated in code has {0} degrees of freedom.'.format(model_from_code.dof_count))




# # ================== FORWARD KINEMATICS TEST ==========================
def feedback_handler(group_fbk):
    angles = group_fbk.position
    transform = arm.get_end_effector(angles)
    print('x,y,z: {0}, {1}, {2}'.format(100*transform[0, 3], 100*transform[1, 3], 100*transform[2, 3]))


group.add_feedback_handler(feedback_handler)
group.feedback_frequency = 10.0  # Prevent printing to the screen too much

## Control the robot at 10 Hz for 30 seconds1
sleep(500)






#  # ================== INVERSE KINEMATICS TEST ==========================
# target_xyz = [9.30/100.0, 32.1/100.0, -5/100.0]

# ################################################################
# # Get position feedback from robot to use as initial conditions for local optimization.
# ################################################################

# # group_fbk = hebi.GroupFeedback(group.size)
# # if group.get_next_feedback(reuse_fbk=group_fbk) is None:
# #     print("Couldn't get feedback.")
# #     exit(1)

# # Note: user should check if the positions are appropriate for initial conditions.
# initial_joint_angles = group_feedback.position

# ################################################################
# # Get IK Solution with one objective
# ################################################################

# # Just one objective:
# # Note: this is a numerical optimization and can be significantly affected by initial conditions (seed joint angles)
# ee_pos_objective = hebi.robot_model.endeffector_position_objective(target_xyz)
# ik_result_joint_angles = arm.solve_inverse_kinematics(initial_joint_angles, ee_pos_objective)

# print('Target position: {0}'.format(target_xyz))
# print('IK joint angles: {0}'.format(ik_result_joint_angles))
# print('FK of IK joint angles: {0}'.format(arm.get_end_effector(ik_result_joint_angles)[0:3, 3]))

# ################################################################
# # Send commands to the physical robot
# ################################################################

# # Move the arm
# # Note: you could use the Hebi Trajectory API to do this smoothly
# group_command = hebi.GroupCommand(group.size)
# group_command.position = ik_result_joint_angles

# for i in range(100):
#     group.send_command(group_command)
#     # Note: the arm will go limp after the 100 ms command lifetime,
#     # so the command is repeated every 50 ms for 5 seconds.
#     sleep(0.05)

