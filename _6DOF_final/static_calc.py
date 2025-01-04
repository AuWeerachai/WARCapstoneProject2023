import numpy as np
from collections import deque

def compute_rotation_x(theta):
    cos_angle = round(np.cos(theta), 3)
    sin_angle = round(np.sin(theta), 3)
    return np.array([[1, 0, 0], [0, cos_angle, -sin_angle], [0, sin_angle, cos_angle]])

def compute_rotation_y(theta):
    cos_angle = round(np.cos(theta), 3)
    sin_angle = round(np.sin(theta), 3)
    return np.array([[cos_angle, 0, sin_angle], [0, 1, 0], [-sin_angle, 0, cos_angle]])

def compute_rotation_z(theta):
    cos_angle = round(np.cos(theta), 3)
    sin_angle = round(np.sin(theta), 3)
    return np.array([[cos_angle, -sin_angle, 0], [sin_angle, cos_angle, 0], [0, 0, 1]])

def static_torque_hard_calc(theta):
    calculated_torque = np.zeros((7,1))
    #Length
    L_mtr = np.array([0.10795, 0, 0.1397, 0.3302, 0.3302, 0.0762, 0.0635, 0.08255, 0.0381, 0.10795]).reshape((10,1))            #[m]

    #Mass
    mass_individual = np.array([0, 1.1203, 1.03546291918943, 0.660462919189427, 0.50976375, 1.69764687637753]).reshape((6,1)) #[kg]
    mass_B1toB6_kg = np.sum(mass_individual[1:6])
    mass_B2toB6_kg = np.sum(mass_individual[1:6])
    mass_B3toB6_kg = np.sum(mass_individual[2:6])
    mass_B4toB6_kg = np.sum(mass_individual[3:6])
    mass_B5toB6_kg = np.sum(mass_individual[4:6])
    mass_B6toB6_kg = np.sum(mass_individual[5:6])
    grav_acc_mps2 = 9.81

    #Convert to same coordiante frame from body fixed reference systems
    C_B1toI  = compute_rotation_z(theta[0])
    C_B2toB1 = compute_rotation_y(-theta[1])
    C_B3toB2 = compute_rotation_x(theta[2])
    C_B4toB3 = compute_rotation_y(-theta[3])
    C_B5toB4 = compute_rotation_y(-theta[4])
    C_B6toB5 = compute_rotation_x(theta[5])

    C_B2toI = np.dot(C_B1toI,C_B2toB1)
    C_B3toI = np.dot(C_B2toI,C_B3toB2)
    C_B4toI = np.dot(C_B3toI,C_B4toB3)
    C_B5toI = np.dot(C_B4toI,C_B5toB4)
    C_B6toI = np.dot(C_B5toI,C_B6toB5)

    pos_G1relIinI   = np.array([0, 0, 0.10795]).reshape((3,1))
    pos_G2relG1inB1 = np.array([0, 0, 0]).reshape((3,1))
    pos_G3relG2inB2 = np.array([0.1397, 0, 0]).reshape((3,1))
    pos_G4relG3inB3 = np.array([0.3302, -0.0635, 0]).reshape((3,1))
    pos_G5relG4inB4 = np.array([0.3302, -0.08255, 0]).reshape((3,1))
    pos_G6relG5inB5 = np.array([0.0762, -0.0381, 0]).reshape((3,1))
    pos_EFrelG6inB6 = np.array([0.10795, 0, 0]).reshape((3,1))

    pos_CMB2relG2inB2 = np.array([.1337, -.424, 0]).reshape((3,1))
    pos_CMB3relG3inB3 = np.array([.2104, 0.000223330063988215, 0]).reshape((3,1))
    pos_CMB4relG4inB4 = np.array([.2708, -.0158, 0]).reshape((3,1))
    pos_CMB5relG5inB5 = np.array([.0876, -.0689, 0]).reshape((3,1))
    pos_CMB6relG6inB6 = np.array([.0569, 0, 0]).reshape((3,1))

    #Body 2 cm in I
    pos_G2relG1inI = np.dot(C_B1toI,pos_G2relG1inB1)
    pos_G2relIinI = pos_G1relIinI + pos_G2relG1inI
    pos_CMB2relG2inI = np.dot(C_B2toI,pos_CMB2relG2inB2)   
    pos_CMB2relIinI = pos_G2relIinI + pos_CMB2relG2inI     

    #Body 3 cm in I
    pos_G3relG2inI = np.dot(C_B2toI,pos_G3relG2inB2)
    pos_G3relIinI = pos_G2relIinI + pos_G3relG2inI
    pos_CMB3relG3inI = np.dot(C_B3toI,pos_CMB3relG3inB3)
    pos_CMB3relIinI = pos_G3relIinI + pos_CMB3relG3inI

    #Body 4 cm in I
    pos_G4relG3inI   = np.dot(C_B3toI,pos_G4relG3inB3)
    pos_G4relIinI    = pos_G3relIinI + pos_G4relG3inI
    pos_CMB4relG4inI = np.dot(C_B4toI,pos_CMB4relG4inB4)
    pos_CMB4relIinI  = pos_G4relIinI + pos_CMB4relG4inI

    #Body 5 cm in I
    pos_G5relG4inI   = np.dot(C_B4toI,pos_G5relG4inB4)
    pos_G5relIinI    = pos_G4relIinI + pos_G5relG4inI
    pos_CMB5relG5inI = np.dot(C_B5toI,pos_CMB5relG5inB5)
    pos_CMB5relIinI  = pos_G5relIinI + pos_CMB5relG5inI

    #Body 6 cm in I
    pos_G6relG5inI   = np.dot(C_B5toI,pos_G6relG5inB5)
    pos_G6relIinI    = pos_G5relIinI + pos_G6relG5inI
    pos_CMB6relG6inI = np.dot(C_B6toI,pos_CMB6relG6inB6)
    pos_CMB6relIinI  = pos_G6relIinI + pos_CMB6relG6inI

    #Bodies 1 to 6 center of mass
    num_B1toB6 = mass_individual[1] * pos_CMB2relIinI + mass_individual[2] * pos_CMB3relIinI + mass_individual[3] * pos_CMB4relIinI + mass_individual[4] * pos_CMB5relIinI + mass_individual[5] * pos_CMB6relIinI
    pos_CM_B1toB6_relIinI = num_B1toB6 / mass_B1toB6_kg


    #Bodies 2 to 6 center of mass
    num_B2toB6 = mass_individual[1] * pos_CMB2relIinI + mass_individual[2] * pos_CMB3relIinI + mass_individual[3] * pos_CMB4relIinI + mass_individual[4] * pos_CMB5relIinI + mass_individual[5] * pos_CMB6relIinI
    pos_CM_B2toB6_relIinI = num_B2toB6 / mass_B2toB6_kg

    #Bodies 3 to 6 center of mass
    num_B3toB6 = mass_individual[2] * pos_CMB3relIinI + mass_individual[3] * pos_CMB4relIinI + mass_individual[4] * pos_CMB5relIinI + mass_individual[5] * pos_CMB6relIinI
    pos_CM_B3toB6_relIinI = num_B3toB6 / mass_B3toB6_kg

    #Bodies 4 to 6 center of mass
    num_B4toB6 = mass_individual[3] * pos_CMB4relIinI + mass_individual[4] * pos_CMB5relIinI + mass_individual[5] * pos_CMB6relIinI
    pos_CM_B4toB6_relIinI = num_B4toB6 / mass_B4toB6_kg

    #Bodies 5 to 6 center of mass
    num_B5toB6 = mass_individual[4] * pos_CMB5relIinI + mass_individual[5] * pos_CMB6relIinI
    pos_CM_B5toB6_relIinI = num_B5toB6 / mass_B5toB6_kg

    #Bodies 6 to 6 center of mass
    pos_CM_B6toB6_relIinI = pos_CMB6relIinI

    #Static Torque Estimate
    grav_frc_B1toB6_inI = mass_B1toB6_kg * grav_acc_mps2 * np.array([0, 0, -1])
    grav_frc_B2toB6_inI = mass_B2toB6_kg * grav_acc_mps2 * np.array([0, 0, -1])
    grav_frc_B3toB6_inI = mass_B3toB6_kg * grav_acc_mps2 * np.array([0, 0, -1])
    grav_frc_B4toB6_inI = mass_B4toB6_kg * grav_acc_mps2 * np.array([0, 0, -1])
    grav_frc_B5toB6_inI = mass_B5toB6_kg * grav_acc_mps2 * np.array([0, 0, -1])
    grav_frc_B6toB6_inI = mass_B6toB6_kg * grav_acc_mps2 * np.array([0, 0, -1])

    #^^^^^ everything above this has been debugged

    #Actuator 1
    trq_CM_B1toB6_relG1inI = np.cross(pos_CM_B1toB6_relIinI.reshape((3,)),grav_frc_B1toB6_inI).reshape((3,1))
    neg_of_trq_sense_axis_ACT1inB1 = np.array([0, 0, -1]).reshape((3,1))
    trq_sense_axis_ACT1inI = np.dot(C_B1toI,neg_of_trq_sense_axis_ACT1inB1)
    calculated_torque[0] = np.dot(trq_sense_axis_ACT1inI.T, trq_CM_B1toB6_relG1inI)

    #Actuator 2
    pos_CM_B2toB6_relG2inI  = pos_CM_B2toB6_relIinI - pos_G2relIinI
    trq_CM_B2toB6_relG2inI = np.cross(pos_CM_B2toB6_relG2inI.reshape((3,)), grav_frc_B2toB6_inI).reshape((3,1))
    neg_of_trq_sense_axis_ACT2inB2 = np.array([0, 1, 0]).reshape((3,1))
    trq_sense_axis_ACT2inI = np.dot(C_B2toI,neg_of_trq_sense_axis_ACT2inB2)
    calculated_torque[1] = np.dot(trq_sense_axis_ACT2inI.T, trq_CM_B2toB6_relG2inI)
    calculated_torque[2] = np.dot(trq_sense_axis_ACT2inI.T, trq_CM_B2toB6_relG2inI)


    #Actuator 3
    pos_CM_B3toB6_relG3inI = pos_CM_B3toB6_relIinI - pos_G3relIinI
    trq_CM_B3toB6_relG3inI = np.cross(pos_CM_B3toB6_relG3inI.reshape((3,)), grav_frc_B3toB6_inI).reshape((3,1))
    neg_of_trq_sense_axis_ACT3inB3 = np.array([-1, 0, 0]).reshape((3,1)) 
    trq_sense_axis_ACT3inI = np.dot(C_B3toI,neg_of_trq_sense_axis_ACT3inB3)
    calculated_torque[3] = np.dot(trq_sense_axis_ACT3inI.T, trq_CM_B3toB6_relG3inI)

    #Actuator 4
    pos_CM_B4toB6_relG4inI = pos_CM_B4toB6_relIinI - pos_G4relIinI
    trq_CM_B4toB6_relG4inI = np.cross(pos_CM_B4toB6_relG4inI.reshape((3,)), grav_frc_B4toB6_inI).reshape((3,1))
    neg_of_trq_sense_axis_ACT4inB4 = np.array([0, 1, 0]).reshape((3,1))
    trq_sense_axis_ACT4inI = np.dot(C_B4toI,neg_of_trq_sense_axis_ACT4inB4)
    calculated_torque[4] = np.dot(trq_sense_axis_ACT4inI.T, trq_CM_B4toB6_relG4inI)

    #Actuator 5
    pos_CM_B5toB6_relG5inI = pos_CM_B5toB6_relIinI - pos_G5relIinI
    trq_CM_B5toB6_relG5inI = np.cross(pos_CM_B5toB6_relG5inI.reshape((3,)), grav_frc_B5toB6_inI).reshape((3,1))
    neg_of_trq_sense_axis_ACT5inB5 = np.array([0, 1, 0]).reshape((3,1))
    trq_sense_axis_ACT5inI = np.dot(C_B5toI,neg_of_trq_sense_axis_ACT5inB5)
    calculated_torque[5] = np.dot(trq_sense_axis_ACT5inI.T, trq_CM_B5toB6_relG5inI)

    #Actuator 6
    pos_CM_B6toB6_relG6inI = pos_CM_B6toB6_relIinI - pos_G6relIinI
    trq_CM_B6toB6_relG6inI = np.cross(pos_CM_B6toB6_relG6inI.reshape((3,)), grav_frc_B6toB6_inI).reshape((3,1))
    neg_of_trq_sense_axis_ACT6inB6 = np.array([-1, 0, 0]).reshape((3,1))
    trq_sense_axis_ACT6inI  = C_B6toI * neg_of_trq_sense_axis_ACT6inB6
    #calculated_torque[6] = np.dot(trq_sense_axis_ACT6inI.T, trq_CM_B6toB6_relG6inI)
    return calculated_torque

def static_calc_params():
    arr_depth = 10
    rolling_window = deque([np.zeros(7)] * arr_depth)
    return rolling_window


def static_torque(theta, rolling_window, t, unwinding_flag, dangerousTorque):
    rolling_window.append(theta.flatten())
    calculated_torque = np.mean(rolling_window, axis=0)
    rolling_window.popleft()
    del_torque = abs(theta) - abs(calculated_torque)
    condition1 = (t > .1) & (del_torque >= 7)
    condition2 = (t > .2) & (del_torque >= 6)
    condition3 = (t > .3) & (del_torque >= 5)
    condition4 = (t > .5) & (del_torque >= 4)
    condition5 = (t > 1)  & (del_torque >= 3)
    conditions = np.array([condition1, condition2, condition3, condition4, condition5], dtype=bool)
    if not unwinding_flag:
        if np.any(conditions):
            print("obstruction at time = {}".format(np.round(t, decimals = 4)))
            print(np.round(theta, decimals=2))
            print(np.round(calculated_torque,decimals = 2))
            print(np.round(del_torque, decimals = 2))
            dangerousTorque = True
        
    return dangerousTorque, rolling_window

