
clear all
close all
clc

theta = [0.6 0.5 0.3]';

C_A2B     = util_body_321_to_DC(theta)
C_A2B_mex = util_body_321_to_DC_mex(theta)

q_A2B     = util_body_321_to_quat(theta)
q_A2B_mex = util_body_321_to_quat_mex(theta)

theta     = util_DC_to_body321(C_A2B)
theta_mex = util_DC_to_body321_mex(C_A2B_mex)

q_A2B     = util_DC_to_quat(C_A2B)
q_A2B_mex = util_DC_to_quat_mex(C_A2B_mex)

q_A2B_conj     = util_quat_conj(q_A2B)
q_A2B_conj_mex = util_quat_conj_mex(q_A2B_mex)

q1 = util_random_quat(2)';
q2 = util_random_quat(2)';

qerr     = util_quat_error(q1, q2)
qerr_mex = util_quat_error_mex(q1, q2)

s = RandStream('mt19937ar','Seed',1);
RandStream.setGlobalStream(s);

q_B2C = util_random_quat(1)

s = RandStream('mt19937ar','Seed',1);
RandStream.setGlobalStream(s);

q_B2C_mex = util_random_quat_mex(1)

q_A2C     = util_quat_multiply(q_B2C, q_A2B)
q_A2C_mex = util_quat_multiply_mex(q_B2C, q_A2B)

theta     = util_quat_to_body_321(q_A2B)
theta_mex = util_quat_to_body_321_mex(q_A2B)

C_A2B     = util_quat_to_DC(q_A2B)
C_A2B_mex = util_quat_to_DC(q_A2B_mex)

s = RandStream('mt19937ar','Seed',1);
RandStream.setGlobalStream(s);

vin = util_random3Dvector(4)

s = RandStream('mt19937ar','Seed',1);
RandStream.setGlobalStream(s);

vin_mex = util_random3Dvector_mex(4)

va     = util_rot_vect_with_quat(vin(:,1), q_A2B, 0)
va_mex = util_rot_vect_with_quat(vin_mex(:,1), q_A2B_mex, 0)

vb     = util_rot_vect_with_quat(vin(:,1), q_A2B, 1)
vb_mex = util_rot_vect_with_quat_mex(vin_mex(:,1), q_A2B_mex, 1)
