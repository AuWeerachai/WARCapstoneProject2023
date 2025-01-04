
clear all
close all
clc

theta = coder.typeof(double(0), [3 1]);
codegen util_body_321_to_DC -args {theta} -nargout 1

theta = coder.typeof(double(0), [3 1]);
codegen util_body_321_to_quat -args {theta} -nargout 1

C_AtoB = coder.typeof(double(0), [3 3]);
codegen util_DC_to_body321 -args {C_AtoB} -nargout 1

C_AtoB = coder.typeof(double(0), [3 3]);
codegen util_DC_to_quat -args {C_AtoB} -nargout 1

q = coder.typeof(double(0), [4 1]);
codegen util_quat_conj -args {q} -nargout 1

q1 = coder.typeof(double(0), [Inf 4]);
q2 = coder.typeof(double(0), [Inf 4]);
codegen util_quat_error -args {q1, q2} -nargout 1

qIn = coder.typeof(double(0), [4 1]);
qInPrev = coder.typeof(double(0), [4 1]);
codegen util_quat_flip -args {qIn, qInPrev} -nargout 1

Q_BtoC = coder.typeof(double(0), [4 1]);
Q_AtoB = coder.typeof(double(0), [4 1]);
codegen util_quat_multiply -args {Q_BtoC, Q_AtoB} -nargout 1

Q = coder.typeof(double(0), [4 1]);
codegen util_quat_to_body_321 -args {Q} -nargout 1

Q_AtoB = coder.typeof(double(0), [4 1]);
codegen util_quat_to_DC -args {Q_AtoB} -nargout 1

N = coder.typeof(double(0), [1 1]);
codegen util_random_quat -args {N} -nargout 1

N = coder.typeof(double(0), [1 1]);
codegen util_random3Dvector -args {N} -nargout 1

Vin = coder.typeof(double(0), [3 1]);
Q_AtoB = coder.typeof(double(0), [4 1]);
ConjFlag = coder.typeof(double(0), [1 1]);
codegen util_rot_vect_with_quat -args {Vin, Q_AtoB, ConjFlag} -nargout 1
