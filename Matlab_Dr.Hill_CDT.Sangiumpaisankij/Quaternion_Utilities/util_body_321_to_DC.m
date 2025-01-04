
%**************************************************************************
% Function: util_body_321_to_DC.m                                         *
% Purpose: Compute the DCM from the body 321 rotation orientation angle   *
%          sequence.                                                      *
%                                                                         *
% Inputs:                                                                 *
% THETA = 3 x 1 body 321 rotation orientation angle vector                *
% Row 1 = Roll angle (radians)                                             *
% Row 2 = Pitch angle (radians)                                           *
% Row 3 = Yaw angle (radians)                                            *
%                                                                         *
% Outputs:                                                                *
% C_AtoB = DCM mapping frame A to frame B                                 *
%                                                                         *
%*************************************************************************/

function C_A2B = util_body_321_to_DC(THETA)

C_A2B = zeros(3,3);
 
cs_th1 = cos(THETA(3)); sn_th1 = sin(THETA(3));

cs_th2 = cos(THETA(2)); sn_th2 = sin(THETA(2));

cs_th3 = cos(THETA(1)); sn_th3 = sin(THETA(1));

C_A2B(1,1) =  cs_th1 * cs_th2;

C_A2B(2,1) =  cs_th1 * sn_th2 * sn_th3 - sn_th1 * cs_th3;

C_A2B(3,1) =  cs_th1 * sn_th2 * cs_th3 + sn_th1 * sn_th3;

C_A2B(1,2) =  sn_th1 * cs_th2;

C_A2B(2,2) =  sn_th1 * sn_th2 * sn_th3 + cs_th1 * cs_th3;

C_A2B(3,2) =  sn_th1 * sn_th2 * cs_th3 - cs_th1 * sn_th3;

C_A2B(1,3) = -sn_th2;

C_A2B(2,3) =  cs_th2 * sn_th3;

C_A2B(3,3) =  cs_th2 * cs_th3;

return