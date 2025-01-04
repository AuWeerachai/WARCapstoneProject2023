
%**************************************************************************
% Function: util_DC_to_body_321.m                                         *
% Purpose: Compute the body 321 rotation orientation angle sequence from  *
%          the DCM.                                                       *
%                                                                         *
% Inputs:                                                                 *
% C_AtoB = DCM mapping frame A to frame B                                 *
%                                                                         *
% Outputs:                                                                *
% THETA = 3 x 1 body 321 rotation orientation angle vector                *
% Row 1 = Roll angle (radians)                                             *
% Row 2 = Pitch angle (radians)                                           *
% Row 3 = Yaw angle (radians)                                            *
%                                                                         *
%*************************************************************************/

function THETA = util_DC_to_body321(C_AtoB)
THETA = zeros(3,1);
THETA(1) = atan2(C_AtoB(2,3),C_AtoB(3,3));
THETA(2) = real(asin(-C_AtoB(1,3)));
THETA(3) = atan2(C_AtoB(1,2),C_AtoB(1,1));
return