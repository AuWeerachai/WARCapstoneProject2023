
%**************************************************************************
% Function: util_quat_to_body_321.m                                       *
% Purpose: Compute the body 321 rotation orientation angle sequence       *
%          representing the quaternion.                                   *
%                                                                         *
% Inputs:                                                                 *
% Q = 4 x 1 or 1 x 4 quaternion vector                                    *
%                                                                         *
% Outputs:                                                                *
% THETA = 3 x 1 body 321 rotation orientation angle vector                *
% Row 1 = Roll angle (radians)                                             *
% Row 2 = Pitch angle (radians)                                           *
% Row 3 = Yaw angle (radians)                                            *
%                                                                         *
% Note: Quaternion convention assumes Q(4) is the scalar.                 *
%*************************************************************************/

function THETA = util_quat_to_body_321(Q)
THETA = zeros(3,1);
m11 = 2*( Q(1)*Q(1) + Q(4)*Q(4) ) - 1;
m12 = 2*( Q(1)*Q(2) + Q(3)*Q(4) );
m13 = 2*( Q(1)*Q(3) - Q(2)*Q(4) );
m23 = 2*( Q(2)*Q(3) + Q(1)*Q(4) );
m33 = 2*( Q(3)*Q(3) + Q(4)*Q(4) ) - 1;
THETA(1) = atan2(m23,m33);
THETA(2) = real(asin(-m13));
THETA(3) = atan2(m12,m11);
return