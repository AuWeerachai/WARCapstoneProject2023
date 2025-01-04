
%**************************************************************************
% Function: util_body_321_to_quat.m                                       *
% Purpose: Compute the quaternion representing the body 321 rotation      *
%          orientation angle sequence.                                    *
%                                                                         *
% Inputs:                                                                 *
% THETA = 3 x 1 body 321 rotation orientation angle vector                *
% Row 1 = Roll angle (radians)                                            *
% Row 2 = Pitch angle (radians)                                           *
% Row 3 = Yaw angle (radians)                                             *
%                                                                         *
% Outputs:                                                                *
% Q = 4 x 1 quaternion vector                                             *
%                                                                         *
% Note: Quaternion convention assumes Q(4) is the scalar.                 *
%*************************************************************************/

function Q = util_body_321_to_quat(THETA)
Q = zeros(4,1); HalfTheta = 0.5*THETA;
cosHalfPhi   = cos(HalfTheta(1)); sinHalfPhi   = sin(HalfTheta(1));
cosHalfTheta = cos(HalfTheta(2)); sinHalfTheta = sin(HalfTheta(2));
cosHalfPsi   = cos(HalfTheta(3)); sinHalfPsi   = sin(HalfTheta(3)); 
Q(1) = cosHalfPsi*cosHalfTheta*sinHalfPhi - sinHalfPsi*sinHalfTheta*cosHalfPhi;
Q(2) = cosHalfPsi*sinHalfTheta*cosHalfPhi + sinHalfPsi*cosHalfTheta*sinHalfPhi;
Q(3) = sinHalfPsi*cosHalfTheta*cosHalfPhi - cosHalfPsi*sinHalfTheta*sinHalfPhi;
Q(4) = cosHalfPsi*cosHalfTheta*cosHalfPhi + sinHalfPsi*sinHalfTheta*sinHalfPhi;
if (Q(4) < 0)
    Q = -Q;
end
return