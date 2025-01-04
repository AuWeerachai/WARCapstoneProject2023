
%**************************************************************************
% Function: util_rot_vect_with_quat.m                                     *
% Purpose: Compute the vector rotation using a quaternion                 *
% Date: August 10, 2007                                                   *
% Author: Dan Hill                                                        *
%                                                                         *
% Inputs:                                                                 *
% Vin      = 3 vector in frame A or B                                     *
% Q_AtoB   = Quaternion mapping from frame A to frame B                   *
% ConjFlag = Conjugation flag                                             *
%        0 = Rotation of vector is from frame A to frame B                *
%        1 = Rotation of vector is from frame B to frame A                *
%                                                                         *
% Outputs:                                                                *
% Vout = 3 vector in Frame A or frame B                                   *
%                                                                         *
% Warning: Q_AtoC = Q_BtoC x Q_AtoB is equivalent to                      *
%          C_AtoC = C_BtoC * C_AtoB, where C_XtoY denotes the direction   *
%          cosine matrix mapping frame X to frame Y.                      *
%                                                                         *
%         The quaternion to DCM equivalence is based on the utilities     *
%         util_quat_to_DC and util_DC_to_quat. If the order is reversed,  *
%         or the left handed conversion utilities are used, then the      *
%         equivalence will be in error.                                   *
%                                                                         *
% Note: Quaternion convention assumes Q(4) is the scalar.                 *
%*************************************************************************/

function Vout = util_rot_vect_with_quat(Vin,Q_AtoB,ConjFlag)

if (ConjFlag == 0)
   Vout = (2*Q_AtoB(4)*Q_AtoB(4) - 1)*Vin    ...
        + 2*dot(Vin,Q_AtoB(1:3))*Q_AtoB(1:3) ...
        + 2*Q_AtoB(4)*cross(Vin,Q_AtoB(1:3));
else
   Vout = (2*Q_AtoB(4)*Q_AtoB(4) - 1)*Vin    ...
        + 2*dot(Vin,Q_AtoB(1:3))*Q_AtoB(1:3) ...
        - 2*Q_AtoB(4)*cross(Vin,Q_AtoB(1:3)); 
end
return