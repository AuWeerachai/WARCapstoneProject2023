
%**************************************************************************
% Function: util_quat_to_DC.m                                             *
% Purpose: Compute quaternion to DCM map                                  *
% Date: August 10, 2007                                                   *
% Author: Dan Hill                                                        *
%                                                                         *
% Inputs:                                                                 *
% Q_AtoB = Quaternion for the map from frame A to frame B                 *
%                                                                         *
% Outputs:                                                                *
% C_AtoB = DCM mapping frame A to frame B                                 *
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

function C_AtoB = util_quat_to_DC(Q_AtoB)

C_AtoB = zeros(3,3);

C_AtoB(1,1) = 2.0*(Q_AtoB(1)*Q_AtoB(1) + Q_AtoB(4)*Q_AtoB(4)) - 1.0;
C_AtoB(2,1) = 2.0*(Q_AtoB(1)*Q_AtoB(2) - Q_AtoB(3)*Q_AtoB(4));
C_AtoB(3,1) = 2.0*(Q_AtoB(1)*Q_AtoB(3) + Q_AtoB(2)*Q_AtoB(4));
  
C_AtoB(1,2) = 2.0*(Q_AtoB(1)*Q_AtoB(2) + Q_AtoB(3)*Q_AtoB(4));
C_AtoB(2,2) = 2.0*(Q_AtoB(2)*Q_AtoB(2) + Q_AtoB(4)*Q_AtoB(4)) - 1.0;
C_AtoB(3,2) = 2.0*(Q_AtoB(2)*Q_AtoB(3) - Q_AtoB(1)*Q_AtoB(4));
  
C_AtoB(1,3) = 2.0*(Q_AtoB(1)*Q_AtoB(3) - Q_AtoB(2)*Q_AtoB(4));
C_AtoB(2,3) = 2.0*(Q_AtoB(2)*Q_AtoB(3) + Q_AtoB(1)*Q_AtoB(4));
C_AtoB(3,3) = 2.0*(Q_AtoB(3)*Q_AtoB(3) + Q_AtoB(4)*Q_AtoB(4)) - 1.0;
return