
%**************************************************************************
% Function: util_quat_multiply.m                                          *
% Purpose: Compute quaternion multiplication                              *
% Date: August 10, 2007                                                   *
% Author: Dan Hill                                                        *
%                                                                         *
% Inputs:                                                                 *
% Q_BtoC = Quaternion for the rotation of frame B to frame C              *
% Q_AtoB = Quaternion for the rotation of frame B to frame C              *
%                                                                         *
% Outputs:                                                                *
% Q_AtoC = Quaternion for the rotation of frame A to frame C              *
%                                                                         *
% Warning: Q_AtoC = Q_BtoC x Q_AtoB is equivalent to                      *
%          C_AtoC = C_BtoC * C_AtoB, where C_XtoY denotes the direction   *
%         cosine matrix mapping frame X to frame Y.                       *
%                                                                         *
%         The quaternion to DCM equivalence si based on the utilities     *
%         util_quat_to_DC and util_DC_to_quat. If the order is reversed,  *
%         or the left handed conversion utilities are used, then the      *
%         equivalence will be in error.                                   * 
%                                                                         *
% Note: Quaternion convention assumes Q[4] is the scalar.                 *
%*************************************************************************/

function Q_AtoC = util_quat_multiply(Q_BtoC, Q_AtoB)

Q_AtoC = zeros(4,1);

Q_AtoC(1) =  Q_AtoB(4)*Q_BtoC(1) - Q_AtoB(3)*Q_BtoC(2) ...
          +  Q_AtoB(2)*Q_BtoC(3) + Q_AtoB(1)*Q_BtoC(4);
  
Q_AtoC(2) =  Q_AtoB(3)*Q_BtoC(1) + Q_AtoB(4)*Q_BtoC(2) ...
          -  Q_AtoB(1)*Q_BtoC(3) + Q_AtoB(2)*Q_BtoC(4);
    
Q_AtoC(3) = -Q_AtoB(2)*Q_BtoC(1) + Q_AtoB(1)*Q_BtoC(2) ...
          +  Q_AtoB(4)*Q_BtoC(3) + Q_AtoB(3)*Q_BtoC(4);
  
Q_AtoC(4) = -Q_AtoB(1)*Q_BtoC(1) - Q_AtoB(2)*Q_BtoC(2) ...
            -Q_AtoB(3)*Q_BtoC(3) + Q_AtoB(4)*Q_BtoC(4);
return        