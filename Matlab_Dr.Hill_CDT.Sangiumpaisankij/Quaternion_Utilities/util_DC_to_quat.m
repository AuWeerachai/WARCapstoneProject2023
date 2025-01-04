
%**************************************************************************
% Function: util_DC_to_quat.m                                             *
% Purpose: Compute DCM to quaternion map                                  *
% Date: August 10, 2007                                                   *
% Author: Dan Hill                                                        *
%                                                                         *
% Inputs:                                                                 *
% C_AtoB = DCM mapping frame A to frame B                                 *
%                                                                         *
% Outputs:                                                                *
% Q_AtoB = Quaternion mapping from frame A to frame B                     *
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

function Q_AtoB = util_DC_to_quat(C_AtoB)

Q_AtoB = zeros(4,1); 
TempQuat = zeros(4,1); 
InsideSquareRoot = zeros(4,1);


MatrixTrace = C_AtoB(1,1) + C_AtoB(2,2) + C_AtoB(3,3);

for i=1:3
  InsideSquareRoot(i) = 1.0 + 2.0*C_AtoB(i,i) - MatrixTrace;    
end

InsideSquareRoot(4) = 1.0 + MatrixTrace; 
PivotNumber = 1;

for i=1:3
  if ( InsideSquareRoot(i+1) > InsideSquareRoot(PivotNumber) ) 
     PivotNumber = i + 1; 
  end
end

if (InsideSquareRoot(PivotNumber) > 0.0)
   Pivot = 2.0*sqrt(InsideSquareRoot(PivotNumber));
   if (Pivot > 1.0e-12)
      switch PivotNumber
        case 1 
          TempQuat(1) = 0.25*Pivot;
          TempQuat(2) = (C_AtoB(2,1) + C_AtoB(1,2))/Pivot;
          TempQuat(3) = (C_AtoB(1,3) + C_AtoB(3,1))/Pivot;
          TempQuat(4) = (C_AtoB(2,3) - C_AtoB(3,2))/Pivot;
        case 2
          TempQuat(1) = (C_AtoB(2,1) + C_AtoB(1,2))/Pivot;
          TempQuat(2) = 0.25*Pivot;
          TempQuat(3) = (C_AtoB(3,2) + C_AtoB(2,3))/Pivot;
          TempQuat(4) = (C_AtoB(3,1) - C_AtoB(1,3))/Pivot;       
        case 3
          TempQuat(1) = (C_AtoB(1,3) + C_AtoB(3,1))/Pivot;
          TempQuat(2) = (C_AtoB(3,2) + C_AtoB(2,3))/Pivot;
          TempQuat(3) = 0.25*Pivot;
          TempQuat(4) = (C_AtoB(1,2) - C_AtoB(2,1))/Pivot;     
        case 4
          TempQuat(1) = (C_AtoB(2,3) - C_AtoB(3,2))/Pivot;
          TempQuat(2) = (C_AtoB(3,1) - C_AtoB(1,3))/Pivot;
          TempQuat(3) = (C_AtoB(1,2) - C_AtoB(2,1))/Pivot; 
          TempQuat(4) = 0.25*Pivot;
      end
      Q_AtoB = TempQuat/norm(TempQuat);
      if (Q_AtoB(4) < 0)
         Q_AtoB = -Q_AtoB; 
      end
   else
      Q_AtoB = [0 0 0 1]'; 
   end
else
   Q_AtoB = [0 0 0 1]'; 
end
return