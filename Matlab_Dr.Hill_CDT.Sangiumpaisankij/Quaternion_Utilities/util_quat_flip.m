
%**************************************************************************
% Function: util_quat_flip.m                                              *
% Purpose: Flip quaternion sign                                           *
%                                                                         *
% Date: August 10, 2007                                                   *
% Author: Dan Hill                                                        *
%                                                                         *
% Inputs:                                                                 *
% qIn     = 4 vector of current quaternion                                *
% qInPrev = 4 vector of previous quaternion                               *
%                                                                         *
% Outputs:                                                                *
% qOut = 4 vector of output quaternion                                    *
%*************************************************************************/

function qOut  = util_quat_flip(qIn,qInPrev)
if (qIn'*qInPrev < 0)
   qOut = -qIn;
else
   qOut = qIn; 
end
return