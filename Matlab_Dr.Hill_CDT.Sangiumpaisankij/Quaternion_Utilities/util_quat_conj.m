
%**************************************************************************
% Function: util_quat_conj.m                                              *
% Purpose: Compute the quaternion conjugate                               *
%                                                                         *
% Date: August 10, 2007                                                   *
% Author: Dan Hill                                                        *
%                                                                         *
% Inputs:                                                                 *
% q = 4 x 1 or 1 x 4 quaternion                                           *
%                                                                         *
% Outputs:                                                                *
% qConj = 4 x 1 or 1 x 4 quaternion conjugate                             *
%                                                                         *
% Note: Quaternion convention assumes Q(4) is the scalar.                 *
%*************************************************************************/

function qConj = util_quat_conj(q)
qConj = q;
qConj(1:3) = -q(1:3);
return