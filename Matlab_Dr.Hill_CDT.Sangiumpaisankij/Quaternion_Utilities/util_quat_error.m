
%**************************************************************************
% Function: util_quat_error.m                                             *
% Purpose: Compute the quaternion error q1 x q2^-1                        *
%                                                                         *
% Date: August 10, 2007                                                   *
% Author: Dan Hill                                                        *
%                                                                         *
% Inputs:                                                                 *
% q1 = m x 4 array of quaternions                                         *
% q2 = m x 4 array of quaternions                                         *
%                                                                         *
% Outputs:                                                                *
% qout = m x 4 array of quaternions                                       *
%                                                                         *
% Note: Quaternion convention assumes Q(4) is the scalar.                 *
%*************************************************************************/

function qout = util_quat_error(q1,q2)

dim_of_q1 = size(q1,1);

qout = zeros(dim_of_q1,4);

q2(:,1:3) = -q2(:,1:3);

qout(:,1) =  q1(:,4).*q2(:,1) + q1(:,3).*q2(:,2) ...
          -  q1(:,2).*q2(:,3) + q1(:,1).*q2(:,4);
      
qout(:,2) = -q1(:,3).*q2(:,1) + q1(:,4).*q2(:,2) ...
          +  q1(:,1).*q2(:,3) + q1(:,2).*q2(:,4);
      
qout(:,3) =  q1(:,2).*q2(:,1) - q1(:,1).*q2(:,2) ...
          +  q1(:,4).*q2(:,3) + q1(:,3).*q2(:,4);
      
qout(:,4) = -q1(:,1).*q2(:,1) - q1(:,2).*q2(:,2) ...
          -  q1(:,2).*q2(:,3) + q1(:,4).*q2(:,4);
      
qnorm = (qout(:,1).^2 + qout(:,2).^2 + qout(:,3).^2 + qout(:,4).^2).^0.5;
qout(:,1) = qout(:,1)./qnorm;
qout(:,2) = qout(:,2)./qnorm;
qout(:,3) = qout(:,3)./qnorm;
qout(:,4) = qout(:,4)./qnorm;
return     