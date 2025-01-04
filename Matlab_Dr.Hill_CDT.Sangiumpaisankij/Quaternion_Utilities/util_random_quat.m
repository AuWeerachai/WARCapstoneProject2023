
%**************************************************************************
% Function: util_random_quat.m                                            *
% Purpose: Compute quaternion to DCM map                                  *
% Date: August 10, 2007                                                   *
% Author: Dan Hill                                                        *
%                                                                         *
% Inputs:                                                                 *
% N = Number of random quaternions                                        *
%                                                                         *
% Outputs:                                                                *
% Q = 4 x N array of random quaternions                                   *
%*************************************************************************/

function Q = util_random_quat(N)

Q = zeros(4,N);
u = randn(3,N);
for i=1:N
   u(:,i) = u(:,i)/norm(u(:,i)); 
end

Theta = 2*pi*rand(N,1);
for i=1:N
  Q(1:3,i) = u(:,i)*sin(0.5*Theta(i));
  Q(4,i) = cos(0.5*Theta(i));
  if (Q(4,i) < 0)
     Q(:,i) = -Q(:,i); 
  end
end
return