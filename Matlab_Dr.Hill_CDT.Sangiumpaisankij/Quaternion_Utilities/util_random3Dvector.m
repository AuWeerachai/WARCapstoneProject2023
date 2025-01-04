
%**************************************************************************
% Function: util_random3Dvector.m                                         *
% Purpose: Compute uniformly distributed random vectors over the unit     * 
%          sphere                                                         *
% Date: August 10, 2007                                                   *
% Author: Dan Hill                                                        *
%                                                                         *
% Inputs:                                                                 *
% N = Number of random vectors                                            *
%                                                                         *
% Outputs:                                                                *
% V = 3 x N array of random vectors                                       *
%*************************************************************************/

function V = util_random3Dvector(N)

V = zeros(3,N);
for i=1:N
   az = rand(1);
   el = rand(1);
   theta = 2*pi*az;
   phi = asin(2*el - 1);
   [Vx, Vy, Vz] = sph2cart(theta,phi,1); 
   V(:,i) = [Vx Vy Vz]'/norm([Vx Vy Vz]);
end
return