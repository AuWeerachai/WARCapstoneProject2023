%% Run this code first

%% specify where to go (final and start destination)

%[0.56515 -0.18415 0.33074533723830762399487070979376] %Home 

%COORDINATE OF TABLE WORKSPACE OF THE WARS ROBOT 
%[0.5  0     0.4] %TOP MIDDLE
%[0.5  0.44  0.4] %TOP LEFT
%[0.5 -0.44  0.4] %TOP RIGHT
%[0    0.44  0.4] %BOTTOM LEFT
%[0   -0.44 0.4]  %BOTTOM RIGHT

efi = [0.56515 -0.18415 0.33074533723830762399487070979376]  %initial point coordinate
eff = [0.5  0.44  0.4]  %final point cooridnate

%specify how many points (from waypoint 1 to waypoint n which is the end
%point)
%n_point will equal to number of point without counting starting point

%travel y-way = 23 points from left or right  (4 cm apart)
%travel x-way = 11 points from top to bottom  (5 cm apart)

n_point = 5

%% vector calculation
ef_vec = [(eff(1)-efi(1)) (eff(2)-efi(2)) (eff(3)-efi(3)) ]    %vector
length = sqrt((ef_vec(1))^2 + (ef_vec(2))^2 + (ef_vec(3))^2)   %length of vector
ef_unitvec = ef_vec/length                                     %unit vector
 
point = []       %Original version, starting point not included.

unit_length = length/n_point
for i=1:n_point
    point = [point; efi(1) + ef_unitvec(1)*unit_length*i efi(2) + ef_unitvec(2)*unit_length*i efi(3) + ef_unitvec(3)*unit_length*i ];
end

point                                         %print point on command chat
plot3(point(:,1),point(:,2),point(:,3),'.')
xlim([-1.5,1.5])
ylim([-1.5,1.5])
zlim([-1.5,1.5])



