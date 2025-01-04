%% Run this code when want to know attitude and position with given joint angles

%% Input angle in radian

the = [1.0017923479602684315821802603358
   -0.70344906463106883969825002394161
0
    0.15468160144569859283539306345382
     2.1195637899802668660941786521275
     1.0017923479602684315821802603358]

the1 = the(1)
the2 = the(2)
the3 = the(3)
the4 = the(4)
the5 = the(5)
the6 = the(6)


% the1 = 459.78950331002296137418953531166
% the2 = -76.507220898537664145430272649457
% the3 = -138.23007675795090249235630471826
% the4 = -872.42270083406202517357745815467
% the5 = 454.12907879220725426114139821949
% the6 = 19.966531807451907989419466267121
% 
% z1 = mod(the1, 2*pi)
% z2 = mod(the2,2*pi)
% z3 = mod(the3,2*pi)
% z4 = mod(the4,2*pi)
% z5 = mod(the5,2*pi)
% z6 = mod(the6,2*pi)

%% Input angle in degree
% This angle will be real angle based on coordinate system. The angle of
% robot must be flip for th2,4,5

% the1 = 0;
% the2 = 0;
% the3 = 0;
% the4 = 0;
% the5 = 90;
% the6 = 0;
% 
% z1 = the1*pi/180
% z2 = the2*pi/180
% z3 = the3*pi/180
% z4 = the4*pi/180
% z5 = the5*pi/180
% z6 = the6*pi/180


%% Check
A01 = [cos(z1) -sin(z1) 0 0
       sin(z1)  cos(z1) 0 0
         0        0     1 L1
         0        0     0 1 ];
                         
A12 = [cos(z2) 0 sin(z2) L2
       0 1 0 0
       -sin(z2) 0 cos(z2) 0
       0 0 0 1];

A23 = [1 0 0 L3
       0 cos(z3) -sin(z3) 0
       0 sin(z3) cos(z3) 0
       0 0 0 1];

A34 = [cos(z4) 0 sin(z4) L4
       0 1 0 -L7
       -sin(z4) 0 cos(z4) 0
       0 0 0 1];

A45 = [cos(z5) 0 sin(z5) L5
       0 1 0 -L8
       -sin(z5) 0 cos(z5) 0
       0 0 0 1];

A56 = [1 0 0 L6
       0 cos(z6) -sin(z6) -L9
       0 sin(z6) cos(z6) 0
       0 0 0 1];

A6E = [1 0 0 L10
       0 1 0 0
       0 0 1 0
       0 0 0 1];

format short G
Mapsym = vpa(A01*A12*A23*A34*A45*A56*A6E)




