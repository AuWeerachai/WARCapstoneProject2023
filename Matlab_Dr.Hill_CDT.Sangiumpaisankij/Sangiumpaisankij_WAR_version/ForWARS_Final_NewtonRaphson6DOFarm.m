clc
syms z1 z2 z3 z4 z5 z6 ax ay az ox oy oz nx ny nz px py pz L1 L2 L3 L4 L5 L6 L7 L8 L9 L10
%% Define the start gimball angle and destination orientation (will be the same as start position)

% %TOP MIDDLE
% th1 = 21.6108;
% th2 = -73.2142;
% th3 = 0;
% th4 = 68.6421;
% th5 = 94.5721;
% th6 = 21.6108;

%TOP LEFT
% th1 = 57.3985;
% th2 = -40.3045;
% th3 = 0;
% th4 = 8.8623;
% th5 = 121.4422;
% th6 = 57.3985;

%TOP RIGHT
% th1 = -25.2971;
% th2 = -40.3045;
% th3 = 0;
% th4 = 8.8623;
% th5 = 121.4422;
% th6 = -25.2970;

%BOTTOM LEFT
% th1 = 114.7414;
% th2 = -81.4809;
% th3 = 0;
% th4 = 79.4877;
% th5 = 91.9932;
% th6 = 114.7414;

%BOTTOM RIGHT
% th1 = -65.2586;
% th2 = -81.4809;
% th3 = 0;
% th4 = 79.4877;
% th5 = 91.9932;
% th6 = -65.2586;

th1 = 0;
th2 = -60;
th3 = 0;
th4 = 60
th5 = 90;
th6 = 0;





%%% orientation of start and final of end effector (A,O,N vector)
Ades = [0 0 -1];
Odes = [0 1 0];
Ndes = [1 0 0];

%% symbo math

tic

%Map from the latter to present one
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

Mapsym = A01*A12*A23*A34*A45*A56*A6E;


%How much each vector is in x y z global
Amapsym=Mapsym(1:3,1); %vector of pointing at +X originally = cone pointing (for orientation, this one is the focus)
Omapsym=Mapsym(1:3,2); %vector of pointing at +Y
Nmapsym=Mapsym(1:3,3); %vector of pointing at +Z
Pmapsym=Mapsym(1:3,4); % metre in x,y,z in global from origin for end-effector

ax=Amapsym(1,1);ay=Amapsym(2,1);az=Amapsym(3,1);
ox=Omapsym(1,1);oy=Omapsym(2,1);oz=Omapsym(3,1);
nx=Nmapsym(1,1);ny=Nmapsym(2,1);nz=Nmapsym(3,1);
px=Pmapsym(1,1);py=Pmapsym(2,1);pz=Pmapsym(3,1);

Zvecsym = [z1;z2;z3;z4;z5;z6];
Fsym = transpose([ax ay az ox oy oz nx ny nz px py pz]);

Avecsym= [ax,ay,az];
Ovecsym= [ox,oy,oz];
Nvecsym= [nx,ny nz];
Pvecsym= [px,py,pz];

% find the rate of change 
J_a = jacobian(Avecsym,Zvecsym);
J_o = jacobian(Ovecsym,Zvecsym);
J_n = jacobian(Nvecsym,Zvecsym);
J_p = jacobian(Pvecsym,Zvecsym);
J = [J_a
    J_o
    J_n
    J_p];

%%Symbo math end

%% Fix length and prepare the value for substitution 
%Fix length
Le1= 4.25*0.0254;
Le2= 0*0.0254;
Le3= 5.5*0.0254;
Le4= 13*0.0254;
Le5= 13*0.0254;
Le6= 3*0.0254;
Le7= 2.5*0.0254; 
Le8= 3.25*0.0254;
Le9= 1.5*0.0254; 
Le10= 4.25*0.0254; 

L1=Le1;L2=Le2;L3=Le3;L4=Le4;L5=Le5;
L6=Le6;L7=Le7;L8=Le8;L9=Le9;L10=Le10;

%%%
z1=th1*(pi/180);
z2=th2*(pi/180);
z3=th3*(pi/180);
z4=th4*(pi/180);
z5=th5*(pi/180);
z6=th6*(pi/180);


%% Loop 
tolerance = 0.01;

%%parameter used to count the loop
loopcount = 1;
looptarget = n_point;
pointend=0;
iteration=0;
iterationPerPoint_array=[];
point_array=[];

%%data collection for Arm graphic plots
F01_array=[];
F02_array=[];
F03_array=[];
F04_array=[];
F05_array=[];
F06_array=[];
F0E_array=[];

angle_array_rad = [];


% loop begin
for i=1:n_point
    iterationPerPoint=0;
    Fdes = transpose([Ades(1),Ades(2),Ades(3),Odes(1),Odes(2),Odes(3),Ndes(1),Ndes(2),Ndes(3),point(i,1),point(i,2),point(i,3)]);
    check = false;  
    
    while check == false   
        Zvec = subs(Zvecsym);
        Fnow=vpa(subs(Fsym));
        Jnum = vpa(subs(J));
        x=pinv(Jnum)*(Fdes-Fnow);
        z1=z1+x(1,1);
        z2=z2+x(2,1);
        z3=z3+x(3,1);
        z4=z4+x(4,1);
        z5=z5+x(5,1);
        z6=z6+x(6,1);
        if abs(x)<tolerance
            check = true;
        else
            check = false;
        end
    iteration=iteration+1
    iterationPerPoint=iterationPerPoint+1;
    end

    F01=vpa(subs(A01));
    F02=vpa(subs(A01*A12));
    F03=vpa(subs(A01*A12*A23));
    F04=vpa(subs(A01*A12*A23*A34));
    F05=vpa(subs(A01*A12*A23*A34*A45));
    F06=vpa(subs(A01*A12*A23*A34*A45*A56));
    F0E=vpa(subs(A01*A12*A23*A34*A45*A56*A6E));
    
    F01_array=[F01_array;F01];
    F02_array=[F02_array;F02];
    F03_array=[F03_array;F03];
    F04_array=[F04_array;F04];
    F05_array=[F05_array;F05];
    F06_array=[F06_array;F06];
    F0E_array=[F0E_array;F0E];

    angle_array_rad = [angle_array_rad; z1,z2,z3,z4,z5,z6];

    pointend=pointend+1
    iterationPerPoint_array=[iterationPerPoint_array,iterationPerPoint];
    point_array=[point_array,pointend];

end


fprintf("**************************** DONE ******************************************")
Total_Iteration = iteration
toc

angle_radian = vpa(subs(Zvecsym))
angle_degree = vpa(angle_radian.*180./pi)
for i = 1:numel(angle_degree)
    if angle_degree(i)>=0
        angle_degree(i) = mod(angle_degree(i),360);
    elseif angle_degree(i)<0
        angle_degree(i) = mod(angle_degree(i),-360);
    end
end
mod_angle_degree = angle_degree
figure(1)
hold on
plot(point_array,iterationPerPoint_array)
plot(point_array,iterationPerPoint_array,'.')
xlabel("points between start and final")
ylabel("iteration of NewtonRaphson to be there")

%%%%%

angle_for_hebi_rad = vpa([mod_angle_degree(1)*(pi/180) 
                         mod_angle_degree(2)*(-pi/180)
                         (mod_angle_degree(3)*(pi/180))-(pi/2)
                         mod_angle_degree(4)*(-pi/180)
                         mod_angle_degree(5)*(-pi/180)
                         mod_angle_degree(6)*(pi/180)])


fprintf("**************************** Array zone ******************************************")

vpa(angle_array_rad)
vpa(mod(angle_array_rad,2*pi))

angle_array_degree = vpa(angle_array_rad.*(180/pi))
[row,col] = size(angle_array_rad);
modder = [];


%Modular the whole matrix for smaller angle, based on the first point (not counting start point)
for j=1:col
    if angle_array_degree(1,j) >= 0
        modder = [modder,angle_array_degree(1,j)-mod(angle_array_degree(1,j),360)];

    elseif angle_array_degree(1,j) < 0
        modder = [modder,angle_array_degree(1,j)-mod(angle_array_degree(1,j),-360)];
    end
end


for i=1:row
    for j=1:col
            angle_array_degree(i,j) = angle_array_degree(i,j) - modder(1,j);
    end
end

modder;
angle_array_degree_mod = angle_array_degree


angle_array_degree_180 = vpa(angle_array_degree_mod);
[p,q] = size(angle_array_degree_180);

for i=1:q
    if angle_array_degree_180(1,i) > 0 && angle_array_degree_180(1,i) > 180
        angle_array_degree_180(:,i) = -360 + angle_array_degree_180(:,i);
    elseif angle_array_degree_180(1,i) < 0 && angle_array_degree_180(1,i) < -180
        angle_array_degree_180(:,i) = 360 + angle_array_degree_180(:,i);
    end
end


angle_array_degree_180
hebi = angle_array_degree_180;
angle_for_hebi_rad_array = vpa([hebi(:,1)*(pi/180),hebi(:,2)*(-pi/180),(hebi(:,3)*(pi/180))-(pi/2), hebi(:,4)*(-pi/180),hebi(:,5)*(-pi/180),hebi(:,6)*(pi/180)]) 
angle_for_hebi_degree_array = vpa([hebi(:,1),hebi(:,2)*-1,(hebi(:,3))-90, hebi(:,4)*-1,hebi(:,5)*-1,hebi(:,6)]) 
For_Jack_Alvin_gimball = transpose(angle_for_hebi_rad_array);
For_Jack_Alvin_gimball = [For_Jack_Alvin_gimball(1,:)
                          For_Jack_Alvin_gimball(2,:)
                         -For_Jack_Alvin_gimball(2,:)
                          For_Jack_Alvin_gimball(3,:)
                          For_Jack_Alvin_gimball(4,:)
                          For_Jack_Alvin_gimball(5,:)]
                          

%% Try to calculate rate
% AH = angle_for_hebi_degree_array;
% hebi_actuator_rate=[]; %in degree/s
% time_rate = 1; %second per point
% for i=1:row
%     if i==1
%            hebi_actuator_rate = [hebi_actuator_rate; (AH(i,1)-th1)/time_rate ,(AH(i,1)-th2)/time_rate, (AH(i,3)-th3)/time_rate,(AH(i,4)-th4)/time_rate,(AH(i,5)-th5)/time_rate,(AH(1,6)-th6)/time_rate];
%     elseif i~=1
%            hebi_actuator_rate = [hebi_actuator_rate;(AH(i,1)-AH(i-1,1))/time_rate,(AH(i,2)-AH(i-1,2))/time_rate,(AH(i,3)-AH(i-1,3))/time_rate,(AH(i,4)-AH(i-1,4))/time_rate,(AH(i,5)-AH(i-1,5))/time_rate,(AH(i,5)-AH(i-1,5))/time_rate];
%      end
% end
% 
% vpa(hebi_actuator_rate)
    
                             






















