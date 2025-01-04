
clear all; close all; clc;

r2d = 180/pi; d2r = pi/180;

addpath('C:/Users/CDT Sangiumpaisankij/OneDrive - West Point/Desktop/Fall2023/2.ME404/Dan_Hill_HEBI_code_for_Sang/hebi')

addpath('C:/Users/CDT Sangiumpaisankij/OneDrive - West Point/Desktop/Fall2023/2.ME404/Dan_Hill_HEBI_code_for_Sang/Quaternion_Utilities')

addpath('C:\Users\CDT Sangiumpaisankij\OneDrive - West Point\Desktop\Fall2023\2.ME404\Dan_Hill_HEBI_code_for_Sang\CmdProfile_Utilities')

% Creating a group by selecting custom names
family = 'Arm';
names = {'m1', 'm2','m3','m4','m5','m6','m7','m8'};
group = HebiLookup.newGroupFromNames(family, names);

%% gain settings

gains = group.getGains(); tmpFbk = group.getNextFeedback();

gains.controlStrategy = 4*ones(1,8);

% Pos Kp original
%gains.positionKp = [30 50 50 15 40 100 100 75];

% Vel Kp original
%gains.velocityKp = [0.05 0.03 0.03 0.05 0.1 0.1 0.1 0.1];

% effort Kp original
%gains.effortKp = [0.25 0.1 0.1 0.25 0.3 0.3 0.3 0.2]

% effort Ki original
%gains.effortKi = zeros(1,8);

% effort Kd original
%gains.effortKd = [0.0010  0.0001 0.0001 0.0010 0.0001 0.0001 0.0001 0.0001];

% effort KFF original
gains.effortFF = ones(1,8);

% test G1 to G6
gains.effortKp = zeros(1,8);
gains.effortKi = zeros(1,8);
gains.effortKd = zeros(1,8);

% test G1 (m8)
gains.positionKp(8) = 80;
gains.positionKi(8) = 10;
gains.positionKd(8) = 10;

gains.velocityKp(8) = 0.05;
gains.velocityKi(8) = 0.00;
gains.velocityKd(8) = 0.00;

% test G2 (m6,m7)
gains.positionKp(6) =  80; gains.positionKp(7) =  80;
gains.positionKi(6) =   5; gains.positionKi(7) =  5;
gains.positionKd(6) =   5; gains.positionKd(7) =  5;

gains.velocityKp(6) = 0.10; gains.velocityKp(7) = 0.10;
gains.velocityKi(6) = 0.00; gains.velocityKi(7) = 0.00;
gains.velocityKd(6) = 0.00; gains.velocityKd(7) = 0.00;

% test G3 (m5)
gains.positionKp(5) = 30;
gains.positionKi(5) =  5;
gains.positionKd(5) =  1;

gains.velocityKp(5) = 0.10;
gains.velocityKi(5) = 0.00;
gains.velocityKd(5) = 0.00;

% test G4 (m3)
gains.positionKp(3) = 40;
gains.positionKi(3) = 10;
gains.positionKd(3) = 10;

gains.velocityKp(3) = 0.03;
gains.velocityKi(3) = 0.00;
gains.velocityKd(3) = 0.00;

% test G5 (m2)
gains.positionKp(2) = 40;
gains.positionKi(2) =  1;
gains.positionKd(2) =  1;

gains.velocityKp(2) = 0.03;
gains.velocityKi(2) = 0.00;
gains.velocityKd(2) = 0.00;

% test G6 (grip-wrist)
gains.positionKp(1) = 50;
gains.positionKi(1) =  1;
gains.positionKd(1) =  1;

gains.velocityKp(1) = 0.05;
gains.velocityKi(1) = 0.00;
gains.velocityKd(1) = 0.00;

group.send('gains', gains);

pause(1)

tic;

cmd = CommandStruct();

q_desired_deg = zeros(1,8); qdot_desired_dps = zeros(1,8);

q_desired_deg(5) = -90;

q_desired_rad = d2r * q_desired_deg;

qdot_desired_rps = d2r * qdot_desired_dps;

cmd.position = q_desired_rad;
cmd.velocity = qdot_desired_rps;

while toc < 600

    %group.send(cmd);

    fbk = group.getNextFeedback;
    q = fbk.position; 
    qdot = fbk.velocity;
    
    qdot_dps = r2d * qdot;
     
    q_deg = r2d * q; 
    qdot_degpersec = r2d * qdot;
    
    q_deg
    qdot_degpersec
    
end
