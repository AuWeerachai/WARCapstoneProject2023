
clear all
close all
restoredefaultpath
clc
%
% Functions: 
%
% 1: compute_fwd_map_ver_2
% 2: compute_ef_pos_cmd_profile
% 3: compute_ef_atitude_cmd_profile
% 4: compute_ef_cmds_ver2
% 5: compute_inverse_kinematics_ver2
% 6: various Quaternion_Utilities folder functions
%
% Plot scripts:
% 1: plot_gim_cmds
% 2: plot_ef_cmds
% 3: plot_ef_cmds_err

% dock figures
set(0,'DefaultFigureWindowStyle','docked')

% plot font sizes
AxisFontSize   = 16;
LabelFontSize  = 16;
LegendFontSize = 16;
TitleFontSize  = 16;
Linewidth      = 2;

% paths to quaternion and profile utilities')
addpath('/home/usmarobot/ay23_capstone-15-war/Dan_Hill_HEBI_code_for_Sang/hebi')
addpath('/home/usmarobot/ay23_capstone-15-war/Dan_Hill_HEBI_code_for_Sang/Quaternion_Utilities')
addpath('/home/usmarobot/ay23_capstone-15-war/Dan_Hill_HEBI_code_for_Sang/CmdProfile_Utilities')
addpath('/home/usmarobot/ay23_capstone-15-war/Dan_Hill_HEBI_code_for_Sang')
hebi_load();
% reset random number generator to its initial settings
%stream = RandStream.getGlobalStream;
%reset(stream);

% Newton-Raphson inverse kinematics algorithm parameters
converge_tol = 1e-8; n_iter_max = 20;

% conversion constants
r2d = 180/pi; d2r = pi/180; inch2mtr = 0.0254;

% approximate link lengths (meters)
%L_inch = [4.25; 0; 5.5; 13.0; 13.0; 3.0; 2.50; 3.25; 1.5; 7.50]; L_mtr = inch2mtr * L_inch;
L_inch = [4.25; 0; 5.5; 13.0; 13.0; 3.0; 2.50; 3.25; 1.5; 4.25]; 
L_mtr = inch2mtr * L_inch;

%% initial gimbal angles and ef position and attitude

%Ask if we will use manual or random inititation
while 1
  init_gim_angles_flag = input('\n DO YOU WANT MANUAL OR RANDOM INITIAL GIMBAL ANGLES (0=RANDOM, 1=MANUAL)>>');
  
  if init_gim_angles_flag == 0 || init_gim_angles_flag == 1
     if init_gim_angles_flag == 0
        fprintf('\n USING RANDOM INITIAL GIMBAL ANGLES \n');
     else
        fprintf('\n USING MANUAL INITIAL GIMBAL ANGLES \n'); 
     end
     break;
  else
    fprintf('\n ANSWER MUST BE 0 OR 1. TRY AGAIN. \n');
  end
end

%create a place holder for initial gimbal angle
init_gim_angles_deg = zeros(6,1);

%receive gimbal angle from user or random it up
if init_gim_angles_flag  
    fprintf('\n DO NOT MAKE ALL INITIAL GIMBAL ANGLES CLOSE TO ZERO (SINGULAR CONFIGURATION) \n');
    for i=1:6
        str = ['\n ENTER INITIAL GIMBAL ANGLE ', num2str(i), ' (DEG)>>'];
        init_gim_angles_deg(i) = input(str);
    end
else
    init_gim_angles_deg = 180 * (-1 + 2*rand(6,1));
    fprintf('\n RANDOM INITIAL GIMBAL ANGLES BETWEEN -180 AND +180 DEG \n');
    for i=1:6
      fprintf('\n %d = %+0.2f', i, init_gim_angles_deg(i)); 
    end
    fprintf('\n')
end

%change gimbal angle to radian for calculation
init_gim_angles_rad = d2r * init_gim_angles_deg;

%create a reference frame map like in MC306
init_Amap = compute_fwd_map_ver2(init_gim_angles_rad, L_mtr);

%% position command profile parameters

CMD_POS_STRUCT.strt_time = 30; % single dwell start time
CMD_POS_STRUCT.duration  = 10; % single dwell duration

%CMD_POS_STRUCT.strt_time = [30 70]; % dwell start time
%CMD_POS_STRUCT.duration  = [10 10]; % dwell duration

%CMD_POS_STRUCT.strt_time = [30 70 110]; % dwell start time
%CMD_POS_STRUCT.duration  = [10 10  10]; % dwell duration


%since this design of kinematic is time base. We use time to let actuator
%determine rate of moving by time
% CMD_POS_STRUCT.strt_time = [20 50 80 110]; % dwell start time
% CMD_POS_STRUCT.duration  = [10 10  10  10]; % dwell duration

%check for error in inputting time
if length(CMD_POS_STRUCT.strt_time) == length(CMD_POS_STRUCT.duration)
   n_tg_time = length(CMD_POS_STRUCT.strt_time);
else
   fprintf('\n NUMBER OF TARGET POSITION DWELL START TIMES IS NOT CONSISTENT WITH NUMBER OF DWELL DURATIONS: EXITING \n');
   return
end

% Set up by Dr.Hill
% minimum slew duration
CMD_POS_STRUCT.min_slew_dur_sec = 0.1;




%start to receive position
% initial ef position in I frame
CMD_POS_STRUCT.Init_x_pos = init_Amap(1,4);
CMD_POS_STRUCT.Init_y_pos = init_Amap(2,4);
CMD_POS_STRUCT.Init_z_pos = init_Amap(3,4);

fprintf('\n INITIAL COMMAND POSITION IN BASE FRAME (METERS) \n');
fprintf('\n X POS = %+0.3f', CMD_POS_STRUCT.Init_x_pos);
fprintf('\n Y POS = %+0.3f', CMD_POS_STRUCT.Init_y_pos);
fprintf('\n Z POS = %+0.3f \n', CMD_POS_STRUCT.Init_z_pos);



% dwell ef position
CMD_POS_STRUCT.x_pos = +0.5;
CMD_POS_STRUCT.y_pos = -0.493;
CMD_POS_STRUCT.z_pos = +0.30;

%CMD_POS_STRUCT.x_pos = [+0.45 +0.45];
%CMD_POS_STRUCT.y_pos = [+0.45 -0.45];
%CMD_POS_STRUCT.z_pos = [+0.30 +0.30];

%CMD_POS_STRUCT.x_pos = [+0.45 +0.45 +0.45];
%CMD_POS_STRUCT.y_pos = [+0.45 -0.45 +0.45];
%CMD_POS_STRUCT.z_pos = [+0.30 +0.30 +0.30];


%Desired point
% CMD_POS_STRUCT.x_pos = [+0.45 +0.45 +0.45 +0.45];
% CMD_POS_STRUCT.y_pos = [+0.45 -0.45 +0.45  0.00];
% CMD_POS_STRUCT.z_pos = [+0.30 +0.30 +0.30 +0.40];

n_tg_x_pos = length(CMD_POS_STRUCT.x_pos);
n_tg_y_pos = length(CMD_POS_STRUCT.y_pos);
n_tg_z_pos = length(CMD_POS_STRUCT.z_pos);


%%checking
if ~(n_tg_x_pos == n_tg_y_pos && n_tg_x_pos == n_tg_z_pos && n_tg_time == n_tg_x_pos)
   fprintf('\n NUMBER OF TARGET DWELL POSITIONS IS NOT CONSISTENT WITH NUMBER OF DWELL START TIMES AND DURATIONS: EXITING \n');
   return
end

%Profile jerk contribution
while 1
  CMD_POS_STRUCT.tJ_pct = input('\n ENTER CMD POS PROFILE JERK CONTRIBUTION (10 to 100 PERCENT, TRY 60 to 100)>>'); 
  if CMD_POS_STRUCT.tJ_pct < 10 || CMD_POS_STRUCT.tJ_pct > 100
     fprintf('\n CMD POS PROFILE JERK MUST BE BETWEEN 10 AND 100. TRY AGAIN. \n');
  else
     break;
  end
  
end
%%%%

%xyz_accl_limit_mps2 = input('\n ENTER INITIAL XYZ ACCEL LIMIT (METER/SEC^2)>>');
xyz_accl_limit_mps2 = 2;

CMD_POS_STRUCT.xyz_accl_limit_mps2 = xyz_accl_limit_mps2;

%% attiude command profile parameters

CMD_ATT_STRUCT.strt_time = CMD_POS_STRUCT.strt_time;
CMD_ATT_STRUCT.duration  = CMD_POS_STRUCT.duration;

% minimum slew duration
CMD_ATT_STRUCT.min_slew_dur_sec = 0.1;

% initial ef attitude in I frame
%
% Amap:  dcm map from EF(E) frame to global (I) frame
% 
% Amap': dcm map from global (I) frame to EF(E) frame
%
CMD_ATT_STRUCT.Initq_ItoB = util_DC_to_quat(transpose(init_Amap(1:3,1:3)));

init_rol_pit_yaw_angles_rad = util_quat_to_body_321(CMD_ATT_STRUCT.Initq_ItoB);

init_rol_pit_yaw_angles_deg = r2d * init_rol_pit_yaw_angles_rad;

fprintf('\n INITIAL EF ROL ANGLE REL I FRAME = %+0.2f DEG \n', init_rol_pit_yaw_angles_deg(1));
fprintf('\n INITIAL EF PIT ANGLE REL I FRAME = %+0.2f DEG \n', init_rol_pit_yaw_angles_deg(2));
fprintf('\n INITIAL EF YAW ANGLE REL I FRAME = %+0.2f DEG \n', init_rol_pit_yaw_angles_deg(3));
%
% dwell ef attitude (E relative to I)
%
% body 321 rotation (3-yaw, 2-pitch, 1-roll)
%
% -180 <= yaw_deg <= 180, -90 < pit_deg < 90, -180 <= rol_deg <= 180
%
yaw_deg = zeros(1,n_tg_time);
pit_deg = zeros(1,n_tg_time);
rol_deg = zeros(1,n_tg_time);

n_tg_yaw = n_tg_time; 
n_tg_pit = n_tg_time; 
n_tg_rol = n_tg_time;

if ~(n_tg_yaw == n_tg_pit && n_tg_yaw == n_tg_rol && n_tg_yaw == n_tg_time)
   fprintf('\n NUMBER OF TARGET DWELL ATTITUDES IS NOT CONSISTENT WITH NUMBER OF DWELL START TIMES AND DURATIONS: EXITING \n');
   return
end

for i=1:n_tg_time

  yaw_deg(i) = atan2d(CMD_POS_STRUCT.y_pos(i), CMD_POS_STRUCT.x_pos(i));

  pit_deg(i) = +89.99;

  rol_deg(i) = 0;
    
  rpy_angles_rad = d2r * [rol_deg(i); pit_deg(i); yaw_deg(i)];

  CMD_ATT_STRUCT.q_ItoB(i,:) = util_body_321_to_quat(rpy_angles_rad)';

end

while 1

  CMD_ATT_STRUCT.tJ_pct = input('\n ENTER CMD ATT PROFILE JERK CONTRIBUTION (10 to 100 PERCENT, TRY 60 to 100)>>');
  
  if CMD_ATT_STRUCT.tJ_pct < 10 || CMD_ATT_STRUCT.tJ_pct > 100
     fprintf('\n CMD ATT PROFILE JERK MUST BE BETWEEN 10 AND 100. TRY AGAIN. \n');
  else
     break;
  end
  
end

%accl_limit_dps2 = input('\n ENTER INITIAL ANGULAR ACCEL LIMIT (DEG/SEC^2)>>');
accl_limit_dps2 = 2;

accl_limit_rps2 = d2r * accl_limit_dps2;

CMD_ATT_STRUCT.accl_limit_rps2 = accl_limit_rps2;

% number of pos and att command targets must be the same
CMD_POS_STRUCT.n_tg = n_tg_time;
CMD_ATT_STRUCT.n_tg = CMD_POS_STRUCT.n_tg;

%%  command profile generation

time_step = 0.02; time_end = CMD_POS_STRUCT.strt_time(end) + CMD_POS_STRUCT.duration(end) + 20;

time = (0:time_step:time_end)'; npts = length(time);

% reserve storage for plots
ef_cmd_pos = zeros(npts,3); 
ef_cmd_vel = zeros(npts,3); 
ef_cmd_acc = zeros(npts,3);

ef_cmd_quat_ItoE      = zeros(npts,4);
ef_cmd_rpy_angles_rad = zeros(npts,3);
ef_cmd_ang_rate_rps   = zeros(npts,3); 
ef_cmd_ang_accl_rps2  = zeros(npts,3);

ef_cmd_ang_rate_rps_bkdiff = zeros(npts-1,3);

ef_cmds_out = zeros(npts,24);

for i=1:npts

  [err_flag_pos, ef_cmds] = compute_ef_pos_cmd_profile(time(i), CMD_POS_STRUCT);

  ef_cmd_pos(i,:) = ef_cmds(1:3);
  ef_cmd_vel(i,:) = ef_cmds(4:6);
  ef_cmd_acc(i,:) = ef_cmds(7:9);

  [err_flag_att, ef_cmds] = compute_ef_atitude_cmd_profile(time(i), CMD_ATT_STRUCT);

  ef_cmd_quat_ItoE(i,:)     = ef_cmds(1:4);
  ef_cmd_ang_rate_rps(i,:)  = ef_cmds(5:7);
  ef_cmd_ang_accl_rps2(i,:) = ef_cmds(8:10);

  % map quaternions to body 321 roll, pitch, yaw angles
  ef_cmd_rpy_angles_rad(i,:) = util_quat_to_body_321(ef_cmd_quat_ItoE(i,:)');

  % back difference body rate check
  if i > 1

    q_ItoB = ef_cmd_quat_ItoE(i,:)';

    G = [ q_ItoB(4)  q_ItoB(3) -q_ItoB(2) -q_ItoB(1)
         -q_ItoB(3)  q_ItoB(4)  q_ItoB(1) -q_ItoB(2)
          q_ItoB(2) -q_ItoB(1)  q_ItoB(4) -q_ItoB(3)];

    ef_cmd_ang_rate_rps_bkdiff(i-1,:) = ( 2 * G * (ef_cmd_quat_ItoE(i,:) - ef_cmd_quat_ItoE(i-1,:))' / time_step )';

  end

  ef_cmds_out(i,:) = compute_ef_cmds_ver2(ef_cmd_pos(i,:), ef_cmd_vel(i,:), ef_cmd_quat_ItoE(i,:), ef_cmd_ang_rate_rps(i,:))';

end

%% command gimbal angles

%
% direction cosine matrix from end effector frame (E) to base frame (I)
%
% C_EtoI = [nx ox ax
%           ny oy ay
%           nz oz az]
%
% 4x4 map from effector frame to base frame
%
% A = [nx ox ax px
%      ny oy ay py
%      nz oz az pz
%      0  0  0  1]
%
% unit vector definition
%  [nx; ny; nz] = approach vector  in I frame
%  [ox; oy; oz] = fingertip vector in I frame
%  [ax; ay; az] = cross([nx; ny; nz], [ox; oy; oz]) in I frame
%
% d/dt [C_EtoI] = C_EtoI * skew(W)
%
% skew(W) = [ 0 -Wz Wy
%            Wz  0 -Wx
%           -Wy  Wx 0]
%
%  W = [Wx; Wy; Wz] angular rate of E frame rel to I frame in E frame axes
%

% reserve storage
cmd_gim_angles_out_rad = zeros(npts,6); cmd_gim_rates_out_rps = zeros(npts,6);

ef_cmd_pos_chk = zeros(npts,3); ef_cmd_att_err = zeros(npts,3); ef_cmd_quat_ItoE_chk = zeros(npts,4); cond_of_J = zeros(npts,1);

% initialize nominal gimbal angles based on sensor readings
nom_gim_angles_rad = init_gim_angles_rad;

for i=1:npts

  % extract ef attitude and position commands
  Nvec_pert = ef_cmds_out(i,1:3)'; Ovec_pert = ef_cmds_out(i,4:6)'; Avec_pert = ef_cmds_out(i,7:9)'; Pvec_pert = ef_cmds_out(i,10:12)';
  
  % extract ef rate and velocity commands
  Ndotvec_pert = ef_cmds_out(i,13:15)'; Odotvec_pert = ef_cmds_out(i,16:18)'; Adotvec_pert = ef_cmds_out(i,19:21)'; Pdotvec_pert = ef_cmds_out(i,22:24)';
  
  % inverse kinematics for gimbal angle and rate commands
  [cmd_gim_angles_rad, cmd_gim_rates_rps, n_iter, cond_of_J(i)] = compute_inverse_kinematics_ver2(converge_tol, n_iter_max, nom_gim_angles_rad, L_mtr, ...
                                                                  Nvec_pert, Ovec_pert, Avec_pert, Pvec_pert, Ndotvec_pert, Odotvec_pert, Adotvec_pert, Pdotvec_pert); 
  % update nominal gimbal angles
  nom_gim_angles_rad = cmd_gim_angles_rad;

  % save commanded gimbal angles and rates for plots
  cmd_gim_angles_out_rad(i,:) = cmd_gim_angles_rad'; cmd_gim_rates_out_rps(i,:) = cmd_gim_rates_rps';

  % ef pos and attitude check
  Amap_tot_chk = compute_fwd_map_ver2(cmd_gim_angles_rad, L_mtr);

  ef_cmd_pos_chk(i,:) = Amap_tot_chk(1:3,4)';
  
  C_EtoI_chk = Amap_tot_chk(1:3,1:3); ef_cmd_quat_ItoE_chk(i,:) = util_DC_to_quat(C_EtoI_chk')';

  % remove quaternion flips
  if i > 1
     ef_cmd_quat_ItoE_chk(i,:) = util_quat_flip(ef_cmd_quat_ItoE_chk(i,:)', ef_cmd_quat_ItoE_chk(i-1,:)');
  end

  ef_cmd_quat_err = util_quat_multiply(ef_cmd_quat_ItoE(i,:)', util_quat_conj(ef_cmd_quat_ItoE_chk(i,:)')); ef_cmd_att_err(i,:) = 2*ef_cmd_quat_err(1:3);

end

% unwrap yaw and roll angles
ef_cmd_rpy_angles_rad(:,1) = unwrap(ef_cmd_rpy_angles_rad(:,1));
ef_cmd_rpy_angles_rad(:,3) = unwrap(ef_cmd_rpy_angles_rad(:,3));

%% plots

hfjac = figure('Name','Cond Of J');

plot(time, cond_of_J, 'b');
title_str = 'CONDITION NUMBER OF JACOBIAN';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('Cond [nd]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

plot_gim_cmds

plot_ef_cmds

plot_ef_cmds_err
