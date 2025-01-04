
clear all; close all; clc;

warning('off');

% dock figures
set(0,'DefaultFigureWindowStyle','docked')

% plot font sizes
AxisFontSize   = 16;
LabelFontSize  = 16;
LegendFontSize = 16;
TitleFontSize  = 16;
Linewidth      = 2;

% paths to quaternion and profile utilities
addpath('Quaternion_Utilities', 'CmdProfile_Utilities')
addpath('/home/usmarobot/ay23_capstone-15-war/Dan_Hill_HEBI_code_for_Sang/hebi')
hebi_load();

plot_flag = 1; % 0=no plots, 1=plots

%% HEBI group definition
%  
% Arm   Gimbal 
% m1,   G6
% m2,   G5
% m3,   G4
% m4,   gripper
% m5,   G3
% m6,   G2R
% m7,   G2L
% m8,   G1
%
% load HEBI library


% Creating a group by selecting custom names
family = 'Arm';
names = {'m1', 'm2','m3','m4','m5','m6','m7','m8'};
group = HebiLookup.newGroupFromNames(family, names);

cmd = CommandStruct();

gains = group.getGains(); tmpFbk = group.getNextFeedback();

gains.controlStrategy = 4*ones(1,8);

% Pos Kp original
gains.positionKp = [30 50 50 15 40 100 100 75];

% Vel Kp original
gains.velocityKp = [0.05 0.03 0.03 0.05 0.1 0.1 0.1 0.1];

% effort Kp original
gains.effortKp = [0.25 0.1 0.1 0.25 0.3 0.3 0.3 0.2];

% effort Ki original
gains.effortKi = zeros(1,8);

% effort Kd original
gains.effortKd = [0.0010  0.0001 0.0001 0.0010 0.0001 0.0001 0.0001 0.0001];

if 0

% effort KFF original
gains.effortFF = ones(1,8);

% test G1 to G6
gains.effortKp = zeros(1,8);
gains.effortKi = zeros(1,8);
gains.effortKd = zeros(1,8);

% test G1
gains.positionKp(8) = 80;
gains.positionKi(8) = 10;
gains.positionKd(8) = 10;

gains.velocityKp(8) = 0.05;
gains.velocityKi(8) = 0.00;
gains.velocityKd(8) = 0.00;

% test G2 (m6,m7)
gains.positionKp(6) =  80; gains.positionKp(7) =  80;
gains.positionKi(6) =   5;  gains.positionKi(7) =  5;
gains.positionKd(6) =   5;  gains.positionKd(7) =  5;

gains.velocityKp(6) = 0.10; gains.velocityKp(7) = 0.10;
gains.velocityKi(6) = 0.00; gains.velocityKi(7) = 0.00;
gains.velocityKd(6) = 0.00; gains.velocityKd(7) = 0.00;

% test G3
gains.positionKp(5) = 30;
gains.positionKi(5) =  5;
gains.positionKd(5) =  1;

gains.velocityKp(5) = 0.10;
gains.velocityKi(5) = 0.00;
gains.velocityKd(5) = 0.00;

% test G4
gains.positionKp(3) = 40;
gains.positionKi(3) = 10;
gains.positionKd(3) = 10;

gains.velocityKp(3) = 0.03;
gains.velocityKi(3) = 0.00;
gains.velocityKd(3) = 0.00;

% test G5
gains.positionKp(2) = 40;
gains.positionKi(2) =  1;
gains.positionKd(2) =  1;

gains.velocityKp(2) = 0.03;
gains.velocityKi(2) = 0.00;
gains.velocityKd(2) = 0.00;

% test G6
gains.positionKp(1) = 50;
gains.positionKi(1) =  1;
gains.positionKd(1) =  1;

gains.velocityKp(2) = 0.05;
gains.velocityKi(2) = 0.00;
gains.velocityKd(2) = 0.00;

end

group.send('gains', gains);

pause(1)

%% command generation constants

% Newton-Raphson inverse kinematics algorithm parameters
converge_tol = 1e-8; n_iter_max = 20;

% conversion constants
r2d = 180/pi; d2r = pi/180; inch2mtr = 0.0254;

% approximate link lengths (meters)
L_inch = [4.25; 0; 5.5; 13.0; 13.0; 3.0; 2.50; 3.25; 1.5; 4.25]; L_mtr = inch2mtr * L_inch;

%% end effector (ef) dwell start times and durations

CMD_POS_STRUCT.strt_time = [20]; % dwell start time
CMD_POS_STRUCT.duration  = [10]; % dwell duration

if length(CMD_POS_STRUCT.strt_time) == length(CMD_POS_STRUCT.duration)
   n_tg_time = length(CMD_POS_STRUCT.strt_time);
else
   fprintf('\n NUMBER OF TARGET POSITION DWELL START TIMES IS NOT CONSISTENT WITH NUMBER OF DWELL DURATIONS: EXITING \n');
   return
end

% minimum slew duration
CMD_POS_STRUCT.min_slew_dur_sec = 0.1;

%% initial fwd map using measured HEBI gimbal angles 

init_gim_angles_deg = zeros(6,1);
 
fprintf('\n DO NOT MAKE ALL INITIAL GIMBAL ANGLES CLOSE TO ZERO (SINGULAR CONFIGURATION) \n');
for i=1:6
  str = ['\n ENTER INITIAL GIMBAL ANGLE ', num2str(i), ' (DEG)>>'];
  init_gim_angles_deg(i) = input(str);
end

init_gim_angles_rad = d2r * init_gim_angles_deg;

init_Amap = compute_fwd_map_ver2(init_gim_angles_rad, L_mtr);

%% initialize HEBI command generation

cmd.position = zeros(1,8);
cmd.velocity = zeros(1,8);

fprintf('\n SENDING INITIAL HEBI GIMBAL ANGLE COMMANDS \n')

% initialize nominal gimbal angles based on sensor readings
HEBI_nom_gim_angles_rad = init_gim_angles_rad;

%% position command profile parameters

% initial ef position
CMD_POS_STRUCT.Init_x_pos = init_Amap(1,4);
CMD_POS_STRUCT.Init_y_pos = init_Amap(2,4);
CMD_POS_STRUCT.Init_z_pos = init_Amap(3,4);

fprintf('\n INITIAL EF COMMAND POSITION IN BASE FRAME (METERS) \n');
fprintf('\n X POS = %+0.3f', CMD_POS_STRUCT.Init_x_pos);
fprintf('\n Y POS = %+0.3f', CMD_POS_STRUCT.Init_y_pos);
fprintf('\n Z POS = %+0.3f \n', CMD_POS_STRUCT.Init_z_pos);

% dwell ef position
CMD_POS_STRUCT.x_pos = +0.50;
CMD_POS_STRUCT.y_pos = -0.4935;
CMD_POS_STRUCT.z_pos = +0.20;

% CMD_POS_STRUCT.x_pos = [+0.5 +0.5 +0.5 +0.5];
% CMD_POS_STRUCT.y_pos = [-0.4935 -0.45 -0.40  -0.35];
% CMD_POS_STRUCT.z_pos = [+0.20 +0.20 +0.20 +0.20];

n_tg_x_pos = length(CMD_POS_STRUCT.x_pos);
n_tg_y_pos = length(CMD_POS_STRUCT.y_pos);
n_tg_z_pos = length(CMD_POS_STRUCT.z_pos);

if ~(n_tg_x_pos == n_tg_y_pos && n_tg_x_pos == n_tg_z_pos && n_tg_time == n_tg_x_pos)
   fprintf('\n NUMBER OF TARGET DWELL POSITIONS IS NOT CONSISTENT WITH NUMBER OF DWELL START TIMES AND DURATIONS: EXITING \n');
   return
end

xyz_accl_limit_mps2 = 2; CMD_POS_STRUCT.xyz_accl_limit_mps2 = xyz_accl_limit_mps2;

while 1

  CMD_POS_STRUCT.tJ_pct = input('\n ENTER CMD POS PROFILE JERK CONTRIBUTION (10 to 100 PERCENT, TRY 60 to 100)>>');
  
  if CMD_POS_STRUCT.tJ_pct < 10 || CMD_POS_STRUCT.tJ_pct > 100
     fprintf('\n CMD POS PROFILE JERK MUST BE BETWEEN 10 AND 100. TRY AGAIN. \n');
  else
     break;
  end
  
end

%% attiude command profile parameters

CMD_ATT_STRUCT.strt_time = CMD_POS_STRUCT.strt_time;
CMD_ATT_STRUCT.duration  = CMD_POS_STRUCT.duration;

% minimum slew duration
CMD_ATT_STRUCT.min_slew_dur_sec = 0.1;

% initial ef attitude
%
% Amap:  dcm map from EF(E) frame to global (I) frame
% 
% Amap': dcm map from global (I) frame to EF(E) frame
%
CMD_ATT_STRUCT.Initq_ItoB = util_DC_to_quat(transpose(init_Amap(1:3,1:3)));

init_rol_pit_yaw_angles_rad = util_quat_to_body_321(CMD_ATT_STRUCT.Initq_ItoB);

init_rol_pit_yaw_angles_deg = r2d * init_rol_pit_yaw_angles_rad;

fprintf('\n INITIAL EF YAW ANGLE REL I FRAME = %+0.2f DEG \n', init_rol_pit_yaw_angles_deg(1));
fprintf('\n INITIAL EF PIT ANGLE REL I FRAME = %+0.2f DEG \n', init_rol_pit_yaw_angles_deg(2));
fprintf('\n INITIAL EF ROL ANGLE REL I FRAME = %+0.2f DEG \n', init_rol_pit_yaw_angles_deg(3));
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

n_tg_yaw = n_tg_time; n_tg_pit = n_tg_time; n_tg_rol = n_tg_time;

if ~(n_tg_yaw == n_tg_pit && n_tg_yaw == n_tg_rol && n_tg_yaw == n_tg_time)
   fprintf('\n NUMBER OF TARGET DWELL ATTITUDES IS NOT CONSISTENT WITH NUMBER OF DWELL START TIMES AND DURATIONS: EXITING \n');
   return
end

for i=1:n_tg_time

  yaw_deg(i) = atan2d(CMD_POS_STRUCT.y_pos(i), CMD_POS_STRUCT.x_pos(i));

  %pit_deg(i) = +89.99;
  pit_deg(i) = +82;

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

%% command generation preview

build_six_dof_gim_cmds_preview

fprintf('\n CHECK COMMAND PROFILE PREVIEW PLOTS FOR CONSISTENCY \n')

while 1

  answer = input('\n DO YOU WANT TO EXECUTE THE HEBI COMMAND PROFILE (y or n)>>', 's');

  if strcmp(answer,'y') == 1 || strcmp(answer,'n') 

     if strcmp(answer, 'n')
        HEBI_exe_flag = 0;
        fprintf('\n ABORTING HEBI COMMAND PROFILE \n');
     else
        HEBI_exe_flag = 1;
     end
     break;
  else
    fprintf('\n ANSWER MUST BE y OR n. TRY AGAIN. \n)');
  end

end

if HEBI_exe_flag == 0
   return
else
   close all;
end

fprintf('\n PAUSED: HIT RETURN TO START HEBI PROFILE COMMANDS \n');
pause
fprintf('\n EXECUTING HEBI PROFILE COMMANDS \n');

%%  execute HEBI command generation

% reserve storage
HEBI_ef_cmd_pos = zeros(3,1);
HEBI_ef_cmd_vel = zeros(3,1);
HEBI_ef_cmd_acc = zeros(3,1);

HEBI_ef_cmd_quat_ItoE      = zeros(4,1);
HEBI_ef_cmd_rpy_angles_rad = zeros(3,1);
HEBI_ef_cmd_ang_rate_rps   = zeros(3,1); 
HEBI_ef_cmd_ang_accl_rps2  = zeros(3,1);

HEBI_exe_time_step = time_step; HEBI_exe_time = 0; HEBI_exe_time_end = CMD_POS_STRUCT.strt_time(n_tg_time) + CMD_POS_STRUCT.duration(n_tg_time);

% reserve storage for plots
if plot_flag

   HEBI_exe_time_save = zeros(npts,1);
  
   HEBI_ef_cmd_pos_save            = zeros(npts,3);
   HEBI_ef_cmd_vel_save            = zeros(npts,3);
   HEBI_ef_cmd_acc_save            = zeros(npts,3);
   HEBI_ef_cmd_quat_ItoE_save      = zeros(npts,4);
   HEBI_ef_cmd_rpy_angles_rad_save = zeros(npts,3);
   HEBI_ef_cmd_ang_rate_rps_save   = zeros(npts,3);
   HEBI_ef_cmd_ang_accl_rps2_save  = zeros(npts,3);

   HEBI_cmd_gim_angles_out_rad = zeros(npts,6); 
   HEBI_cmd_gim_rates_out_rps  = zeros(npts,6);

   HEBI_mea_gim_angles_out_rad = zeros(npts,6); 
   HEBI_mea_gim_rates_out_rps  = zeros(npts,6);

end

% start HEBI execution timer and step counter
tic; HEBI_cnt = 0;

cmd.position = zeros(1,8); cmd.velocity = zeros(1,8);

while 1

  toc_time = toc;

  if toc_time >= HEBI_exe_time     
  
      [err_flag_pos, HEBI_ef_cmds] = compute_HEBI_ef_pos_cmd_profile(HEBI_exe_time, CMD_POS_STRUCT);
    
      HEBI_ef_cmd_pos = HEBI_ef_cmds(1:3);
      HEBI_ef_cmd_vel = HEBI_ef_cmds(4:6);
      HEBI_ef_cmd_acc = HEBI_ef_cmds(7:9);
    
      [err_flag_att, HEBI_ef_cmds] = compute_HEBI_ef_atitude_cmd_profile(HEBI_exe_time, CMD_ATT_STRUCT);
    
      HEBI_ef_cmd_quat_ItoE     = HEBI_ef_cmds(1:4);
      HEBI_ef_cmd_ang_rate_rps  = HEBI_ef_cmds(5:7);
      HEBI_ef_cmd_ang_accl_rps2 = HEBI_ef_cmds(8:10);
    
      % map quaternions to body 321 roll, pitch, yaw angles
      HEBI_ef_cmd_rpy_angles_rad = util_quat_to_body_321(HEBI_ef_cmd_quat_ItoE);
    
      HEBI_ef_cmds_out = compute_ef_cmds_ver2(HEBI_ef_cmd_pos, HEBI_ef_cmd_vel, HEBI_ef_cmd_quat_ItoE, HEBI_ef_cmd_ang_rate_rps)';
    
      % extract ef attitude and position commands
      HEBI_Nvec_pert = HEBI_ef_cmds_out(1:3)'; HEBI_Ovec_pert = HEBI_ef_cmds_out(4:6)'; HEBI_Avec_pert = HEBI_ef_cmds_out(7:9)'; HEBI_Pvec_pert = HEBI_ef_cmds_out(10:12)';
      
      % extract ef rate and velocity commands
      HEBI_Ndotvec_pert = HEBI_ef_cmds_out(13:15)'; HEBI_Odotvec_pert = HEBI_ef_cmds_out(16:18)'; HEBI_Adotvec_pert = HEBI_ef_cmds_out(19:21)'; HEBI_Pdotvec_pert = HEBI_ef_cmds_out(22:24)';
      
      % inverse kinematics for gimbal angle and rate commands
      [HEBI_cmd_gim_angles_rad, HEBI_cmd_gim_rates_rps, HEBI_n_iter, HEBI_cond_of_J] = ...
                                                                                       ...
      compute_inverse_kinematics_ver2(converge_tol, n_iter_max, HEBI_nom_gim_angles_rad, L_mtr, ...
                                      HEBI_Nvec_pert, HEBI_Ovec_pert, HEBI_Avec_pert, HEBI_Pvec_pert, HEBI_Ndotvec_pert, HEBI_Odotvec_pert, HEBI_Adotvec_pert, HEBI_Pdotvec_pert); 
      % update nominal gimbal angles
      HEBI_nom_gim_angles_rad = HEBI_cmd_gim_angles_rad;
    
      % increment HEBI_exe_time step counter
      HEBI_cnt = HEBI_cnt + 1;

      cmd.position(1) = +HEBI_cmd_gim_angles_rad(6);    % m1, G6,  +X
      cmd.position(2) = +HEBI_cmd_gim_angles_rad(5);    % m2, G5,  -Y
      cmd.position(3) = +HEBI_cmd_gim_angles_rad(4);    % m3, G4,  -Y
      cmd.position(4) =  0;                             % m4, gripper
      cmd.position(5) =  HEBI_cmd_gim_angles_rad(3) ... % m5, G3,  +X
                       - 0.5*pi; 
      cmd.position(6) = +HEBI_cmd_gim_angles_rad(2);    % m6, G2R, -Y
      cmd.position(7) = -HEBI_cmd_gim_angles_rad(2);    % m7, G2L, +Y
      cmd.position(8) = +HEBI_cmd_gim_angles_rad(1);    % m8, G1,  +Z

      cmd.velocity(1) = +HEBI_cmd_gim_rates_rps(6);     % m1, G6,  +X
      cmd.velocity(2) = +HEBI_cmd_gim_rates_rps(5);     % m2, G5,  -Y
      cmd.velocity(3) = +HEBI_cmd_gim_rates_rps(4);     % m3, G4,  -YHEBI_mea_gim_angles_out_rad
      cmd.velocity(4) =  0;                             % m4, gripper
      cmd.velocity(5) = +HEBI_cmd_gim_rates_rps(3);     % m5, G3,  +X
      cmd.velocity(6) = +HEBI_cmd_gim_rates_rps(2);     % m6, G2R, -Y
      cmd.velocity(7) = -HEBI_cmd_gim_rates_rps(2);     % m7, G2L, +Y
      cmd.velocity(8) = +HEBI_cmd_gim_rates_rps(1);     % m8, G1,  +Z

      group.send(cmd);

      % gimbal angle and rate measurements
      %fbk = group.getNextFeedback(tmpFbk);
      fbk = group.getNextFeedback;
      gim_pos_meas_rad = fbk.position; 
      gim_vel_meas_rps = fbk.velocity;

      % save HEBI data for plots
      if plot_flag
    
         HEBI_exe_time_save(HEBI_cnt) = HEBI_exe_time; 

         HEBI_mea_gim_angles_out_rad(HEBI_cnt,:) = [fbk.position(8) fbk.position(6) fbk.position(5)+0.5*pi fbk.position(3) fbk.position(2) fbk.position(1)];
         HEBI_mea_gim_rates_out_rps(HEBI_cnt,:)  = [fbk.velocity(8) fbk.velocity(6) fbk.velocity(5)        fbk.velocity(3) fbk.velocity(2) fbk.velocity(1)];

         HEBI_cmd_gim_angles_out_rad(HEBI_cnt,:) = HEBI_cmd_gim_angles_rad';
         HEBI_cmd_gim_rates_out_rps(HEBI_cnt,:)  = HEBI_cmd_gim_rates_rps';
        
         HEBI_ef_cmd_pos_save(HEBI_cnt,:) = HEBI_ef_cmd_pos';
         HEBI_ef_cmd_vel_save(HEBI_cnt,:) = HEBI_ef_cmd_vel';
         HEBI_ef_cmd_acc_save(HEBI_cnt,:) = HEBI_ef_cmd_acc';
 
         HEBI_ef_cmd_rpy_angles_rad_save(HEBI_cnt,:) = HEBI_ef_cmd_rpy_angles_rad';
         HEBI_ef_cmd_quat_ItoE_save(HEBI_cnt,:)      = HEBI_ef_cmd_quat_ItoE';
         HEBI_ef_cmd_ang_rate_rps_save(HEBI_cnt,:)   = HEBI_ef_cmd_ang_rate_rps';
         HEBI_ef_cmd_ang_accl_rps2_save(HEBI_cnt,:)  = HEBI_ef_cmd_ang_accl_rps2';

      end

      % increment HEBI_exe_time
      HEBI_exe_time = HEBI_exe_time + HEBI_exe_time_step;

  end  

  if HEBI_exe_time >= HEBI_exe_time_end
     fprintf('\n ALL MANEUVERS COMPLETE \n')
     break;
  end

end

fprintf('\n POST PROCESSING DATA \n')

% HEBI measured end effector attitude and position
HEBI_mea_rpy_angles = zeros(HEBI_cnt,3);

HEBI_ef_mea_pos_save            = zeros(npts,3);
HEBI_ef_mea_vel_save            = zeros(npts,3);
HEBI_ef_mea_acc_save            = zeros(npts,3);
HEBI_ef_mea_quat_ItoE_save      = zeros(npts,4);
HEBI_ef_mea_rpy_angles_rad_save = zeros(npts,3);
HEBI_ef_mea_ang_rate_rps_save   = zeros(npts,3);
HEBI_ef_mea_ang_accl_rps2_save  = zeros(npts,3);

for i=1:HEBI_cnt
 
  Amap_EtoI = compute_fwd_map_ver2(HEBI_mea_gim_angles_out_rad(i,:), L_mtr);

  %Amap_EtoI = compute_fwd_map_ver2(HEBI_cmd_gim_angles_out_rad(i,:), L_mtr); % check

  HEBI_ef_mea_pos_save(i,:) = Amap_EtoI(1:3,4);

  HEBI_ef_mea_quat_ItoE_save(i,:) = util_DC_to_quat(transpose(Amap_EtoI))';

  % remove quaternion flips
  if i > 1
     HEBI_ef_mea_quat_ItoE_save(i,:) = util_quat_flip(HEBI_ef_mea_quat_ItoE_save(i,:)',HEBI_ef_mea_quat_ItoE_save(i-1,:)')';
  end
  
  HEBI_ef_mea_rpy_angles_rad_save(i,:) = util_DC_to_body321(transpose(Amap_EtoI(1:3,1:3)))';

end

% unwrap yaw and roll angles
HEBI_ef_cmd_rpy_angles_rad_save(:,1) = unwrap(HEBI_ef_cmd_rpy_angles_rad_save(:,1));
HEBI_ef_cmd_rpy_angles_rad_save(:,3) = unwrap(HEBI_ef_cmd_rpy_angles_rad_save(:,3));

%% HEBI command plots

if plot_flag
   plot_hebi_gim_cmds
   plot_hebi_ef_cmds
end
