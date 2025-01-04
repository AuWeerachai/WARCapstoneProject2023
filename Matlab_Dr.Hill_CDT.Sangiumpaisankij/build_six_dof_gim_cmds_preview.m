%% preview command profile generation

time_step = 0.05; time_end = CMD_POS_STRUCT.strt_time(end) + CMD_POS_STRUCT.duration(end);

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