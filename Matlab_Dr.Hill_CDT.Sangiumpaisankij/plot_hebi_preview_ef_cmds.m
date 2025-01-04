%% plots: tranlational pos, vel, and acc

hf1 = figure('Name','HEBI X Cmds');

ax(1) = subplot(311);
plot(HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_pos_save(1:HEBI_cnt,1), 'b');
title_str = 'HEBI COMMANDED END EFFECTOR X POSITION, VELOCITY, AND ACCELERATION';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('pos [mtr]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(2) = subplot(312);
plot(HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_vel_save(1:HEBI_cnt,1), 'b');
grid on
ylabel('rate [mtr/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
plot(HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_acc_save(1:HEBI_cnt,1), 'b');
grid on
ylabel('acc [mtr/s^2]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

hf2 = figure('Name','HEBI Y Cmds');

ax(1) = subplot(311);
plot(HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_pos_save(1:HEBI_cnt,2), 'b');
title_str = 'HEBI COMMANDED END EFFECTOR Y POSITION, VELOCITY, AND ACCELERATION';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('pos [mtr]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(2) = subplot(312);
 plot(HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_vel_save(1:HEBI_cnt,2), 'b');
grid on
ylabel('rate [mtr/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
plot(HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_acc_save(1:HEBI_cnt,2), 'b');
grid on
ylabel('acc [mtr/s^2]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

hf3 = figure('Name','HEBI Z Cmds');

ax(1) = subplot(311);
plot(HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_pos_save(1:HEBI_cnt,3), 'b');
title_str = 'HEBI COMMANDED END EFFECTOR Z POSITION, VELOCITY, AND ACCELERATION';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('pos [mtr]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(2) = subplot(312);
h1 = plot(HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_vel_save(1:HEBI_cnt,3), 'b');
grid on
ylabel('rate [mtr/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
h1 = plot(HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_acc_save(1:HEBI_cnt,3), 'b');
grid on
ylabel('acc [mtr/s^2]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

%% plots: rotational att, ang rate, and ang accl

hf4 = figure('Name','HEBI Quat Cmds');

ax(1) = subplot(411);
plot(HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_quat_ItoE_save(1:HEBI_cnt,1), 'b');
title_str = 'HEBI COMMANDED END EFFECTOR QUATERNION';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('Q1 [nd]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(2) = subplot(412);
plot(HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_quat_ItoE_save(1:HEBI_cnt,2), 'b');
grid on
ylabel('Q2 [nd]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(413);
plot(HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_quat_ItoE_save(1:HEBI_cnt,3), 'b');
grid on
ylabel('Q3 [dn]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(4) = subplot(414);
plot(HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_quat_ItoE_save(1:HEBI_cnt,4), 'b');
grid on
ylabel('Q4 [nd]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

hf5 = figure('Name','HEBI RPY Cmds');

ax(1) = subplot(311);
plot(HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_cmd_rpy_angles_rad_save(1:HEBI_cnt,1), 'b');
title_str = 'HEBI COMMANDED END EFFECTOR ROLL, PITCH, YAW ANGLES';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('ROLL [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(2) = subplot(312);
plot(HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_cmd_rpy_angles_rad_save(1:HEBI_cnt,2), 'b');
grid on
ylabel('PITCH [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
plot(HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_cmd_rpy_angles_rad_save(1:HEBI_cnt,3), 'b');
grid on
ylabel('YAW [deg]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

hf6 = figure('Name','HEBI Rate Cmds');

ax(1) = subplot(411);
plot(HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_cmd_ang_rate_rps_save(1:HEBI_cnt,1), 'b');
title_str = 'HEBI COMMANDED END EFFECTOR  BODY ANGULAR RATE';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('X [deg/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(2) = subplot(412);
plot(HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_cmd_ang_rate_rps_save(1:HEBI_cnt,2), 'b');
grid on
ylabel('Y [deg/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(3) = subplot(413);
plot(HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_cmd_ang_rate_rps_save(1:HEBI_cnt,3), 'b');
grid on
set(gca, 'Fontsize', AxisFontSize)
ylabel('Z [deg/s]','FontSize',LabelFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(4) = subplot(414);
plot(HEBI_exe_time_save(1:HEBI_cnt), r2d*sqrt(sum(HEBI_ef_cmd_ang_rate_rps_save(1:HEBI_cnt,1:3) .* HEBI_ef_cmd_ang_rate_rps_save(1:HEBI_cnt,1:3), 2)), 'b');
grid on
ylabel('MAG [deg/s]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

hf7 = figure('Name','HEBI Accl Cmds');

ax(1) = subplot(411);
plot(HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_cmd_ang_accl_rps2_save(1:HEBI_cnt,1), 'b');
title_str = 'HEBI COMMANDED END EFFECTOR BODY ANGULAR ACCL';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('X [deg/s^2]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(2) = subplot(412);
plot(HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_cmd_ang_accl_rps2_save(1:HEBI_cnt,2), 'b');
grid on
ylabel('Y [deg/s^2]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(413);
plot(HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_cmd_ang_accl_rps2_save(1:HEBI_cnt,3), 'b');
grid on
ylabel('Z [deg/s^2]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(4) = subplot(414);
plot(HEBI_exe_time_save(1:HEBI_cnt), r2d*sqrt(sum(HEBI_ef_cmd_ang_accl_rps2_save(1:HEBI_cnt,1:3) .* HEBI_ef_cmd_ang_accl_rps2_save(1:HEBI_cnt,1:3), 2)), 'b');
grid on
ylabel('HEBI MAG [deg/s^2]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
