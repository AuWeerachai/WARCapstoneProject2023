%% plots: tranlational pos

hf1 = figure('Name','HEBI POS');

ax(1) = subplot(311);
h1 = plot(time, ef_cmd_pos(:,1), 'b', HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_pos_save(1:HEBI_cnt,1), 'r--', ...
                                      HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_mea_pos_save(1:HEBI_cnt,1), 'g:', 'Linewidth', Linewidth);
title_str = 'HEBI END EFFECTOR POSITION';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('X [mtr]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'PRE CMD', 'HEBI CMD', 'HEBI MEAS', 'location', 'Northeast');

ax(2) = subplot(312);
plot(time, ef_cmd_pos(:,2), 'b', HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_pos_save(1:HEBI_cnt,2), 'r--', ...
                                 HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_mea_pos_save(1:HEBI_cnt,2), 'g:', 'Linewidth', Linewidth);
grid on
ylabel('Y [mtr]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
plot(time, ef_cmd_pos(:,3), 'b', HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_pos_save(1:HEBI_cnt,3), 'r--', ...
                                 HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_mea_pos_save(1:HEBI_cnt,3), 'g:', 'Linewidth', Linewidth);
grid on
ylabel('Z [mtr]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

%% plots: rotational att

hf4 = figure('Name','HEBI ATT');

ax(1) = subplot(411);
h1 = plot(time, ef_cmd_quat_ItoE(:,1), 'b', HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_quat_ItoE_save(1:HEBI_cnt,1), 'r--', ...
                                            HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_mea_quat_ItoE_save(1:HEBI_cnt,1), 'g:', 'Linewidth', Linewidth);
title_str = 'END EFFECTOR QUATERNION';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('Q1 [nd]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'PRE CMD', 'HEBI CMD', 'HEBI MEAS', 'location', 'Northeast');

ax(2) = subplot(412);
plot(time, ef_cmd_quat_ItoE(:,2), 'b', HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_quat_ItoE_save(1:HEBI_cnt,2), 'r--', ...
                                       HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_mea_quat_ItoE_save(1:HEBI_cnt,2), 'g:', 'Linewidth', Linewidth);
grid on
ylabel('Q2 [nd]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(413);
plot(time, ef_cmd_quat_ItoE(:,3), 'b', HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_quat_ItoE_save(1:HEBI_cnt,3), 'r--', ...
                                       HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_mea_quat_ItoE_save(1:HEBI_cnt,3), 'g:', 'Linewidth', Linewidth);
grid on
ylabel('Q3 [nd]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(4) = subplot(414);
plot(time, ef_cmd_quat_ItoE(:,4), 'b', HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_cmd_quat_ItoE_save(1:HEBI_cnt,4), 'r--', ...
                                       HEBI_exe_time_save(1:HEBI_cnt), HEBI_ef_mea_quat_ItoE_save(1:HEBI_cnt,4), 'g:', 'Linewidth', Linewidth);
grid on
ylabel('Q4 [nd]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x')

hf5 = figure('Name','HEBI RPY');

ax(1) = subplot(311);
h1 = plot(time, r2d*ef_cmd_rpy_angles_rad(:,1), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_cmd_rpy_angles_rad_save(1:HEBI_cnt,1), 'r--', ...
                                                     HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_mea_rpy_angles_rad_save(1:HEBI_cnt,1), 'g:', 'Linewidth', Linewidth);
title_str = 'ROLL, PITCH, YAW ANGLES';
ylabel('ROLL [deg]','FontSize',LabelFontSize);
title(title_str,'FontSize',TitleFontSize)
grid
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'PRE CMD', 'HEBI CMD', 'HEBI MEAS', 'location', 'Northeast');

ax(2) = subplot(312);
plot(time, r2d*ef_cmd_rpy_angles_rad(:,2), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_cmd_rpy_angles_rad_save(1:HEBI_cnt,2), 'r--', ...
                                                HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_mea_rpy_angles_rad_save(1:HEBI_cnt,2), 'g:', 'Linewidth', Linewidth)
grid on
ylabel('PITCH [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
plot(time, r2d*ef_cmd_rpy_angles_rad(:,3), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_cmd_rpy_angles_rad_save(1:HEBI_cnt,3), 'r--', ...
                                                HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_ef_mea_rpy_angles_rad_save(1:HEBI_cnt,3), 'g:', 'Linewidth', Linewidth)
grid on
ylabel('YAW [deg]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');
