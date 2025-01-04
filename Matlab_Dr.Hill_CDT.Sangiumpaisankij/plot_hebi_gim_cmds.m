%% preview and HEBI gimbal angle commands

hf1to3 = figure('Name','Gim Angs123');

ax(1) = subplot(311);
h1 = plot(time, r2d*cmd_gim_angles_out_rad(:,1), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_cmd_gim_angles_out_rad(1:HEBI_cnt,1), 'r--', ...
                                                      HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_mea_gim_angles_out_rad(1:HEBI_cnt,1), 'g:', 'Linewidth', Linewidth);
title_str = 'GIMBAL ANGLES 1 to 3';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('Gim 1 [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'PREVIEW', 'HEBI CMD', 'HEBI MEAS', 'location', 'Northeast');

ax(2) = subplot(312);
plot(time, r2d*cmd_gim_angles_out_rad(:,2), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_cmd_gim_angles_out_rad(1:HEBI_cnt,2), 'r--', ...
                                                 HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_mea_gim_angles_out_rad(1:HEBI_cnt,2), 'g:', 'Linewidth', Linewidth);
grid on
ylabel('Gim 2 [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
plot(time,  r2d*cmd_gim_angles_out_rad(:,3), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_cmd_gim_angles_out_rad(1:HEBI_cnt,3), 'r--', ...
                                                  HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_mea_gim_angles_out_rad(1:HEBI_cnt,3), 'g:', 'Linewidth', Linewidth);
grid on
ylabel('Gim 3 [deg]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

hf4to6 = figure('Name','Gim Angs456');

ax(1) = subplot(311);
h1 = plot(time, r2d*cmd_gim_angles_out_rad(:,4), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_cmd_gim_angles_out_rad(1:HEBI_cnt,4), 'r--', ...
                                                      HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_mea_gim_angles_out_rad(1:HEBI_cnt,4), 'g:', 'Linewidth', Linewidth);
title_str = 'GIMBAL ANGLES 4 to 6';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('Gim 4 [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'PREVIEW', 'HEBI CMD', 'HEBI MEAS', 'location', 'Northeast');

ax(2) = subplot(312);
plot(time, r2d*cmd_gim_angles_out_rad(:,5), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_cmd_gim_angles_out_rad(1:HEBI_cnt,5), 'r--', ...
                                                 HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_mea_gim_angles_out_rad(1:HEBI_cnt,5), 'g:', 'Linewidth', Linewidth);
grid on
ylabel('Gim 5 [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
plot(time, r2d*cmd_gim_angles_out_rad(:,6), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_cmd_gim_angles_out_rad(1:HEBI_cnt,6), 'r--', ...
                                                 HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_mea_gim_angles_out_rad(1:HEBI_cnt,6), 'g:', 'Linewidth', Linewidth);
grid on
ylabel('Gim 6 [deg]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

%%  preview and HEBI gimbal rate commands

hfr1to3 = figure('Name','Gim Rates123');

ax(1) = subplot(311);
h1 = plot(time, r2d*cmd_gim_rates_out_rps(:,1), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_cmd_gim_rates_out_rps(1:HEBI_cnt,1), 'r--', ...
                                                     HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_mea_gim_rates_out_rps(1:HEBI_cnt,1), 'g:', 'Linewidth', Linewidth);
title_str = 'GIMBAL RATES 1 to 3';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('Gim 1 [deg/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'PRE CMD', 'HEBI CMD', 'HEBI MEAS', 'location', 'Northeast');

ax(2) = subplot(312);
h1 = plot(time, r2d*cmd_gim_rates_out_rps(:,2), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_cmd_gim_rates_out_rps(1:HEBI_cnt,2), 'r--', ...
                                                     HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_mea_gim_rates_out_rps(1:HEBI_cnt,2), 'g:', 'Linewidth', Linewidth);
grid on
ylabel('Gim 2 [deg/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
h1 = plot(time, r2d*cmd_gim_rates_out_rps(:,3), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_cmd_gim_rates_out_rps(1:HEBI_cnt,3), 'r--', ...
                                                     HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_mea_gim_rates_out_rps(1:HEBI_cnt,3), 'g:', 'Linewidth', Linewidth);
grid on
ylabel('Gim 3 [deg/s]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

hfr4to6 = figure('Name','Gim Rates456');

ax(1) = subplot(311);
h1 = plot(time, r2d*cmd_gim_rates_out_rps(:,4), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_cmd_gim_rates_out_rps(1:HEBI_cnt,4), 'r--', ...
                                                     HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_mea_gim_rates_out_rps(1:HEBI_cnt,4), 'g:', 'Linewidth', Linewidth);
title_str = 'GIMBAL RATES 4 to 6';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('Gim 4 [deg/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'PRE CMD', 'HEBI CMD', 'HEBI MEAS', 'location', 'Northeast');

ax(2) = subplot(312);
plot(time, r2d*cmd_gim_rates_out_rps(:,5), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_cmd_gim_rates_out_rps(1:HEBI_cnt,5), 'r--', ...
                                                HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_mea_gim_rates_out_rps(1:HEBI_cnt,5), 'g:', 'Linewidth', Linewidth);
grid on
ylabel('Gim 5 [deg/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
plot(time, r2d*cmd_gim_rates_out_rps(:,6), 'b', HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_cmd_gim_rates_out_rps(1:HEBI_cnt,6), 'r--', ...
                                                HEBI_exe_time_save(1:HEBI_cnt), r2d*HEBI_mea_gim_rates_out_rps(1:HEBI_cnt,6), 'g:', 'Linewidth', Linewidth);
grid on
ylabel('Gim 6 [deg/s]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');
