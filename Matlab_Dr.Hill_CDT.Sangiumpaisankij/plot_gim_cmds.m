%% plots: commanded gimbal angles

hfa1to3 = figure('Name','Gim Angs123');

ax(1) = subplot(311);
plot(time, r2d*cmd_gim_angles_out_rad(:,1), 'b');
title_str = 'COMMANDED GIMBAL ANGLES 1 to 3';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('Gim 1 [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(2) = subplot(312);
plot(time, r2d*cmd_gim_angles_out_rad(:,2), 'b');
grid on
ylabel('Gim 2 [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
plot(time,  r2d*cmd_gim_angles_out_rad(:,3), 'b');
grid on
ylabel('Gim 3 [deg]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

hfa4to6 = figure('Name','Gim Angs456');

ax(1) = subplot(311);
plot(time, r2d*cmd_gim_angles_out_rad(:,4), 'b');
title_str = 'COMMANDED GIMBAL ANGLES 4 to 6';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('Gim 4 [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(2) = subplot(312);
plot(time, r2d*cmd_gim_angles_out_rad(:,5), 'b');
grid on
ylabel('Gim 5 [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
plot(time,  r2d*cmd_gim_angles_out_rad(:,6), 'b');
grid on
ylabel('Gim 6 [deg]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

%% plots: commanded gimbal rates

hfr1to3 = figure('Name','Gim Rates123');

ax(1) = subplot(311);
h1 = plot(time, r2d*cmd_gim_rates_out_rps(:,1), 'b', time(1:end-1), r2d*diff(cmd_gim_angles_out_rad(:,1))/time_step, 'r--');
title_str = 'COMMANDED GIMBAL RATES 1 to 3';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('Gim 1 [deg/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(2) = subplot(312);
h1 = plot(time, r2d*cmd_gim_rates_out_rps(:,2), 'b', time(1:end-1), r2d*diff(cmd_gim_angles_out_rad(:,2))/time_step, 'r--');
grid on
ylabel('Gim 2 [deg/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(3) = subplot(313);
h1 = plot(time,  r2d*cmd_gim_rates_out_rps(:,3), 'b', time(1:end-1), r2d*diff(cmd_gim_angles_out_rad(:,3))/time_step, 'r--');
grid on
ylabel('Gim 3 [deg/s]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

linkaxes(ax,'x');

hfr4to6 = figure('Name','Gim Rates456');

ax(1) = subplot(311);
h1 = plot(time, r2d*cmd_gim_rates_out_rps(:,4), 'b', time(1:end-1), r2d*diff(cmd_gim_angles_out_rad(:,4))/time_step, 'r--');
title_str = 'COMMANDED GIMBAL RATES 4 to 6';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('Gim 4 [deg/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(2) = subplot(312);
h1 = plot(time, r2d*cmd_gim_rates_out_rps(:,5), 'b', time(1:end-1), r2d*diff(cmd_gim_angles_out_rad(:,5))/time_step, 'r--');
grid on
ylabel('Gim 5 [deg/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(3) = subplot(313);
h1 = plot(time,  r2d*cmd_gim_rates_out_rps(:,6), 'b', time(1:end-1), r2d*diff(cmd_gim_angles_out_rad(:,6))/time_step, 'r--');
grid on
ylabel('Gim 6 [deg/s]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast')

linkaxes(ax,'x');
