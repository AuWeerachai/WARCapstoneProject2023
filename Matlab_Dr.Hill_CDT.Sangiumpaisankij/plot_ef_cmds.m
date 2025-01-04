%% plots: tranlational pos, vel, and acc

hf1 = figure('Name','X Cmds');

ax(1) = subplot(311);
plot(time, ef_cmd_pos(:,1), 'b');
title_str = 'COMMANDED END EFFECTOR X POSITION, VELOCITY, AND ACCELERATION';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('pos [mtr]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(2) = subplot(312);
h1 = plot(time, ef_cmd_vel(:,1), 'b', time(1:end-1), diff(ef_cmd_pos(:,1)/time_step), 'r--');
grid on
ylabel('rate [mtr/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(3) = subplot(313);
h1 = plot(time, ef_cmd_acc(:,1), 'b', time(1:end-1), diff(ef_cmd_vel(:,1)/time_step), 'r--');
grid on
ylabel('acc [mtr/s^2]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast')

linkaxes(ax,'x');

hf2 = figure('Name','Y Cmds');

ax(1) = subplot(311);
plot(time, ef_cmd_pos(:,2), 'b');
title_str = 'COMMANDED END EFFECTOR Y POSITION, VELOCITY, AND ACCELERATION';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('pos [mtr]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(2) = subplot(312);
h1 = plot(time, ef_cmd_vel(:,2), 'b', time(1:end-1), diff(ef_cmd_pos(:,2)/time_step), 'r--');
grid on
ylabel('rate [mtr/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(3) = subplot(313);
h1 = plot(time, ef_cmd_acc(:,2), 'b', time(1:end-1), diff(ef_cmd_vel(:,2)/time_step), 'r--');
grid on
ylabel('acc [mtr/s^2]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast')

linkaxes(ax,'x');

hf3 = figure('Name','Z Cmds');

ax(1) = subplot(311);
plot(time, ef_cmd_pos(:,3), 'b');
title_str = 'COMMANDED END EFFECTOR Z POSITION, VELOCITY, AND ACCELERATION';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('pos [mtr]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(2) = subplot(312);
h1 = plot(time, ef_cmd_vel(:,3), 'b', time(1:end-1), diff(ef_cmd_pos(:,3)/time_step), 'r--');
grid on
ylabel('rate [mtr/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(3) = subplot(313);
h1 = plot(time, ef_cmd_acc(:,3), 'b', time(1:end-1), diff(ef_cmd_vel(:,3)/time_step), 'r--');
grid on
ylabel('acc [mtr/s^2]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast')

linkaxes(ax,'x');

%% plots: rotational att, ang rate, and ang accl

hf4 = figure('Name','Quat Cmds');

ax(1) = subplot(411);
plot(time, ef_cmd_quat_ItoE(:,1), 'b');
title_str = 'COMMANDED END EFFECTOR QUATERNION';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('Q1 [nd]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(2) = subplot(412);
plot(time, ef_cmd_quat_ItoE(:,2), 'b');
grid on
ylabel('Q2 [nd]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(413);
plot(time, ef_cmd_quat_ItoE(:,3), 'b');
grid on
ylabel('Q3 [dn]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(4) = subplot(414);
plot(time, ef_cmd_quat_ItoE(:,4), 'b');
grid on
ylabel('Q4 [nd]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

hf5 = figure('Name','RPY Cmds');

ax(1) = subplot(311);
plot(time, r2d*ef_cmd_rpy_angles_rad(:,1), 'b');
title_str = 'COMMANDED END EFFECTOR ROLL, PITCH, YAW ANGLES';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('ROLL [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(2) = subplot(312);
plot(time, r2d*ef_cmd_rpy_angles_rad(:,2), 'b');
grid on
ylabel('PITCH [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
plot(time, r2d*ef_cmd_rpy_angles_rad(:,3), 'b');
grid on
ylabel('YAW [deg]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

hf6 = figure('Name','Rate Cmds');

ax(1) = subplot(411);
h1 = plot(time, r2d*ef_cmd_ang_rate_rps(:,1), 'b', time(1:end-1), r2d*ef_cmd_ang_rate_rps_bkdiff(:,1), 'r--' );
title_str = 'COMMANDED END EFFECTOR  BODY ANGULAR RATE';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('X [deg/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(2) = subplot(412);
h1 = plot(time, r2d*ef_cmd_ang_rate_rps(:,2), 'b', time(1:end-1), r2d*ef_cmd_ang_rate_rps_bkdiff(:,2), 'r--' );
grid on
ylabel('Y [deg/s]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(3) = subplot(413);
h1 = plot(time, r2d*ef_cmd_ang_rate_rps(:,3), 'b', time(1:end-1), r2d*ef_cmd_ang_rate_rps_bkdiff(:,3), 'r--' );
grid on
set(gca, 'Fontsize', AxisFontSize)
ylabel('Z [deg/s]','FontSize',LabelFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(4) = subplot(414);
h1 = plot(time, r2d*sqrt(sum(ef_cmd_ang_rate_rps(:,1:3) .* ef_cmd_ang_rate_rps(:,1:3), 2)), 'b', time(1:end-1), r2d*sqrt(sum(ef_cmd_ang_rate_rps_bkdiff(:,1:3) .* ef_cmd_ang_rate_rps_bkdiff(:,1:3), 2)), 'r--');
grid on
ylabel('MAG [deg/s]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast')

linkaxes(ax,'x');

hf7 = figure('Name','Accl Cmds');

ax(1) = subplot(411);
h1 = plot(time, r2d*ef_cmd_ang_accl_rps2(:,1), 'b', time(1:end-1), r2d*diff(ef_cmd_ang_rate_rps(:,1)/time_step), 'r--');
title_str = 'COMMANDED END EFFECTOR BODY ANGULAR ACCL';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('X [deg/s^2]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(2) = subplot(412);
h1 = plot(time, r2d*ef_cmd_ang_accl_rps2(:,2), 'b', time(1:end-1), r2d*diff(ef_cmd_ang_rate_rps(:,2)/time_step), 'r--');
grid on
ylabel('Y [deg/s^2]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(3) = subplot(413);
h1 = plot(time, r2d*ef_cmd_ang_accl_rps2(:,3), 'b', time(1:end-1), r2d*diff(ef_cmd_ang_rate_rps(:,3)/time_step), 'r--');
grid on
ylabel('Z [deg/s^2]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast');

ax(4) = subplot(414);
h1 = plot(time, r2d*sqrt(sum(ef_cmd_ang_accl_rps2(:,1:3) .* ef_cmd_ang_accl_rps2(:,1:3), 2)), 'b', time(1:end-1), r2d*sqrt(sum(diff(ef_cmd_ang_rate_rps(:,1:3))/time_step .* diff(ef_cmd_ang_rate_rps(:,1:3)/time_step), 2)) , 'r--');
grid on
ylabel('MAG [deg/s^2]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'BACKDIFF', 'location', 'Northeast')
