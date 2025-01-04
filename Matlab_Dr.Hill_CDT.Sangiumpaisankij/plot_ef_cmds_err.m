%% ef pos error

hf1 = figure('Name','EF Pos Err');

ax(1) = subplot(311);
plot(time, ef_cmd_pos(:,1), 'b');
title_str = 'COMMANDED END EFFECTOR POSITION AND ERROR';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('X pos [mtr]','FontSize',LabelFontSize)
plot(time, ef_cmd_pos(:,1)-ef_cmd_pos_chk(:,1), 'b');
grid on
ylabel('pos err [mtr]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(1) = subplot(312);
plot(time, ef_cmd_pos(:,2)-ef_cmd_pos_chk(:,2), 'b');
grid on
ylabel('Y pos err [mtr]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
plot(time, ef_cmd_pos(:,3)-ef_cmd_pos_chk(:,3), 'b');
grid on
ylabel('Z pos err [mtr]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

%% ef quaternion and attitude error

hf4 = figure('Name','EF Att Quat');

ax(1) = subplot(411);
h1 = plot(time, ef_cmd_quat_ItoE(:,1), 'b', time, ef_cmd_quat_ItoE_chk(:,1), 'r--');
title_str = 'COMMANDED END EFFECTOR QUATERNION';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('Q1 [nd]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)
set(gca, 'Fontsize', AxisFontSize)
legend(h1, 'CMD', 'CHK', 'location', 'Northeast')

ax(1) = subplot(412);
h1 = plot(time, ef_cmd_quat_ItoE(:,2), 'b', time, ef_cmd_quat_ItoE_chk(:,2), 'r--');
grid on
ylabel('Q2 [nd]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(413);
h1 = plot(time, ef_cmd_quat_ItoE(:,3), 'b', time, ef_cmd_quat_ItoE_chk(:,3), 'r--');
grid on
ylabel('Q3 [nd]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(4) = subplot(414);
h1 = plot(time, ef_cmd_quat_ItoE(:,4), 'b', time, ef_cmd_quat_ItoE_chk(:,4), 'r--');
grid on
ylabel('Q4 [nd]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');

hf5 = figure('Name','EF Att Err');

ax(1) = subplot(311);
plot(time, r2d*ef_cmd_att_err(:,1), 'b');
title_str = 'COMMANDED END EFFECTOR ATTITUDE ERROR';
title(title_str,'FontSize',TitleFontSize)
grid on
ylabel('X att err [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(1) = subplot(312);
plot(time, r2d*ef_cmd_att_err(:,2), 'b');
grid on
ylabel('Y att err [deg]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

ax(3) = subplot(313);
plot(time,  r2d*ef_cmd_att_err(:,3), 'b');
grid on
ylabel('Z att err [deg]','FontSize',LabelFontSize)
xlabel('Time [sec]','FontSize',LabelFontSize)
set(gca, 'Fontsize', AxisFontSize)

linkaxes(ax,'x');
