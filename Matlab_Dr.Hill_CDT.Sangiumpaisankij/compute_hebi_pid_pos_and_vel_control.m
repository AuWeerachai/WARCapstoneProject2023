%
% Function: compute_hebi_pid_pos_and_vel_control.m
% Purpose: Compute the PID position controller for the 6 dof actuators.
%
% Inputs:
% cmd_pos_rad: 1x6 command gimbal pos (rad)
% cmd_vel_rps: 1x6 command gimbal vel (rad/sec)
% fbk_pos_rad: 1x6 sensed  gimbal pos (rad)
% fbk_vel_rps: 1x6 sensed  gimbal vel (rad/sec)
% Kp_pos:      1x6 proportional pos gains (N-m/rad)
% Ki_pos:      1x6 integral     pos gains (N-m-sec)
% Kd_pos:      1x6 derivative   pos gains (N-m/rad/sec)
% Kp_vel:      1x6 proportional vel gains (N-m/rad/sec)
% Ts:          1x1 sample time (sec)
% trq_lim_Nm:  1x6 torque limit (N-m) 
%
% Outputs:
% pwm_out: PWM motor command (-1 <= pwm_out <= +1)
%
% HEBI actuator names to gimbals
%
% family = 'Arm';
% names = {'grip-wrist', 'm2','m3','m4','m5','m6','m7','m8'};
%
%          rotation about next inner body (NIB)
% m1, G6,  +X
% m2, G5,  -Y
% m3, G4,  -Y
% m4, gripper, not used by PID
% m5, G3,  +X
% m6, G2R, -Y
% m7, G2L, +Y
% m8, G1,  +Z
%
function pwm_out = compute_hebi_pid_pos_and_vel_control(cmd_pos_rad, cmd_vel_rps, fbk_pos_rad, fbk_vel_rps, Kp_pos, Ki_pos, Kd_pos, Kp_vel, Ts, trq_lim_Nm)

persistent pos_error_prev
persistent int_pos_error_prev

if isempty(pos_error_prev)
   pos_error_prev = zeros(1,6); int_pos_error_prev = zeros(1,6);
end

pos_error = cmd_pos_rad - fbk_pos_rad;
vel_error = cmd_vel_rps - fbk_vel_rps;

prop_pos_term  = Kp_pos .* pos_error;
deriv_pos_term = Kd_pos .* (pos_error - pos_error_prev) / Ts;
int_pos_error  = int_pos_error_prev + 0.5 * (pos_error + pos_error_prev) * Ts;
int_pos_term   = Ki_pos .* int_pos_error;
prop_vel_term  = Kp_vel .* vel_error;

trq_out_Nm = prop_pos_term + deriv_pos_term + int_pos_term + prop_vel_term;

for i=1:6

  if abs(trq_out_Nm(i)) >= trq_lim_Nm(i)
    
     trq_out_Nm(i) = sign(trq_out_Nm(i)) * trq_lim_Nm(i);
   
     int_pos_error(i) = int_pos_error_prev(i);
  end

end

pwm_out = trq_out_Nm ./ trq_lim_Nm;

pos_error_prev = pos_error; int_pos_error_prev = int_pos_error;

return