
function ef_cmds = compute_ef_cmds_ver2(ef_cmd_pos, ef_cmd_vel, ef_cmd_quat_ItoE, ef_cmd_rate_rps)
%
% direction cosine matrix map from end effector frame to base frame 
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
% reserve storage
ef_cmds = zeros(24,1);

C_EtoI = util_quat_to_DC(util_quat_conj(ef_cmd_quat_ItoE));

ef_cmds(1:3)   = C_EtoI(1:3,1);
ef_cmds(4:6)   = C_EtoI(1:3,2);
ef_cmds(7:9)   = C_EtoI(1:3,3);
ef_cmds(10:12) = ef_cmd_pos;
%
% skew(W) = [ 0 -Wz Wy
%            Wz  0 -Wx
%           -Wy  Wx 0]
%
w_skew_ErelIinE = zeros(3,3);

w_skew_ErelIinE(1,2) = -ef_cmd_rate_rps(3);
w_skew_ErelIinE(1,3) =  ef_cmd_rate_rps(2);
w_skew_ErelIinE(2,1) =  ef_cmd_rate_rps(3);
w_skew_ErelIinE(2,3) = -ef_cmd_rate_rps(1);
w_skew_ErelIinE(3,1) = -ef_cmd_rate_rps(2);
w_skew_ErelIinE(3,2) =  ef_cmd_rate_rps(1);

Cdot_EtoI = C_EtoI * w_skew_ErelIinE;

ef_cmds(13:15) = Cdot_EtoI(1:3,1);
ef_cmds(16:18) = Cdot_EtoI(1:3,2);
ef_cmds(19:21) = Cdot_EtoI(1:3,3);
ef_cmds(22:24) = ef_cmd_vel;

return