 
function Amap = compute_fwd_map_ver2(theta_rad, L)

% Rotation Sequence: Z, -Y, X, -Y, -Y, X
DCM1 = compute_rot_z(+theta_rad(1));

DCM2 = compute_rot_y(-theta_rad(2));

DCM3 = compute_rot_x(+theta_rad(3));

DCM4 = compute_rot_y(-theta_rad(4));

DCM5 = compute_rot_y(-theta_rad(5));

DCM6 = compute_rot_x(+theta_rad(6));


A1 = [DCM1 [0; 0; L(1)]; 0 0 0 1];

A2 = [DCM2 [L(2); 0; 0]; 0 0 0 1];

A3 = [DCM3 [0; 0; 0]; 0 0 0 1];

A4 = [DCM4 [L(3)+L(4); 0; 0]; 0 0 0 1];

A5 = [DCM5 [L(5); -L(7)-L(8); 0]; 0 0 0 1];

A6 = [DCM6 [L(6)+L(10); -L(9); 0]; 0 0 0 1];

% dcm from ef frame to base frame
Amap = A1 * A2 * A3 * A4 * A5 * A6;

return