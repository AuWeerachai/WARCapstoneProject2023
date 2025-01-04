
function [theta_rad, theta_dot, n_iter, cond_of_J] = compute_inverse_kinematics_ver2(converge_tol, n_iter_max, theta_rad, L, ...
                                                                                     Nvec, Ovec, Avec, Pvec, Ndotvec, Odotvec, Adotvec, Pdotvec)                                                                                                              
for k=1:n_iter_max

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

 Amap = A1 * A2 * A3 * A4 * A5 * A6;
 
 F = [Nvec; Ovec; Avec; Pvec] - [Amap(1:3,1); Amap(1:3,2); Amap(1:3,3); Amap(1:3,4)];

 J = compute_exact_jacobian_ver2(theta_rad, L);
 
 moore_penrose_inv = pinv(J); Y = moore_penrose_inv * F;

 %Y = J \ F;

 theta_rad = theta_rad + Y;

 if k > 1

    if max(abs(theta_rad - theta_rad_prev)) < converge_tol
       break;
    end

 end

 theta_rad_prev = theta_rad;

end % for k=1:n_iter_max

theta_dot = moore_penrose_inv * [Ndotvec; Odotvec; Adotvec; Pdotvec];

%theta_dot = J \ [Ndotvec; Odotvec; Adotvec; Pdotvec];

% condition number of jacobian
cond_of_J = cond(J);

n_iter = k;

return