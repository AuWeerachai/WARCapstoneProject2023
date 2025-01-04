function Jacobian = compute_exact_jacobian_ver2(theta_rad, L)

L34 = L(3) + L(4);

L5x = L(5); L5y = -L(7) - L(8);

L6x = L(6) + L(10); L6y = -L(9);

cs_theta = cos(theta_rad); sn_theta = sin(theta_rad);

%% Par Nx wrt Theta 1 to 6
par_nx_wrt_th1 = -sn_theta(1)*cs_theta(2)*cs_theta(4)*cs_theta(5) + sn_theta(1)*cs_theta(2)*sn_theta(4)*sn_theta(5) + sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5) + sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5) ...
               +  cs_theta(1)*sn_theta(3)*sn_theta(4)*cs_theta(5) + cs_theta(1)*sn_theta(3)*cs_theta(4)*sn_theta(5);
           
par_nx_wrt_th2 = -cs_theta(1)*sn_theta(2)*cs_theta(4)*cs_theta(5) + cs_theta(1)*sn_theta(2)*sn_theta(4)*sn_theta(5) - cs_theta(1)*cs_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5) - cs_theta(1)*cs_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5);

par_nx_wrt_th3 =  cs_theta(1)*sn_theta(2)*sn_theta(3)*sn_theta(4)*cs_theta(5) + cs_theta(1)*sn_theta(2)*sn_theta(3)*cs_theta(4)*sn_theta(5) ...
               +  sn_theta(1)*cs_theta(3)*sn_theta(4)*cs_theta(5) + sn_theta(1)*cs_theta(3)*cs_theta(4)*sn_theta(5);

par_nx_wrt_th4 = -cs_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5) - cs_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5) - cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5) + cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5) ...
               +  sn_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5) - sn_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5);

par_nx_wrt_th5 = -cs_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5) - cs_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5) + cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5) - cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5) ...
               -  sn_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5) + sn_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5);

par_nx_wrt_th6 = 0;

%% Par Ny wrt Theta 1 to 6

par_ny_wrt_th1 =  cs_theta(1)*cs_theta(2)*cs_theta(4)*cs_theta(5) - cs_theta(1)*cs_theta(2)*sn_theta(4)*sn_theta(5) - cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5) - cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5) ...
               +  sn_theta(1)*sn_theta(3)*sn_theta(4)*cs_theta(5) + sn_theta(1)*sn_theta(3)*cs_theta(4)*sn_theta(5);

par_ny_wrt_th2 = -sn_theta(1)*sn_theta(2)*cs_theta(4)*cs_theta(5) + sn_theta(1)*sn_theta(2)*sn_theta(4)*sn_theta(5) - sn_theta(1)*cs_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5) - sn_theta(1)*cs_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5);

par_ny_wrt_th3 =  sn_theta(1)*sn_theta(2)*sn_theta(3)*sn_theta(4)*cs_theta(5) + sn_theta(1)*sn_theta(2)*sn_theta(3)*cs_theta(4)*sn_theta(5) ...
               -  cs_theta(1)*cs_theta(3)*sn_theta(4)*cs_theta(5) - cs_theta(1)*cs_theta(3)*cs_theta(4)*sn_theta(5);

par_ny_wrt_th4 = -sn_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5) - sn_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5) - sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5) + sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5) ...
               -  cs_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5) + cs_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5);

par_ny_wrt_th5 = -sn_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5) - sn_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5) + sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5) - sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5) ...
               +  cs_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5) - cs_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5);

par_ny_wrt_th6 = 0;

%% Par Nz wrt Theta 1 to 6

par_nz_wrt_th1 = 0;

par_nz_wrt_th2 =  cs_theta(2)*cs_theta(4)*cs_theta(5) - cs_theta(2)*sn_theta(4)*sn_theta(5) - sn_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5) - sn_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5);

par_nz_wrt_th3 = -cs_theta(2)*sn_theta(3)*sn_theta(4)*cs_theta(5) - cs_theta(2)*sn_theta(3)*cs_theta(4)*sn_theta(5);

par_nz_wrt_th4 = -sn_theta(2)*sn_theta(4)*cs_theta(5) - sn_theta(2)*cs_theta(4)*sn_theta(5) + cs_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5) - cs_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5);

par_nz_wrt_th5 = -sn_theta(2)*cs_theta(4)*sn_theta(5) - sn_theta(2)*sn_theta(4)*cs_theta(5) - cs_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5) + cs_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5);

par_nz_wrt_th6 = 0;
           
%% Par Ox wrt Theta 1 to 6

par_ox_wrt_th1 =  sn_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5)*sn_theta(6) + sn_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5)*sn_theta(6) + sn_theta(1)*sn_theta(2)*sn_theta(3)*cs_theta(6) ...
               -  sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) + sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6)                            ...
               -  cs_theta(1)*cs_theta(3)*cs_theta(6) - cs_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) + cs_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6);

par_ox_wrt_th2 =  cs_theta(1)*sn_theta(2)*cs_theta(4)*sn_theta(5)*sn_theta(6) + cs_theta(1)*sn_theta(2)*sn_theta(4)*cs_theta(5)*sn_theta(6) - cs_theta(1)*cs_theta(2)*sn_theta(3)*cs_theta(6) ...
               +  cs_theta(1)*cs_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) - cs_theta(1)*cs_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6);

par_ox_wrt_th3 = -cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(6) - cs_theta(1)*sn_theta(2)*sn_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) + cs_theta(1)*sn_theta(2)*sn_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6) ...
               +  sn_theta(1)*sn_theta(3)*cs_theta(6) - sn_theta(1)*cs_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) + sn_theta(1)*cs_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6);

par_ox_wrt_th4 =  cs_theta(1)*cs_theta(2)*sn_theta(4)*sn_theta(5)*sn_theta(6) - cs_theta(1)*cs_theta(2)*cs_theta(4)*cs_theta(5)*sn_theta(6) + cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5)*sn_theta(6) ...
               +  cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5)*sn_theta(6) - sn_theta(1)*sn_theta(3)*cs_theta(4)*sn_theta(5)*sn_theta(6) - sn_theta(1)*sn_theta(3)*sn_theta(4)*cs_theta(5)*sn_theta(6);

par_ox_wrt_th5 = -cs_theta(1)*cs_theta(2)*cs_theta(4)*cs_theta(5)*sn_theta(6) + cs_theta(1)*cs_theta(2)*sn_theta(4)*sn_theta(5)*sn_theta(6) + cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5)*sn_theta(6) ...
               +  cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5)*sn_theta(6) - sn_theta(1)*sn_theta(3)*sn_theta(4)*cs_theta(5)*sn_theta(6) - sn_theta(1)*sn_theta(3)*cs_theta(4)*sn_theta(5)*sn_theta(6);

par_ox_wrt_th6 = -cs_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5)*cs_theta(6) - cs_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5)*cs_theta(6) + cs_theta(1)*sn_theta(2)*sn_theta(3)*sn_theta(6) ...
               +  cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) - cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6)                            ...
               +  sn_theta(1)*cs_theta(3)*sn_theta(6) - sn_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) + sn_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6);

%% Par Oy wrt Theta 1 to 6

par_oy_wrt_th1 = -cs_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5)*sn_theta(6) - cs_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5)*sn_theta(6) - cs_theta(1)*sn_theta(2)*sn_theta(3)*cs_theta(6) ...
               +  cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) - cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6)                            ...
               -  sn_theta(1)*cs_theta(3)*cs_theta(6) - sn_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) + sn_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6);
           
par_oy_wrt_th2 =  sn_theta(1)*sn_theta(2)*cs_theta(4)*sn_theta(5)*sn_theta(6) + sn_theta(1)*sn_theta(2)*sn_theta(4)*cs_theta(5)*sn_theta(6) - sn_theta(1)*cs_theta(2)*sn_theta(3)*cs_theta(6) ...
               +  sn_theta(1)*cs_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) - sn_theta(1)*cs_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6);

par_oy_wrt_th3 = -sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(6) - sn_theta(1)*sn_theta(2)*sn_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) + sn_theta(1)*sn_theta(2)*sn_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6) ...
               -  cs_theta(1)*sn_theta(3)*cs_theta(6) + cs_theta(1)*cs_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) - cs_theta(1)*cs_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6);

par_oy_wrt_th4 =  sn_theta(1)*cs_theta(2)*sn_theta(4)*sn_theta(5)*sn_theta(6) - sn_theta(1)*cs_theta(2)*cs_theta(4)*cs_theta(5)*sn_theta(6) + sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5)*sn_theta(6)  ...
               +  sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5)*sn_theta(6) +  cs_theta(1)*sn_theta(3)*cs_theta(4)*sn_theta(5)*sn_theta(6) + cs_theta(1)*sn_theta(3)*sn_theta(4)*cs_theta(5)*sn_theta(6);

par_oy_wrt_th5 = -sn_theta(1)*cs_theta(2)*cs_theta(4)*cs_theta(5)*sn_theta(6) + sn_theta(1)*cs_theta(2)*sn_theta(4)*sn_theta(5)*sn_theta(6) + sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5)*sn_theta(6) ...
               +  sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5)*sn_theta(6) + cs_theta(1)*sn_theta(3)*sn_theta(4)*cs_theta(5)*sn_theta(6) + cs_theta(1)*sn_theta(3)*cs_theta(4)*sn_theta(5)*sn_theta(6);

par_oy_wrt_th6 = -sn_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5)*cs_theta(6) - sn_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5)*cs_theta(6) + sn_theta(1)*sn_theta(2)*sn_theta(3)*sn_theta(6) ...
               +  sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) - sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6)                            ...
               -  cs_theta(1)*cs_theta(3)*sn_theta(6) + cs_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) - cs_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6);
         
%% Par Oz wrt Theta 1 to 6

par_oz_wrt_th1 = 0;

par_oz_wrt_th2 = -cs_theta(2)*cs_theta(4)*sn_theta(5)*sn_theta(6) - cs_theta(2)*sn_theta(4)*cs_theta(5)*sn_theta(6) - sn_theta(2)*sn_theta(3)*cs_theta(6) + sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) ...
               -  sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6);

par_oz_wrt_th3 =  cs_theta(2)*cs_theta(3)*cs_theta(6) + cs_theta(2)*sn_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) - cs_theta(2)*sn_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6);

par_oz_wrt_th4 =  sn_theta(2)*sn_theta(4)*sn_theta(5)*sn_theta(6) - sn_theta(2)*cs_theta(4)*cs_theta(5)*sn_theta(6) - cs_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5)*sn_theta(6) - cs_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5)*sn_theta(6);

par_oz_wrt_th5 = -sn_theta(2)*cs_theta(4)*cs_theta(5)*sn_theta(6) + sn_theta(2)*sn_theta(4)*sn_theta(5)*sn_theta(6) - cs_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5)*sn_theta(6) - cs_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5)*sn_theta(6);

par_oz_wrt_th6 = -sn_theta(2)*cs_theta(4)*sn_theta(5)*cs_theta(6) - sn_theta(2)*sn_theta(4)*cs_theta(5)*cs_theta(6) - cs_theta(2)*sn_theta(3)*sn_theta(6) - cs_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) + cs_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6);

%% Par Ax wrt Theta 1 to 6

par_ax_wrt_th1 =  sn_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5)*cs_theta(6) + sn_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5)*cs_theta(6) - sn_theta(1)*sn_theta(2)*sn_theta(3)*sn_theta(6) ...
               -  sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) + sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6) ...
               +  cs_theta(1)*cs_theta(3)*sn_theta(6) - cs_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) + cs_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6);

par_ax_wrt_th2 =  cs_theta(1)*sn_theta(2)*cs_theta(4)*sn_theta(5)*cs_theta(6) + cs_theta(1)*sn_theta(2)*sn_theta(4)*cs_theta(5)*cs_theta(6) + cs_theta(1)*cs_theta(2)*sn_theta(3)*sn_theta(6) ...
               +  cs_theta(1)*cs_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) - cs_theta(1)*cs_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6);

par_ax_wrt_th3 =  cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(6) - cs_theta(1)*sn_theta(2)*sn_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) + cs_theta(1)*sn_theta(2)*sn_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6) ...
               -  sn_theta(1)*sn_theta(3)*sn_theta(6) - sn_theta(1)*cs_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) + sn_theta(1)*cs_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6);

par_ax_wrt_th4 =  cs_theta(1)*cs_theta(2)*sn_theta(4)*sn_theta(5)*cs_theta(6) - cs_theta(1)*cs_theta(2)*cs_theta(4)*cs_theta(5)*cs_theta(6) + cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5)*cs_theta(6) ...
               +  cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5)*cs_theta(6) - sn_theta(1)*sn_theta(3)*cs_theta(4)*sn_theta(5)*cs_theta(6) - sn_theta(1)*sn_theta(3)*sn_theta(4)*cs_theta(5)*cs_theta(6);
           
par_ax_wrt_th5 = -cs_theta(1)*cs_theta(2)*cs_theta(4)*cs_theta(5)*cs_theta(6) + cs_theta(1)*cs_theta(2)*sn_theta(4)*sn_theta(5)*cs_theta(6) + cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5)*cs_theta(6) ...
               +  cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5)*cs_theta(6) -  sn_theta(1)*sn_theta(3)*sn_theta(4)*cs_theta(5)*cs_theta(6) - sn_theta(1)*sn_theta(3)*cs_theta(4)*sn_theta(5)*cs_theta(6);

par_ax_wrt_th6 =  cs_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5)*sn_theta(6) + cs_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5)*sn_theta(6) + cs_theta(1)*sn_theta(2)*sn_theta(3)*cs_theta(6) - cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) ...
               +  cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6) +  sn_theta(1)*cs_theta(3)*cs_theta(6) + sn_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) - sn_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6);

%% Par Ay wrt Theta 1 to 6

par_ay_wrt_th1 = -cs_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5)*cs_theta(6) - cs_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5)*cs_theta(6) + cs_theta(1)*sn_theta(2)*sn_theta(3)*sn_theta(6) + cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6)...
               -  cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6) +  sn_theta(1)*cs_theta(3)*sn_theta(6) - sn_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) + sn_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6);

par_ay_wrt_th2 =  sn_theta(1)*sn_theta(2)*cs_theta(4)*sn_theta(5)*cs_theta(6) + sn_theta(1)*sn_theta(2)*sn_theta(4)*cs_theta(5)*cs_theta(6) + sn_theta(1)*cs_theta(2)*sn_theta(3)*sn_theta(6) + sn_theta(1)*cs_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) ...
               -  sn_theta(1)*cs_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6);

par_ay_wrt_th3 =  sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(6) - sn_theta(1)*sn_theta(2)*sn_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) + sn_theta(1)*sn_theta(2)*sn_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6) ...
               +  cs_theta(1)*sn_theta(3)*sn_theta(6) + cs_theta(1)*cs_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) - cs_theta(1)*cs_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6);

par_ay_wrt_th4 =  sn_theta(1)*cs_theta(2)*sn_theta(4)*sn_theta(5)*cs_theta(6) - sn_theta(1)*cs_theta(2)*cs_theta(4)*cs_theta(5)*cs_theta(6) + sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5)*cs_theta(6) ...
               +  sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5)*cs_theta(6) +  cs_theta(1)*sn_theta(3)*cs_theta(4)*sn_theta(5)*cs_theta(6) + cs_theta(1)*sn_theta(3)*sn_theta(4)*cs_theta(5)*cs_theta(6);

par_ay_wrt_th5 = -sn_theta(1)*cs_theta(2)*cs_theta(4)*cs_theta(5)*cs_theta(6) + sn_theta(1)*cs_theta(2)*sn_theta(4)*sn_theta(5)*cs_theta(6) + sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5)*cs_theta(6) + sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5)*cs_theta(6) ...
               + cs_theta(1)*sn_theta(3)*sn_theta(4)*cs_theta(5)*cs_theta(6) + cs_theta(1)*sn_theta(3)*cs_theta(4)*sn_theta(5)*cs_theta(6);

par_ay_wrt_th6 =  sn_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5)*sn_theta(6) + sn_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5)*sn_theta(6) + sn_theta(1)*sn_theta(2)*sn_theta(3)*cs_theta(6) - sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) ...
               +  sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6) -  cs_theta(1)*cs_theta(3)*cs_theta(6) - cs_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) + cs_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6);

%% Par Az wrt Theta 1 to 6

par_az_wrt_th1 = 0;

par_az_wrt_th2 = -cs_theta(2)*cs_theta(4)*sn_theta(5)*cs_theta(6) - cs_theta(2)*sn_theta(4)*cs_theta(5)*cs_theta(6) + sn_theta(2)*sn_theta(3)*sn_theta(6) + sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) - sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6);

par_az_wrt_th3 = -cs_theta(2)*cs_theta(3)*sn_theta(6) + cs_theta(2)*sn_theta(3)*sn_theta(4)*sn_theta(5)*cs_theta(6) - cs_theta(2)*sn_theta(3)*cs_theta(4)*cs_theta(5)*cs_theta(6);

par_az_wrt_th4 =  sn_theta(2)*sn_theta(4)*sn_theta(5)*cs_theta(6) - sn_theta(2)*cs_theta(4)*cs_theta(5)*cs_theta(6) - cs_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5)*cs_theta(6) - cs_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5)*cs_theta(6);

par_az_wrt_th5 = -sn_theta(2)*cs_theta(4)*cs_theta(5)*cs_theta(6) + sn_theta(2)*sn_theta(4)*sn_theta(5)*cs_theta(6) - cs_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5)*cs_theta(6) - cs_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5)*cs_theta(6);

par_az_wrt_th6 =  sn_theta(2)*cs_theta(4)*sn_theta(5)*sn_theta(6) + sn_theta(2)*sn_theta(4)*cs_theta(5)*sn_theta(6) - cs_theta(2)*sn_theta(3)*cs_theta(6) + cs_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5)*sn_theta(6) - cs_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5)*sn_theta(6);

%% Par Px wrt Theta 1 to 6

par_px_wrt_th1 = -L6x*sn_theta(1)*cs_theta(2)*cs_theta(4)*cs_theta(5) - L5x*sn_theta(1)*cs_theta(2)*cs_theta(4) + L6x*sn_theta(1)*cs_theta(2)*sn_theta(4)*sn_theta(5) - L34*sn_theta(1)*cs_theta(2) ...
               + (L5y + L6y)*sn_theta(1)*sn_theta(2)*sn_theta(3) + L6x*sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5) + L5x*sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4) + L6x*sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5) - L(2)*sn_theta(1) ...
               - (L5y + L6y)*cs_theta(1)*cs_theta(3) + L6x*cs_theta(1)*sn_theta(3)*sn_theta(4)*cs_theta(5) + L5x*cs_theta(1)*sn_theta(3)*sn_theta(4) + L6x*cs_theta(1)*sn_theta(3)*cs_theta(4)*sn_theta(5);

par_px_wrt_th2 = -L6x*cs_theta(1)*sn_theta(2)*cs_theta(4)*cs_theta(5) - L5x*cs_theta(1)*sn_theta(2)*cs_theta(4) + L6x*cs_theta(1)*sn_theta(2)*sn_theta(4)*sn_theta(5) - L34*cs_theta(1)*sn_theta(2) ...
               - (L5y + L6y)*cs_theta(1)*cs_theta(2)*sn_theta(3) - L6x*cs_theta(1)*cs_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5) - L5x*cs_theta(1)*cs_theta(2)*cs_theta(3)*sn_theta(4) - L6x*cs_theta(1)*cs_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5);

par_px_wrt_th3 = -(L5y + L6y)*cs_theta(1)*sn_theta(2)*cs_theta(3) + L6x*cs_theta(1)*sn_theta(2)*sn_theta(3)*sn_theta(4)*cs_theta(5) + L5x*cs_theta(1)*sn_theta(2)*sn_theta(3)*sn_theta(4) + L6x*cs_theta(1)*sn_theta(2)*sn_theta(3)*cs_theta(4)*sn_theta(5) ...
               +  (L5y + L6y)*sn_theta(1)*sn_theta(3) + L6x*sn_theta(1)*cs_theta(3)*sn_theta(4)*cs_theta(5) + L5x*sn_theta(1)*cs_theta(3)*sn_theta(4) + L6x*sn_theta(1)*cs_theta(3)*cs_theta(4)*sn_theta(5);

par_px_wrt_th4 = -L6x*cs_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5) - L5x*cs_theta(1)*cs_theta(2)*sn_theta(4) - L6x*cs_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5) ...
              -   L6x*cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5) - L5x*cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4) + L6x*cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5) ...
              +   L6x*sn_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5) + L5x*sn_theta(1)*sn_theta(3)*cs_theta(4) - L6x*sn_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5);

par_px_wrt_th5 = -L6x*cs_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5) - L6x*cs_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5) ...
               +  L6x*cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5) - L6x*cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5) ...
               -  L6x*sn_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5) + L6x*sn_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5);

par_px_wrt_th6 = 0;

%% Par Py wrt Theta 1 to 6

par_py_wrt_th1 =  L6x*cs_theta(1)*cs_theta(2)*cs_theta(4)*cs_theta(5) + L5x*cs_theta(1)*cs_theta(2)*cs_theta(4) - L6x*cs_theta(1)*cs_theta(2)*sn_theta(4)*sn_theta(5) + L34*cs_theta(1)*cs_theta(2) ...
               - (L5y + L6y)*cs_theta(1)*sn_theta(2)*sn_theta(3) - L6x*cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5) - L5x*cs_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4) - L6x*cs_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5) + L(2)*cs_theta(1) ...
               - (L5y + L6y)*sn_theta(1)*cs_theta(3) + L6x*sn_theta(1)*sn_theta(3)*sn_theta(4)*cs_theta(5) + L5x*sn_theta(1)*sn_theta(3)*sn_theta(4) + L6x*sn_theta(1)*sn_theta(3)*cs_theta(4)*sn_theta(5);

par_py_wrt_th2 = -L6x*sn_theta(1)*sn_theta(2)*cs_theta(4)*cs_theta(5) - L5x*sn_theta(1)*sn_theta(2)*cs_theta(4) + L6x*sn_theta(1)*sn_theta(2)*sn_theta(4)*sn_theta(5) - L34*sn_theta(1)*sn_theta(2) ...
               - (L5y + L6y)*sn_theta(1)*cs_theta(2)*sn_theta(3) - L6x*sn_theta(1)*cs_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5) - L5x*sn_theta(1)*cs_theta(2)*cs_theta(3)*sn_theta(4) - L6x*sn_theta(1)*cs_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5);

par_py_wrt_th3 = -(L5y + L6y)*sn_theta(1)*sn_theta(2)*cs_theta(3) + L6x*sn_theta(1)*sn_theta(2)*sn_theta(3)*sn_theta(4)*cs_theta(5) + L5x*sn_theta(1)*sn_theta(2)*sn_theta(3)*sn_theta(4) + L6x*sn_theta(1)*sn_theta(2)*sn_theta(3)*cs_theta(4)*sn_theta(5) ...
               -  (L5y + L6y)*cs_theta(1)*sn_theta(3) - L6x*cs_theta(1)*cs_theta(3)*sn_theta(4)*cs_theta(5) - L5x*cs_theta(1)*cs_theta(3)*sn_theta(4) - L6x*cs_theta(1)*cs_theta(3)*cs_theta(4)*sn_theta(5);

par_py_wrt_th4 = -L6x*sn_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5) - L5x*sn_theta(1)*cs_theta(2)*sn_theta(4) - L6x*sn_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5) ...
               -  L6x*sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5) - L5x*sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4) + L6x*sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5) ...
               -  L6x*cs_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5) - L5x*cs_theta(1)*sn_theta(3)*cs_theta(4) + L6x*cs_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5);

par_py_wrt_th5 = -L6x*sn_theta(1)*cs_theta(2)*cs_theta(4)*sn_theta(5) - L6x*sn_theta(1)*cs_theta(2)*sn_theta(4)*cs_theta(5) ...
               +  L6x*sn_theta(1)*sn_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5) - L6x*sn_theta(1)*sn_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5) ...
               +  L6x*cs_theta(1)*sn_theta(3)*sn_theta(4)*sn_theta(5) - L6x*cs_theta(1)*sn_theta(3)*cs_theta(4)*cs_theta(5);

par_py_wrt_th6 = 0;

%% Par Pz wrt Theta 1 to 6

par_pz_wrt_th1 = 0;

par_pz_wrt_th2 =  L6x*cs_theta(2)*cs_theta(4)*cs_theta(5) + L5x*cs_theta(2)*cs_theta(4) - L6x*cs_theta(2)*sn_theta(4)*sn_theta(5) + L34*cs_theta(2) ...
               - (L5y + L6y)*sn_theta(2)*sn_theta(3) - L6x*sn_theta(2)*cs_theta(3)*sn_theta(4)*cs_theta(5) - L5x*sn_theta(2)*cs_theta(3)*sn_theta(4) - L6x*sn_theta(2)*cs_theta(3)*cs_theta(4)*sn_theta(5);
           
par_pz_wrt_th3 = (L5y + L6y)*cs_theta(2)*cs_theta(3) - L6x*cs_theta(2)*sn_theta(3)*sn_theta(4)*cs_theta(5) - L5x*cs_theta(2)*sn_theta(3)*sn_theta(4) - L6x*cs_theta(2)*sn_theta(3)*cs_theta(4)*sn_theta(5);
 
par_pz_wrt_th4 = -L6x*sn_theta(2)*sn_theta(4)*cs_theta(5) - L5x*sn_theta(2)*sn_theta(4) - L6x*sn_theta(2)*cs_theta(4)*sn_theta(5) ...
               +  L6x*cs_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5) + L5x*cs_theta(2)*cs_theta(3)*cs_theta(4) - L6x*cs_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5);

par_pz_wrt_th5 = -L6x*sn_theta(2)*cs_theta(4)*sn_theta(5) - L6x*sn_theta(2)*sn_theta(4)*cs_theta(5) ...
               -  L6x*cs_theta(2)*cs_theta(3)*sn_theta(4)*sn_theta(5) + L6x*cs_theta(2)*cs_theta(3)*cs_theta(4)*cs_theta(5);

par_pz_wrt_th6 = 0;

Jacobian = zeros(12,6);

Jacobian(1,1) = par_nx_wrt_th1;
Jacobian(1,2) = par_nx_wrt_th2;
Jacobian(1,3) = par_nx_wrt_th3;
Jacobian(1,4) = par_nx_wrt_th4;
Jacobian(1,5) = par_nx_wrt_th5;
Jacobian(1,6) = par_nx_wrt_th6;

Jacobian(2,1) = par_ny_wrt_th1;
Jacobian(2,2) = par_ny_wrt_th2;
Jacobian(2,3) = par_ny_wrt_th3;
Jacobian(2,4) = par_ny_wrt_th4;
Jacobian(2,5) = par_ny_wrt_th5;
Jacobian(2,6) = par_ny_wrt_th6;

Jacobian(3,1) = par_nz_wrt_th1;
Jacobian(3,2) = par_nz_wrt_th2;
Jacobian(3,3) = par_nz_wrt_th3;
Jacobian(3,4) = par_nz_wrt_th4;
Jacobian(3,5) = par_nz_wrt_th5;
Jacobian(3,6) = par_nz_wrt_th6;

Jacobian(4,1) = par_ox_wrt_th1;
Jacobian(4,2) = par_ox_wrt_th2;
Jacobian(4,3) = par_ox_wrt_th3;
Jacobian(4,4) = par_ox_wrt_th4;
Jacobian(4,5) = par_ox_wrt_th5;
Jacobian(4,6) = par_ox_wrt_th6;

Jacobian(5,1) = par_oy_wrt_th1;
Jacobian(5,2) = par_oy_wrt_th2;
Jacobian(5,3) = par_oy_wrt_th3;
Jacobian(5,4) = par_oy_wrt_th4;
Jacobian(5,5) = par_oy_wrt_th5;
Jacobian(5,6) = par_oy_wrt_th6;

Jacobian(6,1) = par_oz_wrt_th1;
Jacobian(6,2) = par_oz_wrt_th2;
Jacobian(6,3) = par_oz_wrt_th3;
Jacobian(6,4) = par_oz_wrt_th4;
Jacobian(6,5) = par_oz_wrt_th5;
Jacobian(6,6) = par_oz_wrt_th6;

Jacobian(7,1) = par_ax_wrt_th1;
Jacobian(7,2) = par_ax_wrt_th2;
Jacobian(7,3) = par_ax_wrt_th3;
Jacobian(7,4) = par_ax_wrt_th4;
Jacobian(7,5) = par_ax_wrt_th5;
Jacobian(7,6) = par_ax_wrt_th6;

Jacobian(8,1) = par_ay_wrt_th1;
Jacobian(8,2) = par_ay_wrt_th2;
Jacobian(8,3) = par_ay_wrt_th3;
Jacobian(8,4) = par_ay_wrt_th4;
Jacobian(8,5) = par_ay_wrt_th5;
Jacobian(8,6) = par_ay_wrt_th6;

Jacobian(9,1) = par_az_wrt_th1;
Jacobian(9,2) = par_az_wrt_th2;
Jacobian(9,3) = par_az_wrt_th3;
Jacobian(9,4) = par_az_wrt_th4;
Jacobian(9,5) = par_az_wrt_th5;
Jacobian(9,6) = par_az_wrt_th6;

Jacobian(10,1) = par_px_wrt_th1;
Jacobian(10,2) = par_px_wrt_th2;
Jacobian(10,3) = par_px_wrt_th3;
Jacobian(10,4) = par_px_wrt_th4;
Jacobian(10,5) = par_px_wrt_th5;
Jacobian(10,6) = par_px_wrt_th6;

Jacobian(11,1) = par_py_wrt_th1;
Jacobian(11,2) = par_py_wrt_th2;
Jacobian(11,3) = par_py_wrt_th3;
Jacobian(11,4) = par_py_wrt_th4;
Jacobian(11,5) = par_py_wrt_th5;
Jacobian(11,6) = par_py_wrt_th6;

Jacobian(12,1) = par_pz_wrt_th1;
Jacobian(12,2) = par_pz_wrt_th2;
Jacobian(12,3) = par_pz_wrt_th3;
Jacobian(12,4) = par_pz_wrt_th4;
Jacobian(12,5) = par_pz_wrt_th5;
Jacobian(12,6) = par_pz_wrt_th6;

return