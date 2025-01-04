function [err_flag_out, y]  = compute_ef_atitude_cmd_profile(t, CMD_STRUCT)

persistent accl_limit_rps2 
persistent curr_slew_strt_time 
persistent curr_slew_stop_time
persistent err_flag
persistent init_flag 
persistent i_tg 
persistent q_ItoB1 
persistent slew_eigen_axis 
persistent slew_strt_time 
persistent   t1   t2   t3   t4   t5   t6   t7
persistent x_t1 x_t2 x_t3 x_t4 x_t5 x_t6 
persistent w_t1 w_t2 w_t3 w_t4 w_t5 w_t6
persistent y_prev

% initialize error flag
err_flag = 0;

% initialization
if isempty(init_flag)
   
   accl_limit_rps2 = 0;
   
   curr_slew_strt_time = 0;
   
   curr_slew_stop_time = 0;
   
   i_tg = 0; 
   
   init_flag = 1;
   
   q_ItoB1 = zeros(4,1);
   
   slew_strt_time = 0;
   
   t1 = 0; t2 = 0; t3 = 0; t4 = 0; t5 = 0; t6 = 0; t7 = 0;
   
   slew_eigen_axis = zeros(3,1);
   
   x_t1 = 0; x_t2 = 0; x_t3 = 0; x_t4 = 0; x_t5 = 0; x_t6 = 0;
   w_t1 = 0; w_t2 = 0; w_t3 = 0; w_t4 = 0; w_t5 = 0; w_t6 = 0;
   
   y_prev = [CMD_STRUCT.Initq_ItoB; zeros(3,1); zeros(3,1)];
   
end

% it time is greater than start time and targets remaining then plan slew
if t >= slew_strt_time && i_tg < CMD_STRUCT.n_tg
       
   % next target pointer
   i_tg = i_tg + 1;
    
   % slew start quaternion
   q_ItoB1 = y_prev(1:4);
   
   % slew stop quaterion
   q_ItoB2 = CMD_STRUCT.q_ItoB(i_tg,:)';
   
   % slew duration
   slew_duration_sec = CMD_STRUCT.strt_time(i_tg) - slew_strt_time;
   
   % eigenslew parameters
   [err_flag, tJ, tR, tC, slew_eigen_axis, accl_limit_rps2] = ...
                                                              ...
   ComputeEigenSlewParams(q_ItoB1, q_ItoB2, CMD_STRUCT.accl_limit_rps2, CMD_STRUCT.min_slew_dur_sec, slew_duration_sec, CMD_STRUCT.tJ_pct);

   % if error then use 100 percent jerk versine profile
   if err_flag == 1 || err_flag == 2
       
      tJ = 0.25 * slew_duration_sec;
      tR = 0;
      tC = 0;
      
      t1x = tJ; t2x = tJ; t3x = 2*tJ; t32 = t3x - t2x; t4x = t3x; t5x = 3*tJ; t54 = t5x - t4x; t6x = t5x; t7x = 4*tJ; t76 = t7x - t6x;
          
      term_x_t6 = 0.5*( 0.5*t1x^2 - 2*(t1x/pi)^2                                     ...
                +     (t2x - t1x)*t3x + 0.5*t3x^2 - 0.5*t2x^2                        ...
                +     ((t32/pi)^2)*( 2 )                                             ...
                +     (-t1x + t2x + t3x )*( t4x - t3x )                              ...
                +     (-t1x + t2x + t3x )*( t5x - t4x )                              ...
                +      -0.5*(t5x^2 - t4x^2) + t4x*(t5x - t4x) + ( (t54/pi)^2 )*( 2 ) ...
                +     (-t1x + t2x + t3x + t4x - t5x)*(t6x - t5x)                     ...
                -       2*( 0.5*(t6x^2 - t5x^2) - t5x*(t6x - t5x) ) );
              
      term_x_t7 = 0.5*( (-t1x + t2x + t3x + t4x + t5x - 2*t6x)*(t7x - t6x) ...
                -        0.5*(t7x^2 - t6x^2) + t6x*(t7x - t6x)             ...
                -        ((t76/pi)^2)*( cos( pi*(t7x - t7x)/t76 ) + 1 ) );
            
      % quat from B1 to I
      q_B1toI = util_quat_conj(q_ItoB1);
            
      % quat from B1 to B2
      q_B1toB2 = util_quat_multiply(q_ItoB2, q_B1toI);
            
      % eigen rotation angle
      [slew_eigen_axis, slew_eigen_angle_rad] = ComputeEigenRotation(q_B1toB2);
          
      accl_limit_rps2 = slew_eigen_angle_rad / (term_x_t6 + term_x_t7);
          
      err_flag = 0;
      
   end

   t1 = tJ;
   t2 = tJ + tR;
   t3 = 2*tJ + tR;
   t4 = 2*tJ + tR + tC;
   t5 = 3*tJ + tR + tC;
   t6 = 3*tJ + 2*tR + tC;
   t7 = 4*tJ + 2*tR + tC;
 
   tp = [t1 t2 t3 t4 t5 t6 t7];
   
   [x_t1, x_t2, x_t3, x_t4, x_t5, x_t6, w_t1, w_t2, w_t3, w_t4, w_t5, w_t6] = ComputeVersineParams(accl_limit_rps2, tp);
   
   % current slew strt time
   curr_slew_strt_time = slew_strt_time;
   
   % current slew stop time
   curr_slew_stop_time = CMD_STRUCT.strt_time(i_tg);
   
   % next slew start time
   slew_strt_time = CMD_STRUCT.strt_time(i_tg) + CMD_STRUCT.duration(i_tg);
    
end

% if err_flag is zero and time is in slew window then slew
if err_flag == 0 && t >= curr_slew_strt_time && t <= curr_slew_stop_time
    
   dt = t - curr_slew_strt_time;

   [eigen_angle, eigen_rate, eigen_accl] = ...
                                           ...
   ComputeEigenSlewPosRateAccl(accl_limit_rps2, dt, t1, t2, t3, t4, t5, t6, t7, x_t1, x_t2, x_t3, x_t4, x_t5, x_t6, w_t1, w_t2, w_t3, w_t4, w_t5, w_t6);

   [qslew_ItoB, wslew, aslew] = ComputeCmdQuatRateAccel(q_ItoB1, slew_eigen_axis, eigen_angle, eigen_rate, eigen_accl);
   
   % if slew is done then hold last state
   if dt <= t7 
      y = [qslew_ItoB; wslew; aslew];
   else
      y = y_prev;
   end
   
   y_prev = y;
 
% otherwise, hold at previous state
else
   y = y_prev;
end

err_flag_out = err_flag;

return

function [err_flag, tJ, tR, tC, slew_eigen_axis, accl_limit_rps2] = ComputeEigenSlewParams(q_ItoB1, q_ItoB2, accl_limit_rps2, min_slew_dur_sec, slew_dur_sec, tJ_pct)

% initialize outputs
err_flag = 0; 

tJ = 0;
tR = 0; 
tC = 0;

% quat from B1 to I
q_B1toI = util_quat_conj(q_ItoB1);

% quat from B1 to B2
q_B1toB2 = util_quat_multiply(q_ItoB2, q_B1toI);

% eigen rotation angle
[slew_eigen_axis, slew_eigen_angle_rad] = ComputeEigenRotation(q_B1toB2);
    
if slew_dur_sec < min_slew_dur_sec
   err_flag = 3;
   return
end

% jerk time
tJ = 0.25*slew_dur_sec*tJ_pct*0.01;

% race and coast time
while 1
    
    if accl_limit_rps2 < 1e-4
       err_flag = 2;
       return
    end
    
    b = 3*tJ - slew_dur_sec;
    c = 2*tJ^2 - slew_dur_sec*tJ + slew_eigen_angle_rad/accl_limit_rps2;
    arg = b^2 - 4*c;
    if arg > 0
       soln1 = 0.5*(-b + sqrt(arg));
       soln2 = 0.5*(-b - sqrt(arg));
       solns = [soln1 soln2];
       index = solns > 0;
       if index(1) == 1 || index(2) == 1
          tR = min(solns(index));
          tC = slew_dur_sec - 4*tJ - 2*tR;
       end
    else
       err_flag = 1;
       return
    end
    
    if tC > 0
       break
    end
 
    accl_limit_rps2 = 0.90*accl_limit_rps2;
    
end

return

function [x_t1, x_t2, x_t3, x_t4, x_t5, x_t6, w_t1, w_t2, w_t3, w_t4, w_t5, w_t6] = ComputeVersineParams(am, tp)

t1 = tp(1);
t2 = tp(2);
t3 = tp(3);
t4 = tp(4);
t5 = tp(5);
t6 = tp(6);
t7 = tp(7);

t21 = t2 - t1;
t32 = t3 - t2;
t43 = t4 - t3;
t54 = t5 - t4;
t65 = t6 - t5;
t76 = t7 - t6;

w_t1 = -am*0.5*t1;

x_t1 = am*0.5*( 0.5*t1^2 - 2*(t1/pi)^2 );

w_t2 = -am*0.5*t1 + am*t2;

x_t2 = am*0.5*( 0.5*t1^2 - 2*(t1/pi)^2 - t1*t2 + t2^2  );

w_t3 = am*0.5*( -t1 + t2 + t3 );

x_t3 = am*0.5*( 0.5*t1^2 - 2*(t1/pi)^2             ...
     +          (t2 - t1)*t3 + 0.5*t3^2 - 0.5*t2^2 ...
     +          ((t32/pi)^2)*( 2 ) );
 
w_t4 = am*0.5*( -t1 + t2 + t3 );
 
x_t4 = am*0.5*( 0.5*t1^2 - 2*(t1/pi)^2             ...
     +          (t2 - t1)*t3 + 0.5*t3^2 - 0.5*t2^2 ...
     +          ((t32/pi)^2)*( 2 )                 ...
     +          (-t1 + t2 + t3)*( t4 - t3 ) );
 
w_t5 = am*0.5*( -t1 + t2 + t3 + t4 - t5 );

x_t5 = am*0.5*( 0.5*t1^2 - 2*(t1/pi)^2             ...
     +          (t2 - t1)*t3 + 0.5*t3^2 - 0.5*t2^2 ...
     +          ((t32/pi)^2 )*( 2 )                ...
     +          (-t1 + t2 + t3 )*( t4 - t3 )       ...
     +          (-t1 + t2 + t3 )*( t5 - t4 )       ...
     -          0.5*(t5^2 - t4^2) + t4*(t5 - t4) + ((t54/pi)^2)*( 2 ) );

w_t6 = am*0.5*( -t1 + t2 + t3 + t4 + t5 - 2*t6 );

x_t6 = am*0.5*( 0.5*t1^2 - 2*(t1/pi)^2                                  ...
     +          (t2 - t1)*t3 + 0.5*t3^2 - 0.5*t2^2                      ...
     +          ((t32/pi)^2)*( 2 )                                      ...
     +          (-t1 + t2 + t3 )*( t4 - t3 )                            ...
     +          (-t1 + t2 + t3 )*( t5 - t4 )                            ...
     +         -0.5*(t5^2 - t4^2) + t4*(t5 - t4) + ( (t54/pi)^2 )*( 2 ) ...
     +          (-t1 + t2 + t3 + t4 - t5)*(t6 - t5)                     ...
     -          2*( 0.5*(t6^2 - t5^2) - t5*(t6 - t5) ) );
 
return

function [x, w, a] = ComputeEigenSlewPosRateAccl(am, t, t1, t2, t3, t4, t5, t6, t7, x_t1, x_t2, x_t3, x_t4, x_t5, x_t6, w_t1, w_t2, w_t3, w_t4, w_t5, w_t6)

t21 = t2 - t1;
t32 = t3 - t2;
t43 = t4 - t3;
t54 = t5 - t4;
t65 = t6 - t5;
t76 = t7 - t6;

if t >= 0 && t < t1
    
    a = am*0.5*( 1 - cos( pi*t/t1) );
    
    w = am*0.5*( t - (t1/pi)*sin( pi*t/t1) );
    
    x = am*0.5*( 0.5*t^2 + ((t1/pi)^2)*( cos(pi*t/t1) - 1 ) );
    
elseif t >= t1 && t < t2
    
    a = am;
    
    w = w_t1 + am*t;
    
    x = x_t1 + am*0.5*( -t1*t + t^2 );
    
elseif t >= t2 && t < t3
    
    a = am*0.5*( 1 - cos( pi*(t3 - t)/t32 ) );
    
    w = w_t2 + am*0.5*( t - t2 + ( t32/pi )*sin( pi*(t3 - t)/t32 ) );
    
    x = x_t2                                      ...
        + am*0.5*( (-t1 + 2*t2)*(t - t2)          ...
        +          0.5*(t^2 - t2^2) - t2*(t - t2) ...
        +          ((t32/pi)^2)*( 1 + cos( pi*(t3 - t)/t32 ) ) );
    
elseif t >= t3 && t < t4
    
    a = 0;
    
    w = w_t3;
    
    x = x_t3 + am*0.5*( -t1 + t2 + t3 )*( t - t3 );
    
elseif t >= t4 && t < t5
    
    a = -am*0.5*( 1 - cos( pi*(t4 - t)/t54 ) );
    
    w = w_t4 - am*0.5*( t - t4 + ( t54/pi )*sin( pi*(t4 - t)/t54 ) );
    
    x = x_t4                                      ...
        + am*0.5*( (-t1 + t2 + t3 )*( t - t4 )    ...
        -          0.5*(t^2 - t4^2) + t4*(t - t4) ...
        -          ((t54/pi)^2)*( cos( pi*(t4 - t)/t54 ) - 1 ) );
    
elseif t >= t5 && t < t6
    
    a = -am;
    
    w = w_t5 + am*0.5*( -2*(t- t5) );
    
    x = x_t5                                          ...
        + am*0.5*( (-t1 + t2 + t3 + t4 - t5)*(t - t5) ...
        -            2*( 0.5*(t^2 - t5^2) - t5*(t - t5) ) );
    
elseif t >= t6 && t < t7
    
    a = -am*0.5*( 1 - cos( pi*(t7 - t)/t76 ) );
    
    w = w_t6 ...
        + am*0.5*(-t + t6 - ( t76/pi )*sin( pi*(t7 - t)/t76 ) );
    
    x = x_t6                                                 ...
        + am*0.5*( (-t1 + t2 + t3 + t4 + t5 - 2*t6)*(t - t6) ...
        -           0.5*(t^2 - t6^2) + t6*(t - t6)           ...
        -           ((t76/pi)^2)*( cos( pi*(t7 - t)/t76 ) + 1 ) );
    
else
    
    a = 0;
    w = 0;
    x = x_t6                                                  ...
        + am*0.5*( (-t1 + t2 + t3 + t4 + t5 - 2*t6)*(t7 - t6) ...
        -           0.5*(t7^2 - t6^2) + t6*(t7 - t6)          ...
        -           ((t76/pi)^2)*( cos( pi*(t7 - t7)/t76 ) + 1 ) );
    
end
    
return

function [qslew_ItoB, wslew, aslew] = ComputeCmdQuatRateAccel(q_ItoB1, slew_eigen_axis, eigen_angle, eigen_rate, eigen_accl)

q_B1toBi = [slew_eigen_axis*sin(0.5*eigen_angle); cos(0.5*eigen_angle)];  
  
% quat I to B 
qslew_ItoB = util_quat_multiply(q_B1toBi, q_ItoB1);
  
% body rate
wslew = slew_eigen_axis*eigen_rate;
  
% body accel
aslew = slew_eigen_axis*eigen_accl;
 
return

function [eigen_axis, eigen_angle] = ComputeEigenRotation(q)

% enforce 4th element positive
if q(4) < 0
   q = -q; 
end

q_mag = norm(q(1:3));

if q_mag > 1e-8
   eigen_axis = q(1:3)/q_mag;
   if abs(q(4)) > 1 
      q(4) = 1;    
   end
   eigen_angle = 2*acos(q(4));
else
   eigen_axis = [1 0 0]';
   eigen_angle = 0;
end

return