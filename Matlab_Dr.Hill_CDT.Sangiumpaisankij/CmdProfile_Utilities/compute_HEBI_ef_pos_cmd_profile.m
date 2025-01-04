function [err_flag_xyz_out, ef_cmds] = compute_HEBI_ef_pos_cmd_profile(t, CMD_STRUCT)

%coder.extrinsic('fprintf')

persistent curr_slew_strt_time 
persistent curr_slew_stop_time

persistent ef_cmds_prev

persistent err_flag_xyz

persistent init_flag 
persistent i_tg
persistent slew_strt_time

persistent   t1   t2   t3   t4   t5   t6   t7
persistent x_t1 x_t2 x_t3 x_t4 x_t5 x_t6 
persistent w_t1 w_t2 w_t3 w_t4 w_t5 w_t6

persistent u_slew_dir
persistent xyz_accl_limit_mps2
persistent x_pos 
persistent y_pos
persistent z_pos

% initialize error flag
err_flag_xyz = 0;

% reserve storage for outputs
ef_cmds = zeros(9,1);

% initialization
if isempty(init_flag)
   
   curr_slew_strt_time = 0;
   
   curr_slew_stop_time = 0;
   
   ef_cmds_prev = [CMD_STRUCT.Init_x_pos; CMD_STRUCT.Init_y_pos; CMD_STRUCT.Init_z_pos];
   
   init_flag = 0;
      
   i_tg = 0; 
   
   x_pos = 0; y_pos = 0; z_pos = 0;
   
   slew_strt_time = 0;

   t1 = 0; t2 = 0; t3 = 0; t4 = 0; t5 = 0; t6 = 0; t7 = 0;
   
   x_t1 = 0; x_t2 = 0; x_t3 = 0; x_t4 = 0; x_t5 = 0; x_t6 = 0;
   w_t1 = 0; w_t2 = 0; w_t3 = 0; w_t4 = 0; w_t5 = 0; w_t6 = 0;
   
   xyz_accl_limit_mps2 = 0;
   
end

% if time is greater than start time and targets remaining then plan slew
if t >= slew_strt_time && i_tg < CMD_STRUCT.n_tg
       
   % next target pointer
   i_tg = i_tg + 1;
    
   % slew start position
   x_pos = ef_cmds_prev(1); y_pos = ef_cmds_prev(2); z_pos = ef_cmds_prev(3); d_pos_strt = 0;
   
   % slew stop position
   dx = CMD_STRUCT.x_pos(i_tg) - x_pos;
   
   dy = CMD_STRUCT.y_pos(i_tg) - y_pos;

   dz = CMD_STRUCT.z_pos(i_tg) - z_pos;

   % slew unit direction vector
   u_slew_dir = [dx; dy; dz] / norm([dx; dy; dz]);
  
   % slew distance
   d_pos_stop = sqrt(dx*dx + dy*dy + dz*dz);
   
   % slew duration
   slew_duration_sec = CMD_STRUCT.strt_time(i_tg) - slew_strt_time;
   
   % x-y position slew parameters
   if CMD_STRUCT.x_pos(i_tg) ~= x_pos || CMD_STRUCT.y_pos(i_tg) ~= y_pos || CMD_STRUCT.z_pos(i_tg) ~= z_pos
       
       [err_flag_xyz, xyz_tJ, xyz_tR, xyz_tC, xyz_accl_limit_mps2] = ...
                                                                ...
       ComputeSlewParameters(d_pos_strt, d_pos_stop, CMD_STRUCT.xyz_accl_limit_mps2, CMD_STRUCT.min_slew_dur_sec, slew_duration_sec, CMD_STRUCT.tJ_pct);
   
       % if error then use 100 percent jerk versine profile
       if err_flag_xyz == 1 || err_flag_xyz == 2
           
          xyz_tJ = 0.25 * slew_duration_sec;
          xyz_tR = 0;
          xyz_tC = 0;
          slew_distance = d_pos_stop - d_pos_strt;
          t1x = xyz_tJ; t2x = xyz_tJ; t3x = 2*xyz_tJ; t32 = t3x - t2x; t4x = t3x; t5x = 3*xyz_tJ; t54 = t5x - t4x; t6x = t5x; t7x = 4*xyz_tJ; t76 = t7x - t6x;
          
          term_x_t6 = 0.5*( 0.5*t1x^2 - 2*(t1x/pi)^2                                     ...
                    +     (t2x - t1x)*t3x + 0.5*t3x^2 - 0.5*t2x^2                        ...
                    +     ((t32/pi)^2)*( 2 )                                             ...
                    +     (-t1x + t2x + t3x )*( t4x - t3x )                              ...
                    +     (-t1x + t2x + t3x )*( t5x - t4x )                              ...
                    +      -0.5*(t5x^2 - t4x^2) + t4x*(t5x - t4x) + ( (t54/pi)^2 )*( 2 ) ...
                    +     (-t1x + t2x + t3x + t4x - t5x)*(t6x - t5x)                     ...
                    -       2*( 0.5*(t6x^2 - t5x^2) - t5x*(t6x - t5x) ) );
              
          term_x_t7 = 0.5*( (-t1x + t2x + t3x + t4x + t5x - 2*t6x)*(t7x - t6x) ...
                   -        0.5*(t7x^2 - t6x^2) + t6x*(t7x - t6x)           ...
                   -        ((t76/pi)^2)*( cos( pi*(t7x - t7x)/t76 ) + 1 ) );
          
          xyz_accl_limit_mps2 = slew_distance / (term_x_t6 + term_x_t7);
               
          err_flag_xyz = 0;
            
       end
   
   else
       xyz_tJ = 0.0; xyz_tR = 0.0; xyz_tC = 0.0; 
   end
   
   % if either err_flag is nonzero then hold previous state
   if err_flag_xyz > 0
       
       ef_cmds_prev = [x_pos; y_pos; z_pos; zeros(3,1); zeros(3,1)];
       
       if err_flag_xyz > 0
           err_flag = err_flag_xyz;
           fprintf('\n Command Generation Error:  err_flag_xyz = %d \n', err_flag);
           return
       end
       
   else
       
       if CMD_STRUCT.x_pos(i_tg) ~= x_pos || CMD_STRUCT.y_pos(i_tg) ~= y_pos || CMD_STRUCT.z_pos(i_tg) ~= z_pos
            
           t1 = xyz_tJ;
           t2 = xyz_tJ   + xyz_tR;
           t3 = 2*xyz_tJ + xyz_tR;
           t4 = 2*xyz_tJ + xyz_tR + xyz_tC;
           t5 = 3*xyz_tJ + xyz_tR + xyz_tC;
           t6 = 3*xyz_tJ + 2*xyz_tR + xyz_tC;
           t7 = 4*xyz_tJ + 2*xyz_tR + xyz_tC;
           
           tp = [t1 t2 t3 t4 t5 t6 t7];
           
           [x_t1, x_t2, x_t3, x_t4, x_t5, x_t6, w_t1, w_t2, w_t3, w_t4, w_t5, w_t6] = ComputeVersineParams(xyz_accl_limit_mps2, tp);
       
       end
       
       
   end
   
   % current slew strt time
   curr_slew_strt_time = slew_strt_time;
   
   % current slew stop time
   curr_slew_stop_time =  CMD_STRUCT.strt_time(i_tg);
   
   % next slew start time
   slew_strt_time = CMD_STRUCT.strt_time(i_tg) + CMD_STRUCT.duration(i_tg);
    
end

% if err_flags are zero and time is in slew window then slew
if err_flag_xyz == 0 && t >= curr_slew_strt_time && t <= curr_slew_stop_time
    
   dt = t - curr_slew_strt_time;

   % delta x-y position, rate, and accel
   if CMD_STRUCT.x_pos(i_tg) ~= x_pos || CMD_STRUCT.y_pos(i_tg) ~= y_pos
       
      [d_xyz_pos, d_xyz_rate, d_xyz_accl] = ...
                                            ...
      ComputeSlewPosRateAccl(xyz_accl_limit_mps2, dt, t1, t2, t3, t4, t5, t6, t7, x_t1, x_t2, x_t3, x_t4, x_t5, x_t6, w_t1, w_t2, w_t3, w_t4, w_t5, w_t6);

   else
       
      d_xyz_pos = 0.0; d_xyz_rate = 0.0; d_xyz_accl = 0.0;
       
   end

   % global x-y-z position, rate, and accel
   pos_slew = [x_pos; y_pos; z_pos] +  d_xyz_pos * u_slew_dir;

   rate_slew = d_xyz_rate * u_slew_dir;

   accl_slew = d_xyz_accl * u_slew_dir;
   
   % if slew is done then hold last state
   if dt <= t7 

      ef_cmds(1) = pos_slew(1);
      ef_cmds(2) = pos_slew(2);
      ef_cmds(3) = pos_slew(3);
      ef_cmds(4) = rate_slew(1);
      ef_cmds(5) = rate_slew(2);
      ef_cmds(6) = rate_slew(3);
      ef_cmds(7) = accl_slew(1);
      ef_cmds(8) = accl_slew(2);
      ef_cmds(9) = accl_slew(3);
      
   else
       
      ef_cmds(1) = ef_cmds_prev(1);
      ef_cmds(2) = ef_cmds_prev(2);
      ef_cmds(3) = ef_cmds_prev(3);
      ef_cmds(4) = ef_cmds_prev(4);
      ef_cmds(5) = ef_cmds_prev(5);
      ef_cmds(6) = ef_cmds_prev(6);
      ef_cmds(7) = ef_cmds_prev(7);
      ef_cmds(8) = ef_cmds_prev(8);
      ef_cmds(9) = ef_cmds_prev(9);
      
   end

   % save previous solutions
   ef_cmds_prev = ef_cmds;

% otherwise, hold at previous state
else
   ef_cmds = ef_cmds_prev;
end

err_flag_xyz_out = err_flag_xyz;

return

function [err_flag, tJ, tR, tC, accl_limit] = ComputeSlewParameters(strt_pos, stop_pos, accl_limit, min_slew_dur_sec, slew_dur_sec, tJ_pct)

% initialize outputs
err_flag = 0; 

tJ = 0;
tR = 0; 
tC = 0;

% slew distance or angle
slew_distance = stop_pos - strt_pos;
    
if slew_dur_sec < min_slew_dur_sec
    err_flag = 3;
    return
end

% jerk time
tJ = 0.25*slew_dur_sec*tJ_pct*0.01;

% race and coast time
while 1
    
    if accl_limit < 1e-4
       err_flag = 2;
       return
    end
    
    b = 3*tJ - slew_dur_sec;
    c = 2*tJ^2 - slew_dur_sec*tJ + slew_distance/accl_limit;
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
 
    accl_limit = 0.90*accl_limit;
    
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

function [x, w, a] = ComputeSlewPosRateAccl(am, t, t1, t2, t3, t4, t5, t6, t7, x_t1, x_t2, x_t3, x_t4, x_t5, x_t6, w_t1, w_t2, w_t3, w_t4, w_t5, w_t6)

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