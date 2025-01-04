function DCM_X = compute_rot_x(angle_rad)

  cs_angle = cos(angle_rad); 
  sn_angle = sin(angle_rad); 

  DCM_X = [1 0 0; 0 cs_angle -sn_angle; 0 sn_angle cs_angle];
end