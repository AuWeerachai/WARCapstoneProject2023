function DCM_Z = compute_rot_z(angle_rad)

  cs_angle = cos(angle_rad); 
  sn_angle = sin(angle_rad); 

  DCM_Z = [cs_angle -sn_angle 0; sn_angle cs_angle 0; 0 0 1];
end