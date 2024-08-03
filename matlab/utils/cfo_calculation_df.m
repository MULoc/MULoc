function cfo_fine= cfo_calculation_df(cfo_coarse, poa, wrap_factor, Q, R, P0)


cfo_coarse = kalman_filter(cfo_coarse,Q,R,cfo_coarse(1),P0);

cfo_fine = zeros(size(cfo_coarse));

phase_diff =  wrapToPi(poa(:,2:2:end)-poa(:,1:2:end));
k = floor(cfo_coarse(:,1:2:end)/wrap_factor);
temp = (phase_diff/2/pi+k(1:end)).*wrap_factor;
temp(temp-cfo_coarse(1:2:end) < wrap_factor/2) = temp(temp-cfo_coarse(1:2:end) < wrap_factor/2) +  wrap_factor;
temp(temp-cfo_coarse(1:2:end) > wrap_factor/2) = temp(temp-cfo_coarse(1:2:end) > wrap_factor/2) -  wrap_factor;

cfo_fine(:,1:2:end) = temp; 
cfo_fine(:,2:2:end) = temp; 

end