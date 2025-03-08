function DD = tof_recover(tag,ancs,anc_1,anc_2,ref,cfo,delta_anc,anc_loc,idx)
% tof_recover - Perform UWB signal recovery for ToF estimates
% 
% Input Parameters:
%   tag:         UWB tag
%   ancs:        UWB anchors
%   anc_1:       The index of the first anchor
%   anc_2:       The index of the second anchor
%   ref:         The index of the reference anchor.
%   cfo:         Clock drift
%   delta_anc:   The time interval between two consecutive UWB packets. 
%   anc_loc:     The locations of all UWB anchors in the coordinate system.
%   idx:         The sample index of the estimates from channel 1 or channel 3.
% 
% Output Parameters:
%   DD:          The Recovered ToF estimates

dw_time = 17.2/2^40;

time1 = tag.rx_times(anc_1,idx);
time2 = ancs(ref).rx_times(anc_1,idx+1);

time3 = tag.rx_times(anc_2,idx);
time4 = ancs(ref).rx_times(anc_2,idx+1);

% Single difference
SD1 = time1-time2;
SD2 = time3-time4;

% Residual error
clock_drift_cali = (abs(anc_1-anc_2)*delta_anc*cfo(ref,:)*1e-6);

% Double difference / Residual error cancellation
DD = ((SD1 - SD2)*dw_time - 1*clock_drift_cali)*3e8;

% Compensate for tau_i,ref and tau_j,ref
dis_anc1_ref = norm(anc_loc(:,anc_1)-anc_loc(:,ref));
dis_anc2_ref = norm(anc_loc(:,anc_2)-anc_loc(:,ref));

DD = DD + dis_anc1_ref - dis_anc2_ref;

DD = DD*100;

for i = 1:length(DD)-1
    if abs(DD(i+1)-DD(i)) > 30
        DD(i+1) = DD(i);
    end
end

end
