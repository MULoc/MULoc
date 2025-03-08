function DD = phase_recover(tag,ancs,anc_1,anc_2,anc_ref,cfo,delta_anc,fc,anc_loc,idx)
% phase_recover - Perform UWB signal recovery for phase estimates
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
%   anc_loc:     The carrier frequency of UWB signal.
%   idx:         The sample index of the estimates from channel 1 or channel 3.
% 
% Output Parameters:
%   DD:          The Recovered phase estimates

c = physconst('LightSpeed');

phase1 = tag.poa(anc_1,idx);
phase2 = ancs(anc_ref).poa(anc_1,idx+1);

phase3 = tag.poa(anc_2,idx);
phase4 = ancs(anc_ref).poa(anc_2,idx+1);

% Single difference
SD1 = phase1-phase2;
SD2 = phase3-phase4;

% Residual error
clock_drift_cali = 2*pi*fc*cfo(anc_ref,:)*1e-6*abs(anc_1-anc_2)*delta_anc;

% Double difference / Residual error cancellation
DD = wrapToPi(SD1 - SD2 + clock_drift_cali);

% Compensate for tau_i,ref and tau_j,ref
dis_anc1_ref = norm(anc_loc(:,anc_1)-anc_loc(:,anc_ref));
dis_anc2_ref = norm(anc_loc(:,anc_2)-anc_loc(:,anc_ref));

DD = DD - wrapToPi(dis_anc1_ref/(c/fc/100)*2*pi) + wrapToPi(dis_anc2_ref/(c/fc/100)*2*pi);

DD = wrapToPi(DD);

end