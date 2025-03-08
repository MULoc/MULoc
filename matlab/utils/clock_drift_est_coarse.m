function cfo_coarse= clock_drift_est_coarse(cfo_dw1000, poa_ch1, poa_ch3, fc1, fc3, delta_turn)
% clock_drift_est_coarse -- Conduct coarse-grained clock drift estimation
%                           by leveraging the phase difference of two frequencies
%
% Inputs:
%   cfo_dw1000: Clock drift estimates supplied by the DW1000 module.
%   poa_ch1: Phase estimates obtained from channel 1.
%   poa_ch3: Phase estimates obtained from channel 3.
%   fc1: Center frequency of channel 1.
%   fc3: Center frequency of channel 3.
%   delta_turn: Time interval between two consecutive rounds of localization.
%
% Output:
%   cfo_coarse: Coarse - grained clock drift estimates.

% The clock drift estimate is wrapped by 1/(fc*delta_turn), as detailed in Section 3.3 of our paper.
% Here, we utilize the frequency difference between channel 1 and channel 3 to enlarge the ambiguity cycle.
wrap_factor = 1/(fc3-fc1)/delta_turn*1e6;

cfo_coarse = zeros(size(cfo_dw1000));

% The phase differences are wrapped to the range [-pi, pi].
phase_diff_ch1 =  wrapToPi(poa_ch1(:,2:2:end)-poa_ch1(:,1:2:end));
phase_diff_ch3 =  wrapToPi(poa_ch3(:,2:2:end)-poa_ch3(:,1:2:end));

% The ambiguity of phase_diff3 is enlarged compared with phase_diff_ch1 and
% phase_diff_ch3, which can help determine the ambiguity number 
phase_diff = phase_diff_ch3-phase_diff_ch1;

% Determine the ambiguity number using DW1000 estimates
l = floor(cfo_dw1000(:,1:2:end)/wrap_factor);

% Caculate coarse-grained clock drift
temp = (phase_diff/2/pi+l(1:end)).*wrap_factor;

% If the difference is less than half of the wrap factor, add the wrap factor.
temp(temp-cfo_dw1000(1:2:end) < wrap_factor/2) = temp(temp-cfo_dw1000(1:2:end) < wrap_factor/2) +  wrap_factor;
% If the difference is greater than half of the wrap factor, subtract the wrap factor.
temp(temp-cfo_dw1000(1:2:end) > wrap_factor/2) = temp(temp-cfo_dw1000(1:2:end) > wrap_factor/2) -  wrap_factor;

cfo_coarse(:,1:2:end) = temp; 
cfo_coarse(:,2:2:end) = temp; 

end