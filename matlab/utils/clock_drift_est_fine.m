function cfo_fine= clock_drift_est_fine(cfo_coarse, poa, fc, delta_turn)
% clock_drift_est_fine - Conduct fine-grained clock drift estimation
%                        using the phase estimates of a single channel
%
% Inputs:
%   cfo_coarse: Clock drift estimates using different UWB channels
%   poa_ch1: Phase estimates
%   fc1: Center frequency
%   delta_turn: Time interval between two consecutive rounds of localization.
%
% Output:
%   cfo_fine: Fine-grained clock drift estimates.

% The clock drift estimate is wrapped by 1/(fc*delta_turn), as detailed in Section 3.3 of our paper.
wrap_factor = 1/fc/delta_turn*1e6;

cfo_fine = zeros(size(cfo_coarse));

% The phase differences are wrapped to the range [-pi, pi].
phase_diff =  wrapToPi(poa(:,2:2:end)-poa(:,1:2:end));

% Determine the ambiguity number using coarse-grained estimates
l = floor(cfo_coarse(:,1:2:end)/wrap_factor);

% Caculate fine-grained clock drift
temp = (phase_diff/2/pi+l(1:end)).*wrap_factor;
% If the difference is less than half of the wrap factor, add the wrap factor.
temp(temp-cfo_coarse(1:2:end) < wrap_factor/2) = temp(temp-cfo_coarse(1:2:end) < wrap_factor/2) +  wrap_factor;
% If the difference is greater than half of the wrap factor, subtract the wrap factor.
temp(temp-cfo_coarse(1:2:end) > wrap_factor/2) = temp(temp-cfo_coarse(1:2:end) > wrap_factor/2) -  wrap_factor;

cfo_fine(:,1:2:end) = temp; 
cfo_fine(:,2:2:end) = temp; 

end