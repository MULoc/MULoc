clear all; close all
%% Data reading
anchor_num = 4;
filename = 'location2_2';
path = './sample_data/';

addpath("./utils");

% Anchors
for i = 1:anchor_num
    anc(i) = read_device_dw1000(path,filename,['anchor',num2str(i)],anchor_num,0);
end

% Tag
tag = read_device_dw1000(path,filename,['tag',num2str(1)],anchor_num,1);

% Anchor locations
[anc_loc,start_loc,end_loc] = read_gt(path,filename,anchor_num);

%% Parameter configuration

% Light speed
c = physconst('LightSpeed');

% UWB Ch.1 (3.5 GHz)
fc1 = 3484.0e6;             % Center Frequency
lambda_ch1 = c/fc1*100;        % Wavelength

% UWB Ch.3 (4.5 GHz)
fc3 = 4484.8e6;             % Center Frequency
lambda_ch3 = c/fc3*100;        % Wavelength

% Time resolution of DW1000
dw_time = 17.2/2^40;

% Time interval between two UWB packets
delta_anc = 615.6464e-6; % 615.65us

% Time interval between two rounds of localization
delta_turn = (delta_anc*anchor_num);
%% Seperating UWB packets captured over different channels

% The DW1000 perform frequency hopping every 2 rounds of localization.
% This results in a repeating pattern where:
% - Rounds 1-2: UWB packets captured over Channel 1
% - Rounds 3-4: UWB packets captured over Channel 3
% The cycle repeats every 4 localization rounds.
idx_temp = mod(tag.idx,4);

% Initialize an empty array to store the useful indices
useful_idx = [];

% Filter out those indices exists packet loss
for  i = 1:length(idx_temp)-4

    % The pattern should be 0, 1, 2, 3, 0 which indicates no packet loss in the cycle
    if (idx_temp(i)==0) && (idx_temp(i+1)==1) && (idx_temp(i+2)==2) && (idx_temp(i+3)==3) && (idx_temp(i+4)==0)
        
        % If the pattern is correct, add the current index i to the useful_idx array
        useful_idx = [useful_idx,i];
    end
end

ch1_idx = sort([useful_idx,useful_idx+1]);
ch3_idx = sort([useful_idx+2,useful_idx+3]);

%% Clock drift estimation

% Original clock drift estimates reported by DW1000 module
cfo_dw1000_ch1 = tag.cfo(:,ch1_idx)*(1.0e6/fc1);
cfo_dw1000_ch3 = tag.cfo(:,ch3_idx)*(1.0e6/fc3);

% Take average of the two estimates for smoothing
cfo_dw1000 = (cfo_dw1000_ch1+cfo_dw1000_ch3)/2;

% Phase estimates of UWB tag
tag_poa_ch1 = tag.poa(:,ch1_idx);
tag_poa_ch3 = tag.poa(:,ch3_idx);

cfo_coarse = zeros(size(cfo_dw1000));
cfo_fine = zeros(size(cfo_dw1000));

for i = 1:anchor_num
    
    cfo_dw1000(i,:) = smooth(cfo_dw1000(i,:),10);

    % Coarse-grained clock drift estimation using two UWB channels
    cfo_coarse(i,:) = clock_drift_est_coarse(cfo_dw1000(i,:), tag_poa_ch1(i,:), tag_poa_ch3(i,:), fc1, fc3, delta_turn);
    cfo_coarse(i,:) = smooth(cfo_coarse(i,:),10);

    % Fine-grained clock drift estimation using a single UWB channel
    cfo_fine_ch1 = clock_drift_est_fine(cfo_coarse(i,:), tag_poa_ch1(i,:), fc1, delta_turn);
    cfo_fine_ch3 = clock_drift_est_fine(cfo_coarse(i,:), tag_poa_ch3(i,:), fc3, delta_turn);

    % Channel 1 and 3 should exhibit identical clock drifts, we can thus
    % take an average of them to reduce random noise
    cfo_fine(i,:) = (cfo_fine_ch1 + cfo_fine_ch3)/2;

end

% Remove outlier samples
for i = 1:anchor_num-1
    for j = 1:5
        temp = diff(cfo_fine(i,:));
        cfo_fine(i,abs(temp)>0.01) = cfo_fine(i,find(abs(temp)>0.01)+5-j);
    end
end

for i = 1:anchor_num
    cfo_fine(i,:) = smooth(cfo_fine(i,:),20);
end

% Debug
% for i = 1:anchor_num
%     figure;
%     plot(cfo_dw1000(i,:));hold on
%     plot(cfo_coarse(i,:));hold on
%     plot(cfo_fine(i,:));hold on
% end

%% Phase recovery

phase_rec_ch1 = zeros(anchor_num-1,length(useful_idx)*2);
phase_rec_ch3 = zeros(anchor_num-1,length(useful_idx)*2);

cali_ch1 = [-0.2,-0.6,-2.2];
cali_ch3 = [0.7,1.55,-1.2];

% Perform UWB signal recovery as detailed in Section 3

% Double difference between anchor 1 and 2, using anchor 3 as the reference
phase_rec_ch1(1,:) = phase_recover(tag,anc,1,2,3,cfo_fine,delta_anc,fc1,anc_loc,ch1_idx)+cali_ch1(1);
phase_rec_ch3(1,:) = phase_recover(tag,anc,1,2,3,cfo_fine,delta_anc,fc3,anc_loc,ch3_idx)+cali_ch3(1);

% Double difference between anchor 1 and 3, using anchor 2 as the reference
phase_rec_ch1(2,:) = phase_recover(tag,anc,1,3,2,cfo_fine,delta_anc,fc1,anc_loc,ch1_idx)+cali_ch1(2);
phase_rec_ch3(2,:) = phase_recover(tag,anc,1,3,2,cfo_fine,delta_anc,fc3,anc_loc,ch3_idx)+cali_ch3(2);

% Double difference between anchor 1 and 4, using anchor 2 as the reference
phase_rec_ch1(3,:) = phase_recover(tag,anc,1,4,2,cfo_fine,delta_anc,fc1,anc_loc,ch1_idx)+cali_ch1(3);
phase_rec_ch3(3,:) = phase_recover(tag,anc,1,4,2,cfo_fine,delta_anc,fc3,anc_loc,ch3_idx)+cali_ch3(3);

% Remove outlier samples
for i = 1:anchor_num-1
    for j = 1:5
        temp = diff(unwrap(phase_rec_ch1(i,:)));
        phase_rec_ch1(i,abs(temp)>1) = phase_rec_ch1(i,find(abs(temp)>1)+1-j);

        temp = diff(unwrap(phase_rec_ch3(i,:)));
        phase_rec_ch3(i,abs(temp)>1) = phase_rec_ch3(i,find(abs(temp)>1)+1-j);
    end
end

% Debug
% for i = 1:anchor_num-1
%     figure;
%     plot(unwrap(phase_rec_ch1(i,:)));hold on
%     plot(unwrap(phase_rec_ch3(i,:)));hold on
% end

%% ToF recovery

tof_rec_ch1 = zeros(anchor_num-1,length(useful_idx)*2);
tof_rec_ch3 = zeros(anchor_num-1,length(useful_idx)*2);

% Parameters for Linear calibration 
k_ch1 = [1.02,1.01,1.05];
b_ch1 = [4,4,0];

k_ch3 = [1.02,1.01,1.03];
b_ch3 = [4,4,0];

% Perform UWB signal recovery as detailed in Section 3

% Double difference between anchor 1 and 2, using anchor 3 as the reference
tof_rec_ch1(1,:) = tof_recover(tag,anc,1,2,3,cfo_fine,delta_anc,anc_loc,ch1_idx)/k_ch1(1)+b_ch1(1);
tof_rec_ch3(1,:) = tof_recover(tag,anc,1,2,3,cfo_fine,delta_anc,anc_loc,ch3_idx)/k_ch3(1)+b_ch3(1);

% Double difference between anchor 1 and 3, using anchor 2 as the reference
tof_rec_ch1(2,:) = tof_recover(tag,anc,1,3,2,cfo_fine,delta_anc,anc_loc,ch1_idx)/k_ch1(2)+b_ch1(2);
tof_rec_ch3(2,:) = tof_recover(tag,anc,1,3,2,cfo_fine,delta_anc,anc_loc,ch3_idx)/k_ch3(2)+b_ch3(2);

% Double difference between anchor 1 and 4, using anchor 2 as the reference
tof_rec_ch1(3,:) = tof_recover(tag,anc,1,4,2,cfo_fine,delta_anc,anc_loc,ch1_idx)/k_ch1(3)+b_ch1(3);
tof_rec_ch3(3,:) = tof_recover(tag,anc,1,4,2,cfo_fine,delta_anc,anc_loc,ch3_idx)/k_ch3(3)+b_ch3(3);

% Take average of the two tof results for smoothing
tof_rec  = (tof_rec_ch1+tof_rec_ch3)/2;

% Refine tof_rec through TL-LS based localization
loc_result = tdoa_solver(-tof_rec', anc_loc*100, 'Chan', 4, [1,1,1]);

tof_rec(1,:) = vecnorm(loc_result-anc_loc(:,1)'*100,2,2)-vecnorm(loc_result-anc_loc(:,2)'*100,2,2);
tof_rec(2,:) = vecnorm(loc_result-anc_loc(:,1)'*100,2,2)-vecnorm(loc_result-anc_loc(:,3)'*100,2,2);
tof_rec(3,:) = vecnorm(loc_result-anc_loc(:,1)'*100,2,2)-vecnorm(loc_result-anc_loc(:,4)'*100,2,2);

% Debug
% plot(tof_rec');hold on

%% Noise Redection for ToF-based measurements

% Take average of the first 50 samples to avoid estimation bias 
tof_rec_sm = zeros(size(tof_rec));
tof_rec_sm(:,1) = mean(tof_rec(:,1:50),2);

sm_factor = 0.7;

double_pdoa_unwrap = zeros(size(tof_rec_sm));

for i = 1:anchor_num-1
    double_pdoa_unwrap(i,:) = unwrap(phase_rec_ch3(i,:))/2/pi*lambda_ch3;
end

% Equation 18
for i = 1:length(tof_rec_sm(1,:))-1
    tof_rec_sm(:,i+1) = tof_rec_sm(:,i) + (1-sm_factor)*(tof_rec(:,i+1)-tof_rec(:,i)) + sm_factor*(-double_pdoa_unwrap(:,i+1) + double_pdoa_unwrap(:,i) );
end

%% Enlarging ambiguity cycle with frequency hopping

% For more details on our ambiguity resolution method, please refer to our paper 
% "Push the Limit of Highly Accurate Ranging on Commercial UWB Devices" (UbiComp'24)

% Phase difference between channel 1 and 3
phase_diff = wrapToPi(phase_rec_ch3 - phase_rec_ch1);

% New ambiguity cycle = 30 cm
lambda_new = (lambda_ch1*lambda_ch3/(lambda_ch1-lambda_ch3));

% Coarse-grained distance difference
N_diff = floor(tof_rec_sm/lambda_new);
d_phase_diff = (-phase_diff/2/pi+N_diff).*lambda_new;

for i = 1:anchor_num-1
    d_phase_diff(i,(d_phase_diff(i,:)-tof_rec_sm(i,:))<lambda_new/2) = d_phase_diff(i,(d_phase_diff(i,:)-tof_rec_sm(i,:))<lambda_new/2) + lambda_new;
    d_phase_diff(i,(d_phase_diff(i,:)-tof_rec_sm(i,:))>lambda_new/2) = d_phase_diff(i,(d_phase_diff(i,:)-tof_rec_sm(i,:))>lambda_new/2) - lambda_new;
    d_phase_diff(i,:) = smooth(d_phase_diff(i,:),10);
end

% Fine-grained distance difference estimation using channel 3
N_ch1 = floor(d_phase_diff/lambda_ch1);
d_phase_ch1 = (-phase_rec_ch1/2/pi+N_ch1).*lambda_ch1;

for i = 1:anchor_num-1
    d_phase_ch1(i,(d_phase_ch1(i,:)-d_phase_diff(i,:))<lambda_ch1/2) = d_phase_ch1(i,(d_phase_ch1(i,:)-d_phase_diff(i,:))<lambda_ch1/2) + lambda_ch1;
    d_phase_ch1(i,(d_phase_ch1(i,:)-d_phase_diff(i,:))>lambda_ch1/2) = d_phase_ch1(i,(d_phase_ch1(i,:)-d_phase_diff(i,:))>lambda_ch1/2) - lambda_ch1;
end

% Fine-grained distance difference estimation using channel 3
N_ch3 = floor(d_phase_diff/lambda_ch3);
d_phase_ch3 = (-phase_rec_ch3/2/pi+N_ch3).*lambda_ch3;

for i = 1:anchor_num-1
    d_phase_ch3(i,(d_phase_ch3(i,:)-d_phase_diff(i,:))<lambda_ch3/2) = d_phase_ch3(i,(d_phase_ch3(i,:)-d_phase_diff(i,:))<lambda_ch3/2) + lambda_ch3;
    d_phase_ch3(i,(d_phase_ch3(i,:)-d_phase_diff(i,:))>lambda_ch3/2) = d_phase_ch3(i,(d_phase_ch3(i,:)-d_phase_diff(i,:))>lambda_ch3/2) - lambda_ch3;
end

% Take average of the two results for smoothing
d_phase = (d_phase_ch3+d_phase_ch1)/2;

% Debug
% plot(tof_rec_sm');hold on
% plot(d_phase_diff');hold on
% plot(d_phase_f1');hold on
% plot(d_phase_f2');hold on


%% Localization using chan's method

% Localization using chan's method
loc_result = tdoa_solver(-d_phase(1:3,:)', anc_loc*100, 'Chan', 3, 1);

gt_loc = [linspace(start_loc(1),end_loc(1),6);linspace(start_loc(2),end_loc(2),6)];

figure
plot(loc_result(:,1),loc_result(:,2),'b');hold on               % Our result
scatter(gt_loc(1,:)*100,gt_loc(2,:)*100,'red','filled')         % Ground truth

scatter(anc_loc(1,:)*100,anc_loc(2,:)*100,100,'filled','d','MarkerEdgeColor',[0.41,0.59,1],'MarkerFaceColor',[0.41,0.59,1]);hold on

xlabel('X (cm)')
ylabel('Y (cm)')

axis equal

legend('MULoc result','Ground truth','Anchor')
set(gca,'linewidth',1,'fontsize',26);
set(gcf,'Position',[100 100 950 580]);