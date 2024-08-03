clear all; close all
%% Data reading
anchor_num = 4;
filename = 'location_1';
path = './';

% Anchors
for i = 1:anchor_num
    ancs(i) = read_device_dw1000_df(path,filename,['anchor',num2str(i)],anchor_num,0);
end

% Tag
tag = read_device_dw1000_df(path,filename,['tag',num2str(1)],anchor_num,1);

anc_loc = read_gt(path,filename,anchor_num);

%% Parameters
c = physconst('LightSpeed');

% channel1
fc1 = 3484.0e6;
lambda1 = c/fc1*100;

% channel2
fc2 = 4484.8e6;
lambda2 = c/fc2*100;

dw_time = 17.2/2^40;

delta_anc = 600e-6; % 600us
delta_turn = (delta_anc*anchor_num);
%% Fequency clustering

idx_temp = mod(tag.idx,4);
useful_idx = [];

for  i = 1:length(idx_temp)-4
    if (idx_temp(i)==0) && (idx_temp(i+1)==1) && (idx_temp(i+2)==2) && (idx_temp(i+3)==3) && (idx_temp(i+4)==0)
        useful_idx = [useful_idx,i];
    end
end

%% Clock drift estimation

wrap_factor_f1 = 1/3494e6/delta_turn*1e6;
wrap_factor_f2 = 1/4492e6/delta_turn*1e6;

cfo = [];

cfo_temp_f1 = tag.cfo(:,sort([useful_idx,useful_idx+1]))*(1.0e6/3494.4e6);
cfo_temp_f2 = tag.cfo(:,sort([useful_idx+2,useful_idx+3]))*(1.0e6/4492.8e6);

cfo_temp = (cfo_temp_f1+cfo_temp_f2)/2;

tag_poa_f1 = tag.poa(:,sort([useful_idx,useful_idx+1]));
tag_poa_f2 = tag.poa(:,sort([useful_idx+2,useful_idx+3]));

cfo_f1 = [];
cfo_f2 = [];

for i = 1:anchor_num
    temp1 = cfo_calculation_df(cfo_temp(i,:), tag_poa_f1(i,:), wrap_factor_f1, 0.1, 1, 0.1);
    temp2 = cfo_calculation_df(cfo_temp(i,:), tag_poa_f2(i,:), wrap_factor_f2, 0.1, 1, 0.1);
    
    cfo_f1 = [cfo_f1; temp1];
    cfo_f2 = [cfo_f2; temp2];

end

%% Phase recovery

Phase_rec_f1 = zeros(anchor_num,length(useful_idx)*2);
Phase_rec_f2 = zeros(anchor_num,length(useful_idx)*2);

[Phase_rec_f1(1,:),Phase_rec_f2(1,:)] = phase_recover_df(tag,ancs,1,2,3,cfo_f1,cfo_f2,delta_anc,fc1,fc2,anc_loc,useful_idx);
[Phase_rec_f1(2,:),Phase_rec_f2(2,:)] = phase_recover_df(tag,ancs,1,3,2,cfo_f1,cfo_f2,delta_anc,fc1,fc2,anc_loc,useful_idx);
[Phase_rec_f1(3,:),Phase_rec_f2(3,:)] = phase_recover_df(tag,ancs,1,4,2,cfo_f1,cfo_f2,delta_anc,fc1,fc2,anc_loc,useful_idx);

Phase_rec_f1(1,:) = wrapToPi(Phase_rec_f1(1,:));
Phase_rec_f1(2,:) = wrapToPi(Phase_rec_f1(2,:));
Phase_rec_f1(3,:) = wrapToPi(Phase_rec_f1(3,:));

Phase_rec_f2(1,:) = wrapToPi(Phase_rec_f2(1,:));
Phase_rec_f2(2,:) = wrapToPi(Phase_rec_f2(2,:));
Phase_rec_f2(3,:) = wrapToPi(Phase_rec_f2(3,:));

%% ToF recovery

ToF_rec_f1 = zeros(anchor_num,length(useful_idx)*2);
ToF_rec_f2 = zeros(anchor_num,length(useful_idx)*2);

[ToF_rec_f1(1,:),ToF_rec_f2(1,:)] = tof_recover_df(tag,ancs,1,2,3,cfo_f1,cfo_f2,delta_anc,anc_loc,useful_idx);
[ToF_rec_f1(2,:),ToF_rec_f2(2,:)] = tof_recover_df(tag,ancs,1,3,2,cfo_f1,cfo_f2,delta_anc,anc_loc,useful_idx);
[ToF_rec_f1(3,:),ToF_rec_f2(3,:)] = tof_recover_df(tag,ancs,1,4,2,cfo_f1,cfo_f2,delta_anc,anc_loc,useful_idx);

ToF_rec  = (ToF_rec_f1+ToF_rec_f2)/2;

%% Noise Redection for ToF-based measurements

ToF_rec_sm = zeros(size(ToF_rec));
ToF_rec_sm(:,1) = ToF_rec(:,1);

sm_factor = 0.7;

double_pdoa_unwrap = zeros(size(ToF_rec_sm));
for i = 1:anchor_num
    double_pdoa_unwrap(i,:) = unwrap(Phase_rec_f2(i,:))/2/pi*lambda2;
end

for i = 1:length(ToF_rec_sm(1,:))-1
    ToF_rec_sm(:,i+1) = ToF_rec_sm(:,i) + (1-sm_factor)*(ToF_rec(:,i+1)-ToF_rec(:,i)) + sm_factor*(-double_pdoa_unwrap(:,i+1) + double_pdoa_unwrap(:,i) );
end

%% Enlarging ambiguity cycle with frequency hopping

double_pdoa_new_f1 = Phase_rec_f1;
double_pdoa_new_f2 = Phase_rec_f2;

phase_diff = wrapToPi(Phase_rec_f2 - Phase_rec_f1);

lambda_new = (lambda1*lambda2/(lambda1-lambda2));

N_diff = floor(ToF_rec_sm/lambda_new);
d_phase_diff = (-phase_diff/2/pi+N_diff).*lambda_new;

for i = 1:anchor_num
    d_phase_diff(i,(d_phase_diff(i,:)-ToF_rec_sm(i,:))<lambda_new/2) = d_phase_diff(i,(d_phase_diff(i,:)-ToF_rec_sm(i,:))<lambda_new/2) + lambda_new;
    d_phase_diff(i,(d_phase_diff(i,:)-ToF_rec_sm(i,:))>lambda_new/2) = d_phase_diff(i,(d_phase_diff(i,:)-ToF_rec_sm(i,:))>lambda_new/2) - lambda_new;
end

N_1 = floor(d_phase_diff/lambda1);
d_phase_f1 = (-Phase_rec_f1/2/pi+N_1).*lambda1;

for i = 1:anchor_num
    d_phase_f1(i,(d_phase_f1(i,:)-d_phase_diff(i,:))<lambda1/2) = d_phase_f1(i,(d_phase_f1(i,:)-d_phase_diff(i,:))<lambda1/2) + lambda1;
    d_phase_f1(i,(d_phase_f1(i,:)-d_phase_diff(i,:))>lambda1/2) = d_phase_f1(i,(d_phase_f1(i,:)-d_phase_diff(i,:))>lambda1/2) - lambda1;
end

N_2 = floor(d_phase_diff/lambda2);
d_phase_f2 = (-Phase_rec_f2/2/pi+N_2).*lambda2;

for i = 1:anchor_num
    d_phase_f2(i,(d_phase_f2(i,:)-d_phase_diff(i,:))<lambda2/2) = d_phase_f2(i,(d_phase_f2(i,:)-d_phase_diff(i,:))<lambda2/2) + lambda2;
    d_phase_f2(i,(d_phase_f2(i,:)-d_phase_diff(i,:))>lambda2/2) = d_phase_f2(i,(d_phase_f2(i,:)-d_phase_diff(i,:))>lambda2/2) - lambda2;
end

d_phase = (d_phase_f2+d_phase_f1)/2;

%% Localization using chan's method
loc_result1 = tdoa_solver(-d_phase(1:3,:)', anc_loc*100, 'Chan', 3, [1]);

figure
plot(loc_result1(:,1),loc_result1(:,2),'b');hold on

xlabel('X (cm)')
ylabel('Y (cm)')

axis equal

set(gca,'linewidth',1,'fontsize',26);
set(gcf,'Position',[100 100 950 580]);
