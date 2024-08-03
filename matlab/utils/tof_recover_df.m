function [double_tof_dis_f1, double_tof_dis_f2] = tof_recover_df(tag,ancs,idx1,idx2,ref,cfo1,cfo2,delta_anc,anc_loc,useful_idx)

dw_time = 17.2/2^40;

time1_f1 = tag.rx_times(idx1,sort([useful_idx,useful_idx+1]));
time2_f1 = ancs(ref).rx_times(idx1,sort([useful_idx+1,useful_idx+2]));

time3_f1 = tag.rx_times(idx2,sort([useful_idx,useful_idx+1]));
time4_f1 = ancs(ref).rx_times(idx2,sort([useful_idx+1,useful_idx+2]));

time1_f2 = tag.rx_times(idx1,sort([useful_idx+2,useful_idx+3]));
time2_f2 = ancs(ref).rx_times(idx1,sort([useful_idx+3,useful_idx+4]));

time3_f2 = tag.rx_times(idx2,sort([useful_idx+2,useful_idx+3]));
time4_f2 = ancs(ref).rx_times(idx2,sort([useful_idx+3,useful_idx+4]));

single_tof1_f1 = time1_f1-time2_f1;
single_tof2_f1 = time3_f1-time4_f1;

single_tof1_f2 = time1_f2-time2_f2;
single_tof2_f2 = time3_f2-time4_f2;

cali_f1 = (abs(idx1-idx2)*delta_anc*cfo1(ref,:)*1e-6)';
cali_f2 = (abs(idx1-idx2)*delta_anc*cfo2(ref,:)*1e-6)';

double_tof_f1 = ((single_tof1_f1 - single_tof2_f1)*dw_time + 1*cali_f1)*3e8;
double_tof_f2 = ((single_tof1_f2 - single_tof2_f2)*dw_time + 1*cali_f2)*3e8;

dis_idx1_ref = norm(anc_loc(:,idx1)-anc_loc(:,ref));
dis_idx2_ref = norm(anc_loc(:,idx2)-anc_loc(:,ref));

double_tof_f1 = double_tof_f1 + dis_idx1_ref - dis_idx2_ref;
double_tof_f2 = double_tof_f2 + dis_idx1_ref - dis_idx2_ref;

double_tof_dis_f1 = double_tof_f1*100;
double_tof_dis_f2 = double_tof_f2*100;

for i = 1:length(double_tof_dis_f1)-1
    if abs(double_tof_dis_f1(i+1)-double_tof_dis_f1(i)) > 20
        double_tof_dis_f1(i+1) = double_tof_dis_f1(i);
    end
end

for i = 1:length(double_tof_dis_f2)-1
    if abs(double_tof_dis_f2(i+1)-double_tof_dis_f2(i)) > 20
        double_tof_dis_f2(i+1) = double_tof_dis_f2(i);
    end
end

end
