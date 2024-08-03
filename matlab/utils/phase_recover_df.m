function [double_f1, double_f2] = phase_recover_df(tag,ancs,idx1,idx2,ref,cfo1,cfo2,delta_anc,fc1,fc2,anc_loc,useful_idx)

c = physconst('LightSpeed');

phase1_f1 = tag.poa(idx1,sort([useful_idx,useful_idx+1]));
phase2_f1 = ancs(ref).poa(idx1,sort([useful_idx+1,useful_idx+2]));

phase3_f1 = tag.poa(idx2,sort([useful_idx,useful_idx+1]));
phase4_f1 = ancs(ref).poa(idx2,sort([useful_idx+1,useful_idx+2]));

phase1_f2 = tag.poa(idx1,sort([useful_idx+2,useful_idx+3]));
phase2_f2 = ancs(ref).poa(idx1,sort([useful_idx+3,useful_idx+4]));

phase3_f2 = tag.poa(idx2,sort([useful_idx+2,useful_idx+3]));
phase4_f2 = ancs(ref).poa(idx2,sort([useful_idx+3,useful_idx+4]));

single1_f1 = phase1_f1-phase2_f1;
single2_f1 = phase3_f1-phase4_f1;

single1_f2 = phase1_f2-phase2_f2;
single2_f2 = phase3_f2-phase4_f2;

cali_f1 = (2*pi*fc1*cfo1(ref,:)*1e-6*abs(idx1-idx2)*delta_anc)';
cali_f2 = (2*pi*fc2*cfo2(ref,:)*1e-6*abs(idx1-idx2)*delta_anc)';

double_f1 = wrapToPi(single1_f1-single2_f1-1*cali_f1);
double_f2 = wrapToPi(single1_f2-single2_f2-1*cali_f2);

dis_idx1_ref = norm(anc_loc(:,idx1)-anc_loc(:,ref));
dis_idx2_ref = norm(anc_loc(:,idx2)-anc_loc(:,ref));

double_f1 = double_f1 - wrapToPi(dis_idx1_ref/(c/fc1/100)*2*pi) + wrapToPi(dis_idx2_ref/(c/fc1/100)*2*pi);
double_f2 = double_f2 - wrapToPi(dis_idx1_ref/(c/fc2/100)*2*pi) + wrapToPi(dis_idx2_ref/(c/fc2/100)*2*pi);

double_f1 = wrapToPi(double_f1);
double_f2 = wrapToPi(double_f2);

end