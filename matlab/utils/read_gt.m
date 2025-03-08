function [anc_loc,start_loc,end_loc] = read_gt(path,filename,anc_num)
% read_gt - Reads ground truth positioning data
% 
% Input Parameters:
%   path:       Path to the data file
%   filename:   Name of the data file
%   anc_num:    Number of UWB anchors
% 
% Output Parameters:
%   anc_loc:    Locations of UWB anchors
%   start_loc:  Starting location of the UWB tag
%   end_loc:    Ending location of the UWB tag

anc_loc = zeros(2,anc_num);
start_loc = zeros(2,1);
end_loc = zeros(2,1);

% We use two laser range finders to determine the ground truth locations of 
% UWB anchors and tags. The first row in "anchor.txt" represents the distance 
% between the two laser range finders. Rows 2 - 4 represent the distances from 
% each anchor to the two range finders. Based on these two distances, the location 
% of each anchor can be determined.
%
% If feasible, it would be more suitable to use a MoCap device to obtain 
% the ground truth.

anc_data = readtable([path,'anchor.txt']);
d = anc_data{1,1};
for i = 2:anc_num+1
    l1 = anc_data{i,2};
    l2 = anc_data{i,3};
    x = (l2^2-l1^2-d^2)/(-2*d);
    y = sqrt(l1^2-x^2);
    loc_sign = anc_data{i,1};
    anc_loc(1,i-1) = loc_sign*y;
    anc_loc(2,i-1) = x;
end

gt_data = readtable([path,'ground_truth.txt']);

for i = 1:length(gt_data{:,1})
    if strcmp(filename,gt_data{i,1})
        loc_sign_s = gt_data{i,2};
        l1_s = gt_data{i,3};
        l2_s = gt_data{i,4};
        loc_sign_e = gt_data{i,5};
        l1_e = gt_data{i,6};
        l2_e = gt_data{i,7};
        x_s = (l2_s^2-l1_s^2-d^2)/(-2*d);
        y_s = sqrt(l1_s^2-x_s^2);        
        x_e = (l2_e^2-l1_e^2-d^2)/(-2*d);
        y_e = sqrt(l1_e^2-x_e^2);        
        start_loc(1) = loc_sign_s*y_s;
        start_loc(2) = x_s;
        end_loc(1) = loc_sign_e*y_e;
        end_loc(2) = x_e;
        break;
    else
        continue;
    end
end

end