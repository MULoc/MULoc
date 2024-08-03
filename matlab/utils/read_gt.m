function [anc_loc,start_loc,end_loc] = read_gt(path,filename,anc_num)

anc_loc = zeros(2,anc_num);
start_loc = zeros(2,1);
end_loc = zeros(2,1);

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

end