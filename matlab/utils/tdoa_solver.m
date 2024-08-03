function [loc_result] = tdoa_solver(tdoa,anc_loc,method,taylor_iter,noise)
% tdoa_solver  Solver for TDoA-based localization
%
% ------ Input Parameters ------
% tdoa ----> TDoA estimation to be solved (N samples, M tdoa pairs)
% anc_loc ----> Anchors' locations (2, K anchors)
% method ----> Localization method ('LS', 'WLS', 'Chan')
% taylor_iter ----> Iteration number for taylor method, set 0 to disable
% noise ----> Noise estimate of different TDoA
%
% ------ Output Parameters ------
% loc_result ---->  Result of localization

warning off;

% Check the input parameters
if ~ismember(method,{'LS','WLS','Chan'})
    error("Unknown Localization Method for TDoA!");
end

[samples, pair_num] = size(tdoa);
[~, anc_num] = size(anc_loc);

loc_result = zeros(samples,2);

if strcmp(method,'LS')

    error("LS not Supported yet");
elseif strcmp(method,'WLS')
    
    error("WLS not Supported yet");
elseif strcmp(method,'Chan')
    
    for i = 1:samples

        p1 = -pinv(anc_loc(:,2:end)'-anc_loc(:,1)');
        p2 = tdoa(i,:)';
        p3 = 0.5*(vecnorm(anc_loc(:,1)',2,2).^2 + p2.^2 - vecnorm(anc_loc(:,2:end)',2,2).^2);
        a1 = anc_loc(:,1);
    
        a = (p1*p2)'*(p1*p2) - 1;
        b = (p1*p2)'*(p1*p3 - a1) + (p1*p3 - a1)'*p1*p2;
        c = (p1*p3 - a1)'*(p1*p3 - a1);
    
        r1 = (-b-sqrt(b^2-4*a*c))/(2*a);

        temp_xy = p1*p2*r1 + p1*p3;
        
        loc_result(i,1) = temp_xy(1);
        loc_result(i,2) = temp_xy(2);
    
    end
end

% Error mitigation based on Taylor series
if taylor_iter

    for i = 1:samples
        for j = 1:taylor_iter
        
            r = vecnorm(loc_result(i,:) - anc_loc',2,2);

            h = tdoa(i,:)' - (r(2:end) - r(1));
            g = (anc_loc(:,1)' - loc_result(i,:))/r(1) - (anc_loc(:,2:end)' - loc_result(i,:))./r(2:end);
            q = diag(noise);

            delta = (g'*(q\g))\(g'*(q\h));
            loc_result(i,:) = loc_result(i,:) + delta'; 

        end
    end

end

end