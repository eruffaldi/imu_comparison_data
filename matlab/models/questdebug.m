%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function [q_out] = questdebug(Q,q_old,dp)
  
qres = Q'*q_old;
[remove_c,bestidx] = max(abs(qres));


if qres(bestidx)>0
    q_out = Q(:,bestidx);
else
    q_out = -Q(:,bestidx);
end


if abs(dp)>0.96 || isnan(dp)
    q_out = q_old;
end