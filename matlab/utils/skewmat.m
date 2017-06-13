%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function S = skewmat(x)

S = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];