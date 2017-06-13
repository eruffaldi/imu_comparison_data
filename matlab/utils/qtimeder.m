%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function [qd, W] = qtimeder(w,q)

%%% w = [w1 w2 w3]', q = q1*i+q2*j+q3*k+q4

% W = [0 w(3) -w(2) w(1); -w(3) 0 w(1) w(2); w(2) -w(1) 0 w(3); -w(1) -w(2)
% -w(3) 0];

Sw = skewmat(w);

W = [Sw w;-w' 0];

qd = 0.5*W*q;


