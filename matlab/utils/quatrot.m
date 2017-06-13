%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
% v_out = q*v*qc

function v_out = quatrot(q,v)
q = q/norm(q);
l = norm(v);
v = v/l;
v = reshape(v,3,1);
v = [v;0];

qc = [-q(1:3); q(4)];

%real component last
Rv = mat4rot(v);
Rq = mat4rot(q);

v_out = l*Rq*Rv*qc;


function R = mat4rot(v)

R = [v(4) -v(3) v(2) v(1); v(3) v(4) -v(1) v(2); -v(2) v(1) v(4) v(3); -v(1) -v(2) -v(3) v(4)];