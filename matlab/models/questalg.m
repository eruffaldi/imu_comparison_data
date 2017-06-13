%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
% quest algoritm for two vectors g,m to be inserted in the v (reference)
% and w (measured) structures. output is Im_x,Im_y,Im_z,Re_w, w=Rv
function [q_opt,R,res,q,dp] = questalg(v,w,sig)

if nargin<1
v.g = [1 1 0]';
v.m = [0 -1 0]';
w.g = [0 1 0]';
w.m = [1 -1 0]';
end

v_g =v.g/norm(v.g);
v_m =v.m/norm(v.m);
w_g =w.g/norm(w.g);
w_m =w.m/norm(w.m);

dp = v_g'*v_m;

if nargin<3
    sig.v_g = 0.1;
    sig.v_m = 0.1;
    sig.w_g = 0.1;
    sig.w_m = 0.1;
end

sq_sig_g = sig.v_g^2 + sig.w_g^2;
sq_sig_m = sig.v_m^2 + sig.w_m^2;
sq_sig_tot = 1/(1/sq_sig_g+1/sq_sig_m);

a_g = sq_sig_tot/sq_sig_g;
a_m = sq_sig_tot/sq_sig_m;

B = a_g*w_g*v_g' + a_m*w_m*v_m';

k_sig = trace(B);
S = B+B';
Z = a_g*cross(w_g,v_g) + a_m*cross(w_m,v_m);

K = [S-k_sig*eye(3), Z; Z', k_sig];

try
    [q,lambda] = eig(K);
catch me
    q_opt = [0 0 0 0]';
    R = zeros(3,3);
    res = zeros(6,1);
    q = zeros(4,4);
    return
end

[remove_a,eig_idx] = max(max(lambda));

q_opt = q(:,eig_idx);

q_i = q_opt(1:3);
q_r = q_opt(4);
R = (q_r^2-q_i'*q_i)*eye(3)+2*(q_i*q_i')+2*q_r*[0 q_i(3) -q_i(2); -q_i(3) 0 q_i(1); q_i(2) -q_i(1) 0];
res.g = w_g-R*v_g;
res.m = w_m-R*v_m;

R = cell(4);
for i=1:4
q_opt = q(:,i);
q_i = q_opt(1:3);
q_r = q_opt(4);
R{i} = (q_r^2-q_i'*q_i)*eye(3)+2*(q_i*q_i')+2*q_r*[0 q_i(3) -q_i(2); -q_i(3) 0 q_i(1); q_i(2) -q_i(1) 0];
res.g(:,i) = w_g-R{i}*v_g;
res.m(:,i) = w_m-R{i}*v_m;
myres(i) = norm([res.g(:,i); res.m(:,i)]);
end

[remove_b,bestidx] = min(myres);

q_opt = q(:,bestidx);
R = R{bestidx};
res.g = res.g(:,bestidx);
res.m = res.m(:,bestidx);

    
for i=1:4
    if q(4,i)<-0.01
       q(:,i)=-q(:,i);
    end
end
      
    
for i=1:size(q_opt,2)
    if q_opt(4,i)<-0.01
           q_opt=-q_opt;
    end
end
