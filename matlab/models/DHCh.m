function  TL  = DHCh( q, params )
%DHCH DH for rowing kinematic chain
%   DHCh([qj(1) qj(2) qR qL], params(1:2));
%   params = [lb lclR luaR lfaR lb lclL luaL lfaL];

% lb = params(1);
% clR = params(2);
% luaR = params(3);
% lfaR = params(4);
% lb = params(5);
% clL = params(6);
% luaL = params(7);
% lfaL = params(8);

luaL = params(1);
lfaL = params(2);

% 
% qR = q(3:7);
% qL = q(8:end);

% aR = [lb 0 0 luaR 0 0];
% alphaR = [-pi/2 pi/2 -pi/2 0 pi/2 0];
% dR = [clR 0 0 0 0 lfaR];
% thetaR = [q(2) qR(1) qR(2) qR(3) qR(4) qR(5)]; 
% offsetR = [0 -pi/2 -pi/2 -pi/2 pi/2 0];

%old
% aL = [0 0 luaL 0 0];
% alphaL = [pi/2 -pi/2 0 pi/2 0];
% dL = [0 0 0 0 lfaL];
% thetaL = q; %q1...q5
% offsetL = [-pi/2 pi/2 -pi/2 pi/2 0];

%old from hsensorgoOAR TODO
% aL = [0 0 luaL 0 0];
% alphaL = [pi/2 -pi/2 0 pi/2 0];
% dL = [0 0 0 0 lfaL];
% thetaL = q; %q1...q5
% offsetL = [-pi/2 pi/2 -pi/2 pi/2 0];

%new from DH_data_pap_fig_IndErg.txt
aL = [0 0 luaL 0 0];
alphaL = [pi/2 pi/2 0 pi/2 pi/2];
dL = [0 0 0 0 lfaL];
thetaL = q; %q1...q5
offsetL = [0 -pi/2 pi/2 pi/2 pi/2];


a1 = 0;
alpha1 = -pi/2;
d1 = q(1);
theta1 = pi/2;
offset1 = 0;

% TR = zeros(4,24);
TL = zeros(4,20); 

ct = cos(theta1 + offset1);
st = sin(theta1 + offset1);
ca = cos(alpha1);
sa = sin(alpha1);
% T1 = [ct -st*ca st*sa a1*ct;
%     st ct*ca -ct*sa a1*st;
%     0   sa      ca    d1;
%     0   0        0    1];
%     
% T1 = eye(4);

for i=1:5
    
%     ct = cos(thetaR(i) + offsetR(i));
%     st = sin(thetaR(i) + offsetR(i));
%     ca = cos(alphaR(i));
%     sa = sin(alphaR(i));
%     TR(1:4,(i-1)*4+1:(i-1)*4+4) = [ct -st*ca st*sa aR(i)*ct;
%         st ct*ca -ct*sa aR(i)*st;
%         0   sa      ca    dR(i);
%         0   0        0    1];
    
    ct = cos(thetaL(i) + offsetL(i));
    st = sin(thetaL(i) + offsetL(i));
    ca = cos(alphaL(i));
    sa = sin(alphaL(i));
    TL(1:4,(i-1)*4+1:(i-1)*4+4) = [ct -st*ca st*sa aL(i)*ct;
        st ct*ca -ct*sa aL(i)*st;
        0   sa      ca    dL(i);
        0   0        0    1];
end


end