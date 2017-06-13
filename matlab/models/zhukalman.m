%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function [g_out, m_out,eps,K,Ktrace]=zhukalman(g_in, m_in, w_in, dw_in, Cov,version)


% covariance matrices
% Q = [Cov.w+Cov.proc zeros(3,3); zeros(3,3) Cov.w+Cov.proc];
Q1 = cell(1,length(w_in));
for i=1:length(w_in)
    newcov_w = Cov.w;%/(0.5*(2+norm(w_in(:,i))));
    Q1{i} = [newcov_w zeros(3,3); zeros(3,3) newcov_w];
end

% Q1 = [Cov.w zeros(3,3); zeros(3,3) Cov.w];
Q2 = [Cov.proc zeros(3,3); zeros(3,3) Cov.proc];
R = [Cov.g zeros(3,3); zeros(3,3) Cov.m];

%inout data sample time
dt = 0.01;

% measuremente model
C = eye(6);

%initialization
thlen = length(g_in);
x_p = zeros(6,thlen);
x_out = zeros(6,thlen);
eps = zeros(6,thlen);
Ktrace = zeros(6,thlen);
P_p = cell(1,thlen);
P = cell(1,thlen);
K = cell(1,thlen);

x_out(:,1) = [g_in(:,1); m_in(:,1)];
P{1} = Q1{1}+Q2;
K{1} = zeros(size(x_out,1),6);
RotMat = eye(3);

for j=2:thlen
    
    %measurements
    y = [g_in(:,j); m_in(:,j)];
    
    Sw = skewmat(w_in(:,j));
    Sdw = skewmat(dw_in(:,j));
    Sxg = skewmat(x_out(1:3,j-1));
    Sxm = skewmat(x_out(4:6,j-1));
    
    
    if strcmp(version,'original')
        A =  [-Sw zeros(3,3); zeros(3,3) -Sw]*dt + eye(6);
        W = eye(6);
        
    elseif strcmp(version,'firstorder')
        %first order approx
        A =  [-Sw zeros(3,3); zeros(3,3) -Sw]*dt + eye(6);
        W = [-Sxg zeros(3,3); zeros(3,3) -Sxm]*dt;

    elseif strcmp(version,'secondorder')
        %second order approx
        A1 = (-Sw*dt-0.5*Sdw*dt^2+0.5*Sw*Sw*dt^2);
        A = [A1 zeros(3,3); zeros(3,3) A1] + eye(6);
        %neglecting delta in W (the complete expression includes terms in S(delta) and S(S(delta)*x)
        Wg = (Sw*dt^2-eye(3)*dt)*Sxg;
        Wm = (Sw*dt^2-eye(3)*dt)*Sxm;
%         Wg = secorderapp(x_out(1:3,j-1),RotMat,w_in,dt);
%         Wm = secorderapp(x_out(4:6,j-1),RotMat,w_in,dt);
        W = [Wg zeros(3,3); zeros(3,3) Wm];
              
    end

    %prediction
    x_p(:,j) = A*x_out(:,j-1);
    P_p{j} = A*P{j-1}*A' + W*Q1{j}*W' + Q2;

    %update
    K{j} = P_p{j}*C' / (C*P_p{j}*C'+R);
    eps(:,j) = (y-C*x_p(:,j));
    x_out(:,j) = x_p(:,j) + K{j}*eps(:,j);
    P{j} = (eye(length(x_out(:,1))) - K{j}*C) * P_p{j};
    
    v.g = x_out(1:3,j-1);
    v.m = x_out(4:6,j-1);
    w.g = x_out(1:3,j);
    w.m = x_out(4:6,j);
    
    [a,RotMat,b] = questalg(v,w);
    Ktrace(j) = trace(K{j});

end

g_out = x_out(1:3,:);
m_out = x_out(4:6,:);