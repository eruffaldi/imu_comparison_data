%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
% Dependencies:
%
% hfunctions? h_pep_5D_L hnjoint_SensOr_5L hnjoint_Mag_5L
% covariances? similar ration 1:100 with Zhu, but different depending on
% sensor
function [ang,opos,Z, ukf_s_params, outCov] = pepmodel(par,Ndata,Tdata,CovMat,Indata,debug,mode)

if mode == 3 % scaling of variables + smooth
    smooth = 1;
    special = 1;
elseif mode == 2 % scaling of variables
    smooth = 1;
    special = 0;
elseif mode == 1
    smooth = 0;
    special = 1;
else
    smooth = 0;
    special = 0;
end

q = 0;
p = 0;
% clc
%%%
%polish data
flds = fields(Ndata);
Ngoodidxs = [];
Tgoodidxs = [];
Igoodidxs = [];

cholx = @chol2; % similar to sdchol
cholx = @cholcov; % MATLAB - NOT WORKING
cholx = @choler; % my with special for 3x3 and then using sdchol
cholx = @cholxbyqr; % using QR - NOT WORKING
cholx = @(X) chol(X)'; % good for def+

% computes missing data
for i=1:length(flds)
   fldname = flds{i};
   Ngoodidxs = [Ngoodidxs; sum(Ndata.(fldname).g)~=0 & sum(Ndata.(fldname).m~=0) & sum(Ndata.(fldname).w,1)~=0]; 
   Tgoodidxs = [Tgoodidxs; sum(Tdata.(fldname).g)~=0 & sum(Tdata.(fldname).m~=0) & sum(Tdata.(fldname).w,1)~=0];
end

ngidxs = sum(Ngoodidxs);
tgidxs = sum(Tgoodidxs);

% remove NaN or missing data in general
for i=1:length(flds)
   fldname = flds{i}; 
   Ndata.(fldname).g = Ndata.(fldname).g(:,ngidxs==max(ngidxs));
   Ndata.(fldname).m = Ndata.(fldname).m(:,ngidxs==max(ngidxs));
   Ndata.(fldname).w = Ndata.(fldname).w(:,ngidxs==max(ngidxs));
   
   Tdata.(fldname).g = Tdata.(fldname).g(:,tgidxs==max(tgidxs));
   Tdata.(fldname).m = Tdata.(fldname).m(:,tgidxs==max(tgidxs));
   Tdata.(fldname).w = Tdata.(fldname).w(:,tgidxs==max(tgidxs));
   
end

% bias of the gyro
for i=1:length(flds)
   fldname = flds{i}; 
   w_off.(fldname) = mean(Ndata.(fldname).w');
end

% OPTIONAL removal on ALL data
if 1==0 %0 - 0
    iflds = fields(Indata);
    for i=1:length(flds)
       fldname = iflds{i};
       Igoodidxs = [Igoodidxs; sum(Indata.(fldname).g)~=0 & sum(Indata.(fldname).m~=0) & sum(Indata.(fldname).w,1)~=0]; 
    end

    igidxs = sum(Igoodidxs);

    for i=1:length(flds)
       fldname = flds{i}; 
       Indata.(fldname).g = Indata.(fldname).g(:,igidxs==max(igidxs));
       Indata.(fldname).m = Indata.(fldname).m(:,igidxs==max(igidxs));
       Indata.(fldname).w = Indata.(fldname).w(:,igidxs==max(igidxs));  
       %%%%
%        Ndata.(fldname).g = Indata.(fldname).g(:,1:100);
%        Ndata.(fldname).m = Indata.(fldname).m(:,1:100);
%        Ndata.(fldname).w = Indata.(fldname).w(:,1:100);
    end
end

sn = size(Ndata.L_lower_arm.g,2);
st = size(Tdata.L_lower_arm.g,2);
% luaR = 0.27;
luaL = par.la/1000;
% lfaR = 0.25;
lfaL =par.lf/1000;

p1L = [luaL*0.66 0 -0.05]; %x y z rispetto al frame padre scritti nel frame sensor
p2L = [lfaL*0.66 0 -0.05];

m0i = [10 10 10]; 

%%% check data, in particular the MAG

m11 = pw_norm(Indata.L_upper_arm.m');
m12 = pw_norm(Indata.L_lower_arm.m');

figure
subplot(2,1,1)
plot(m11)
subplot(2,1,2)
plot(m11)

%% sensor orientation estimation
if exist('x_or','var')==0

    x1L = [0 0 pi/2]; %z y x rispetto al frame padre
    x2L = [pi/2 0 pi/2];
%     x1L = [0 0 0];x2L = [0 0 0];
    xq2 = 0;%***
    xq5 = 0;%****

    sc_params = [luaL lfaL m0i x1L p1L m0i x2L p2L];
    % x =[x1L x2L];
    % q=zeros(5,1);

    % Y = [Sens5.acc(stN:endN,:)' Sens5.acc(stT:endT,:)'; Sens1.acc(stN:endN,:)' Sens1.acc(stT:endT,:)'; Sens2.acc(stN:endN,:)' Sens2.acc(stT:endT,:)';Sens3.acc(stN:endN,:)' Sens3.acc(stT:endT,:)';Sens4.acc(stN:endN,:)' Sens4.acc(stT:endT,:)'];
    % imu acc data as 3xn matrix, alternate IMU1-N IMU1-T IMU2-N IMU2-T

    Yo = [Ndata.L_upper_arm.g Tdata.L_upper_arm.g; Ndata.L_lower_arm.g Tdata.L_lower_arm.g]/9.81;
    % Y = [Ndata.L_upper_arm.g; Ndata.L_lower_arm.g]/9.81;
    % Y = [Tdata.L_upper_arm.g; Tdata.L_lower_arm.g]/9.81;

    % q = [zeros(endN-stN+1,12); zeros(endT-stT+1,2) pi/2*ones(endT-stT+1,1) zeros(endT-stT+1,4) pi/2*ones(endT-stT+1,1) zeros(endT-stT+1,4)];
    % q as zeros or pi/2 sized mxp where m equals size(Y,2) and p is the number of joints
    
    q = [zeros(sn,4) -pi/2*ones(sn,1); pi/2*ones(st,1) zeros(st,4)];%
%     q = [zeros(sn,4); pi/2*ones(st,1) zeros(st,3)];%***,****
%     q = [zeros(sn,5)];
    % q = [pi/2*ones(st,1) zeros(st,4)];

    x0s = [x1L x2L];%
%     x0s = [x1L x2L xq2];%***
%     x0s = [x1L x2L xq2 xq5];%****
    myfun = @(x) hnjoint_SensOr_5L(x,Yo,q,sc_params);

    [x_or,fval] = fminunc(myfun,x0s);
    [x0s' x_or'*180/pi]
%     fval
    % oo.x = x;
    % oo.fval =fval;
    % assignin('base','x_out',oo)

    close all
    if debug
        subplot(211)
        plot(Yo(1:3,:)'),title('Uarm')

        subplot(212)
        plot(Yo(4:6,:)'),title('Larm')
    end
end
% x_or(7:8)
%%% m0 for magnetometers
% clc
% clear Mref

%% magnetometer calibration
if exist('Mref', 'var')==0

    ind=[1,2];
    m0i = [10 10 10];

    q = [zeros(sn,4) -pi/2*ones(sn,1); pi/2*ones(st,1) zeros(st,4)];
%     q = [zeros(sn,1), ones(sn,1)*x_or(7), zeros(sn,3); pi/2*ones(st,1), ones(st,1)*x_or(7), zeros(st,3)];%***
%     q = [zeros(sn,1), ones(sn,1)*x_or(7), zeros(sn,2),ones(sn,1)*x_or(8);pi/2*ones(st,1), ones(st,1)*x_or(7), zeros(st,2), ones(st,1)*x_or(8)];%****
%     q = zeros(sn,5);
%     q = [pi/2*ones(st,1) zeros(st,4)];
    Ym = [Ndata.L_upper_arm.m Tdata.L_upper_arm.m; Ndata.L_lower_arm.m Tdata.L_lower_arm.m];
%     Y = [Ndata.L_upper_arm.m; Ndata.L_lower_arm.m];
%     Y = [Tdata.L_upper_arm.m; Tdata.L_lower_arm.m];
%     Y = mean(Y,2)
%     q=mean(q)
    Mref = zeros(3,length(ind));

    for i=1:length(ind)
        myfun = @(m) hnjoint_Mag_5L(m,x_or,Ym,q, sc_params,ind(i));
        [Mref(:,i),fval] = fminunc(myfun,m0i');
%         fval
    end
    % M0
    Mref

end

%%% UKF setup
close all
% clc
Ts = 0.01;
nj = 5;


ukf_s_params = [luaL lfaL x_or(1:3) p1L Mref(:,1)' x_or(4:6) p2L Mref(:,2)'];
% ukf_s_params = [luaL lfaL x1L p1L Mref(:,1)' x2L p2L Mref(:,2)'];

alpha = 1;
AI = [1 Ts 0.5*Ts*Ts; 0 1 Ts; 0 0 alpha];
if special == 1
    s1 = 0.5;
    s2 = 1/50.0;
    vecReal2Xs = [1 s1 s2 1 s1 s2 1 s1 s2 1 s1 s2 1 s1 s2]';
    %vecReal2Xs = ones(15,1);
    vecXs2real = 1.0./vecReal2Xs;
    matXs2real = diag(vecXs2real);
    matReal2Xs = diag(vecReal2Xs);
    AI = matReal2Xs(1:3,1:3)*AI*matXs2real(1:3,1:3);
    
    zvecReal2Xs = [[1 1 1 1 1 1 1/50 1/50 1/50],[1 1 1 1 1 1 1/50 1/50 1/50]];
    zvecXs2real = 1.0./zvecReal2Xs;
    zmatXs2real = diag(zvecXs2real);
    zmatReal2Xs = diag(zvecReal2Xs);
    
else
    vecXs2real = ones(15,1);
    vecReal2Xs = vecXs2real;
    matXs2real = eye(15);
    matReal2Xs = matXs2real;
    
       
    zvecReal2Xs = ones(18,1);
    zvecXs2real = 1.0./zvecReal2Xs;
    zmatXs2real = diag(zvecXs2real);
    zmatReal2Xs = diag(zvecReal2Xs); 
end
M0 = matReal2Xs*zeros(nj*3,1);
% M0 = matReal2Xs*[zeros(4*3,1); pi/2; 0; 0];

A = blkdiag(AI,AI,AI,AI,AI);

P0 = matReal2Xs*diag(repmat(0.1*1e-0*CovMat.covEps,nj*3,1))*matReal2Xs';%0.45*1e-2 - 0.6*1e-0 - 0.2*1e-0
% P0(1:3,1:3) = diag([1e-4,1e-4,1e-4])*P0(1:3,1:3);
covQ1 = 1.0*1e-1*CovMat.covQ*[(0.5*Ts*Ts)^2 Ts^2 1]; %0.45*1e-1 - 1.2*1e-1 - 1.1*1e-0
Q = blkdiag(diag(covQ1), diag(covQ1), diag(covQ1), diag(covQ1), diag(covQ1));
Q = matReal2Xs*Q*matReal2Xs';

o1 = 1*1e-1*CovMat.L_upper_arm.w;%5*1e-4 - 1*1e-1
o2 = 1*1e-1*CovMat.L_lower_arm.w;%5*1e-1 - 1*1e-1

a1 = 1*1e-5*CovMat.L_upper_arm.g; %1*1e-4 - 1*1e-5
a2 = 1*1e-5*CovMat.L_lower_arm.g; %1*1e-4 - 1*1e-5

m1 = 5*1e-2*CovMat.L_upper_arm.m; %2*1e-3 - 5*1e-1
m2 = 5*1e-1*CovMat.L_lower_arm.m; %2*1e-1 - 5*1e-1
% det(a1)/det(m1)
% det(a2)/det(m2)

%m1=m2;
%a1=a2;
%o1=o2;
m2=m1;
a2=a1;
o2=o1;

R1 = blkdiag(o1,a1,m1);
R2 = blkdiag(o2,a2,m2);

R = zmatReal2Xs*blkdiag(R1,R2)*zmatReal2Xs';

% Z = [Ndata.L_upper_arm.w; Ndata.L_upper_arm.g/9.81; Ndata.L_upper_arm.m;...
%     Ndata.L_lower_arm.w; Ndata.L_lower_arm.g/9.81; Ndata.L_lower_arm.m];

Z = [Indata.L_upper_arm.w; Indata.L_upper_arm.g/9.81; Indata.L_upper_arm.m;...
    Indata.L_lower_arm.w; Indata.L_lower_arm.g/9.81; Indata.L_lower_arm.m];

Z = zmatReal2Xs*Z;
% R1 = blkdiag(o1,a1);
% R2 = blkdiag(o2,a2);
% 
% R = blkdiag(R1,R2);
% 
% Z = [Ndata.L_upper_arm.w; Ndata.L_upper_arm.g/9.81;...
%     Ndata.L_lower_arm.w; Ndata.L_lower_arm.g/9.81];

% Z =[zeros(3,149); repmat([0.95 0.15 -0.2]',1,149);...
%     zeros(3,149); repmat([0.94 0.34 0]',1,149)];

% Z = [Indata.L_upper_arm.w; Indata.L_upper_arm.g/9.81;...
%     Indata.L_lower_arm.w; Indata.L_lower_arm.g/9.81];

% Z=Z(:,1:20);



%%% ukf

l = size(Z,2);
% thsz = 600;
thsz = l;

ukfpar.alpha = 1;
ukfpar.n = 15;
ukfpar.k = 1;
ukfpar.beta = 2;

% [WM, W, cW] = ut_weights(alpha, beta, k, n);
%%%UKF

[WM, W, cW] = ut_mweights(ukfpar.n, ukfpar.alpha, ukfpar.beta, ukfpar.k);

Xout = zeros(size(A,1),thsz);
Pout = cell(thsz,1);
Yout = zeros(size(Z));
El_ou = zeros(3,thsz);
Wr_ou = zeros(3,thsz);
El_ouS = zeros(3,3,thsz);
Wr_ouS = zeros(3,3,thsz);
Puu = zeros(length(P0),length(P0),thsz);
Muu = zeros(length(P0),thsz);

xu = M0;
Pu = P0;

for i=1:thsz%l
    
    Xout(:,i) = xu;
    Pout{i} = Pu;
    
    %%PREDICT
    xp = A*xu;
    Pp = A*Pu*A'+Q;
    
    %%UPDATE
    %sigma points
    Ppc = cholx(Pp);
    Xs = [zeros(size(xp)) Ppc -Ppc];
    Xs = sqrt(cW)*Xs + repmat(xp,1,size(Xs,2));
    
    %transform
    Yt = zeros(size(Z,1),nj*3*2+1);
    for j=1:size(Xs,2)
        [an, ~] = h_pep_5D_L(Xs(:,j).*vecXs2real, ukf_s_params);
        Yt(:,j) =an;
    end
    Yt = zmatReal2Xs*Yt;
    xt = Yt*WM;
    S  = Yt*W*Yt';
    C  = Xs*W*Yt';
%     [xt Z(:,i)]
    %update
    S = S+R;
    K = C/S;
    xu = xp+K*(Z(:,i)-xt);
    Pu = Pp-K*S*K';
    
    xu(1:3:end) = atan2(sin(xu(1:3:end)),cos(xu(1:3:end))); % no scaling here
    Yout(:,i) = zmatXs2real*xt;
    
    if smooth == 1
        Puu(:,:,i) = Pu;
    end
    Muu(:,i) = xu;
    

    % extract covariance of output based on effective
    if smooth == 0 || i == thsz
%     [~, pos] = h_pep_5D_L(Xs(:,j), ukf_s_params);
%     [~, pos] = h_pep_5D_L([Xs(1:3,j); zeros(3,1); Xs(7:12,j); zeros(3,1)] , ukf_s_params);
        [~, pos] = h_pep_5D_L(xu.*vecXs2real, ukf_s_params);
        if nargout == 5
            Yte = zeros(3,nj*3*2+1); % sigma points
            Ytw = Yte;
%             i
            Ppc = cholx(Pu); % use effective out covariance
            Xs = [zeros(size(xp)) Ppc -Ppc];
            Xs = sqrt(cW)*Xs + repmat(xu,1,size(Xs,2)); % use effective mean
            for j=1:size(Xs,2)
                [~, pos] = h_pep_5D_L(Xs(:,j).*vecXs2real, ukf_s_params);
                Yte(:,j) = pos.elbow*1000;
                Ytw(:,j) = pos.wrist*1000;
            end
            % NB: the reconstructed mean should be pos ehm ...
            Se  = Yte*W*Yte';        
            El_ou(:,i) = Yte*WM;
            El_ouS(:,:,i) = Se;

            Sw  = Ytw*W*Ytw';        
            Wr_ou(:,i) = Ytw*WM;
            Wr_ouS(:,:,i) = Sw;
        else
            El_ou(:,i) = pos.elbow*1000;
            Wr_ou(:,i) = pos.wrist*1000;
        end
    end
%     [xt Z(:,i)]
    
end


if smooth == 1
    % start from last-1
  for i=thsz-1:-1:1
    
    % like a predict but on the filtered one
    k = i; % for avoiding replace
    PA = squeeze(Puu(:,:,k));
    
    m_pred = A*Muu(:,k);
    P_pred = A*PA*A'+Q;
    C = PA*A';       
    D = C / P_pred; % gain
    
    oldPu = Puu(:,:,k);
    Muu(:,k)   = Muu(:,k) + D * (Muu(:,k+1) - m_pred);        
    Puu(:,:,k) = Puu(:,:,k) + D * (Puu(:,:,k+1) - P_pred) * D';
    
    Pu = squeeze(Puu(:,:,k)); % store as new output
    
    % verify cholesky quality
    try
        xxx = cholx(Pu);
    catch me
        Pu = oldPu;
        % from now on everything is not working well
    end
    
    xu = Muu(:,k); % store as new output
    
    if nargout == 5
%     [~, pos] = h_pep_5D_L(Xs(:,j), ukf_s_params);
%     [~, pos] = h_pep_5D_L([Xs(1:3,j); zeros(3,1); Xs(7:12,j); zeros(3,1)] , ukf_s_params);
         Ppc = cholx(Pu); % use effective out covariance
        [~, pos] = h_pep_5D_L(xu.*vecXs2real, ukf_s_params);
        Yte = zeros(3,nj*3*2+1); % sigma points
        Ytw = Yte;
        Xs = [zeros(size(xp)) Ppc -Ppc];
        Xs = sqrt(cW)*Xs + repmat(xu,1,size(Xs,2)); % use effective mean
        for j=1:size(Xs,2)
            [~, pos] = h_pep_5D_L(Xs(:,j).*vecXs2real, ukf_s_params);
            Yte(:,j) = pos.elbow*1000;
            Ytw(:,j) = pos.wrist*1000;
        end
        % NB: the reconstructed mean should be pos ehm ...
        Se  = Yte*W*Yte';        
        El_ou(:,i) = Yte*WM;
        El_ouS(:,:,i) = Se;

        Sw  = Ytw*W*Ytw';        
        Wr_ou(:,i) = Ytw*WM;
        Wr_ouS(:,:,i) = Sw;
    else
        El_ou(:,i) = pos.elbow*1000;
        Wr_ou(:,i) = pos.wrist*1000;
    end
    
  end    


end



if debug
    
%     hnjoint_SensOr_5L_deb( x_or,Yo(:,1),[0 0 0 0],sc_params );
    h_pep_5D_L(zeros(15,1), ukf_s_params);
    figure
    plot(Xout(1:3:end,:)'*180/pi), legend({'q_1','q_2','q_3','q_4','q_5'})
    
    figure
    for i=1:6
        subplot(2,3,i)
        plot(Z((i-1)*3+1:i*3,1:thsz)')
        hold on
        plot(Yout((i-1)*3+1:i*3,1:thsz)','--')
    end
    
    figure
    plot3(El_ou(1,:),El_ou(2,:),El_ou(3,:),'*b','MarkerSize',2)
    hold on
    plot3(Wr_ou(1,:),Wr_ou(2,:),Wr_ou(3,:),'r')
    grid
    xlabel('x'),ylabel('y'),zlabel('z')
end

ang.Lelbow = Xout(10,:);
ang.all = Xout(1:3:end,:);
opos.Lelbow = El_ou;
opos.Lwrist = Wr_ou;
if nargout == 5
    outCov.Lelbow = El_ouS;
    outCov.Lwrist = Wr_ouS;
end

%%% old UKF


% load('m00302')
% load('xopt0302')
% load('m00702N')
% load('xopt0702N')


% Parameters structure
% pa2 = params(1);
% pd2 = params(2);
% pa5 = params(3);
% pd7 = params(4);
% pa8 = params(5);
% pd8 = params(6);
% pa11 = params(7);
% pd13 = params(8);
% rs_pi11 = params(9);
% rs_pi12 = params(10);
% rs_pi13 = params(11);
% ts_pi11 = params(12);
% ts_pi12 = params(13);
% ts_pi13 = params(14);
% m0_11 = params(15);
% m0_12 = params(16);
% m0_13 = params(17);

% lb = 0.6;
% lclR = 0.18;
% lclL = 0.18;
% luaR = 0.27;
% luaL = 0.27;
% lfaR = 0.25;
% lfaL = 0.25;


% m0 = [10 10 10]; 

% x0 =  [x(3) x(2) x(1)];
% x1R = [x(6) x(5) x(4)];
% x2R = [x(9) x(8) x(7)];
% x1L = [x(12) x(11) x(10)];
% x2L = [x(15) x(14) x(13)];

% 
% p0 = [-0.70 0 0];
% p1R = [0 luaR*0.66 -0.05];
% p2R = [0 -lfaR*0.66 -0.05];
% p1L = [0 luaL*0.66 -0.05];
% p2L = [0 -lfaR*0.66 -0.05];

% load('arms0702W');
% load('posopt0702N');

% hparams = [arms x0 s0P0' m0(1,:) x1R s1P1R' m0(2,:) x2R s2P2R' m0(3,:) x1L s1P1L' m0(4,:) x2L s2P2L' m0(5,:) zeros(1,9)];

% arms(2) = 0.16;
% arms(7) = 0.16;
% arms(1) = 0.55;
% arms(6) = 0.55;

% hparams = [arms x(1:3) s0P0' m0(1,:) x(4:6) s1P1R' m0(2,:) x(7:9) s2P2R' m0(3,:) x(10:12) s1P1L' m0(4,:) x(13:15) s2P2L' m0(5,:) zeros(1,9)];

%Data with N & T pose for sensors covariances
% load('dataNT0702');


% cmpar = [1 1 1 1 1 1 1 1 1 1 1 1e-1]; %wS1 wS2 aS1 aS2 mS1 mS2


% o2 = cmpar(1)*cov(gyroNT1(2000:2100,:));
% o3 = cmpar(4)*cov(gyroNT2(2000:2100,:));
% o4 = cmpar(7)*cov(gyroNT3(2000:2100,:));
% o5 = cmpar(10)*cov(gyroNT4(2000:2100,:));
% 
% a1 = cov(accNT5(2000:2100,:));
% a2 = cmpar(2)*cov(accNT1(2000:2100,:));
% a3 = cmpar(5)*cov(accNT2(2000:2100,:));
% a4 = cmpar(8)*cov(accNT3(2000:2100,:));
% a5 = cmpar(11)*cov(accNT4(2000:2100,:));
% 
% m1 = cov(magNT5(1000:1100,:));
% m2 = cmpar(3)*cov(magNT1(1000:1100,:));
% m3 = cmpar(6)*cov(magNT2(1000:1100,:));
% m4 = cmpar(9)*cov(magNT3(1000:1100,:));
% m5 = cmpar(12)*cov(magNT4(1000:1100,:));

% omega = 1e-3*eye(3);
% acc = 1e-4*eye(3);
% mag = 0.1*eye(3);
% pos = 1e-6;
% p = 1e-8*eye(3);
% ax = 1e-8*eye(3);

% R3 = blkdiag(o3,a3,m3);
% R4 = blkdiag(o4,a4,m4);
% R5 = blkdiag(o5,a5,m5);

% R = blkdiag(R1,R2,R3,R4,R5,pos,p,p,ax,ax);%,p,p,p,p);
% 
% for i=1:sample
%     Zn(:,i) = randng(1,Z(:,i),R);
%     
% end

% load('seat0702');
% load('Sens0702C');
% load('oarPos0702C');
% load('oarPos0702N');


% Zn = [Sens5.gyro'+repmat([0.1344 0 0]',1,size(Sens5.gyro,1));Sens5.acc';Sens5.mag';Sens1.gyro';Sens1.acc';Sens1.mag';Sens2.gyro';Sens2.acc';Sens2.mag';Sens3.gyro';Sens3.acc';Sens3.mag';Sens4.gyro';Sens4.acc';Sens4.mag'];
% st = 7669+3606;
% lst = 17530;
% ZnC = Zn(:,st:lst); 
% sample = size(ZnC,2);
% Add seat as last measurement
% ZnC = [ZnC;-tI(1181+3606:1181+3606+sample-1);pR(:,1:sample);pL(:,1:sample);xR(:,1:sample);xL(:,1:sample)];

% ZnC = ZnC(:,1870:end);

% YR.time = [];
% YR.signals.values = ZnC';
% YR.signals.dimensions = 58;

% simT = round(sample*0.01);

% Z = [Indata.L_upper_arm.w; Indata.L_upper_arm.g/9.81; Indata.L_upper_arm.m;...
%     Indata.L_lower_arm.w; Indata.L_lower_arm.g/9.81; Indata.L_lower_arm.m];




%%% debug

% %sensor orientations
% if 1==0
% clc
% 
% qL = [pi/2 0 0 0 0];
% xr = [0 0 pi/2 pi/2 0 pi/2];
% 
% hparams = [luaL lfaL m0i x1L p1L m0i x2L p2L];
% T0 = [-1 0 0;0 0 1;0 1 0];
% g = [0 0 -1]';
% 
% [T1, TL] = DHCh(qL, hparams(1:2));
% 
% t = 0*qL(1)+xr(1);
% y2 =  xr(2);
% x2 =  xr(3);
% S1 = [cos(t) -sin(t) 0 0;sin(t) cos(t) 0 0;0 0 1 0;0 0 0 1];
% S2 = [cos(y2) 0 sin(y2) 0; 0 1 0 0;-sin(y2) 0 cos(y2) 0; 0 0 0 1];
% S3 = [1 0 0 0;0 cos(x2) -sin(x2) 0;0 sin(x2) cos(x2) 0; 0 0 0 1];
% S1L = S1*S2*S3;
% 
% t = 0*qL(3)+xr(4);
% y2 =  xr(5);
% x2 =  xr(6);
% S1 = [cos(t) -sin(t) 0 0;sin(t) cos(t) 0 0;0 0 1 0;0 0 0 1];
% S2 = [cos(y2) 0 sin(y2) 0; 0 1 0 0;-sin(y2) 0 cos(y2) 0; 0 0 0 1];
% S3 = [1 0 0 0;0 cos(x2) -sin(x2) 0;0 sin(x2) cos(x2) 0; 0 0 0 1];
% S2L = S1*S2*S3;
% 
% Sg1L = T0*T1(1:3,1:3)*TL(1:3,1:3)*TL(1:3,5:7)*TL(1:3,9:11)*S1L(1:3,1:3)
% Sg2L = T0*T1(1:3,1:3)*TL(1:3,1:3)*TL(1:3,5:7)*TL(1:3,9:11)*TL(1:3,13:15)*TL(1:3,17:19)*S2L(1:3,1:3)
% 
% ddxs = [Sg1L(1:3,1:3)'*g;Sg2L(1:3,1:3)'*g];
% 
% end

end

function R = cholxbyqr(X)
[Q,R] = qr(X);
R = R';
end

% Returns the cholesky decomposition of MVG covariance matrix, as used for
% the UKF
%
% Emanuele Ruffaldi 2013 - PERCRO
function [ A ] = choler( X )

    if length(X) == 3
        % SDCHOL
        S = X;
        l11 = sqrt(S(1,1));
        l22 = sqrt(S(2,2)-S(2,1).^2);
        l33 = sqrt(S(3,3)-(S(3,1).^2+S(3,2).^2));
        l21 = S(2,1)/l11;
        l31 = S(3,1)/l11;
        l32 = (S(3,2)-l31*l21)/l22;
        A = [l11 0 0; l21 l22 0; l31 l32 l33];
    else
        A = sdchol(X)';
    end

end


function T = sdchol(A)
%SDCHOL Cholesky-like decomposition for positive semidefinite matrices
%   T = SDCHOL(A) uses only the diagonal and upper triangle of A.  If
%   A is positive semi-definite, then T=SDCHOL(A) produces a matrix T
%   such that A = T'*T.  If A is positive definite, then T is the
%   upper triangular Cholesky factor returned by chol(A).  If A is not
%   positive definite, T may be neither square nor triangular.  If
%   A has negative eigenvalues, an error message will be printed.

% Similar to: http://it.mathworks.com/matlabcentral/fileexchange/2360-the-matrix-computation-toolbox/content/matrixcomp/cholp.m
[T,p] = chol(A);

if p > 0
    [m,n] = size(A);

    assert(m == n, 'Matrix must be square.');

    % It's too bad matlab's ldl() doesn't have an option forcing the
    % diagonal matrix to be diagonal (and possibly complex) instead
    % of block diagonal, as it'd probably be the fastest and most
    % stable algorithm here. But we'll make do with the Schur
    % decomposition instead.
    A = triu(A) + triu(A,1)'; % use only upper triangle
    [U, D] = schur(A, 'complex'); % A = U*D*U' with U unitary, D
                                  % diagonal (since A symmetric)
    D = diag(D);

    if ~isreal(D)
        if any(imag(D) > 10*eps)
            error('Matrix must be positive semi-definite');
        else
            D = real(D);
        end
    end

    if any(D < 0)
        if any(D < 10 * eps)
            error('Matrix must be positive semi-definite');
        else
            D(D < 0) = 0;
        end
    end

    T = diag(sqrt(D)) * U';
end
end

function  L = chol2( A )


%CHOL2 Summary of this function goes here
%   Detailed explanation goes here
n = size(A,1);
L = eye(n);

% if any(eig(A) < 0) || any(any(imag(A)))
%     p = 1;
%     return
% end

funct

L(1,1) = sqrt(A(1,1));
% if imag(L(1,1))
%     error('L(1,1) complex!');
if L(1,1) == 0
    error('L(1,1) zero!');
end


for j=1:n
    L(j,1) = 1/L(1,1)*A(1,j);
end

for i=2:n
    L(i,i) = sqrt(A(i,i)-sum(L(i,1:i-1).^2));
%     if imag(L(i,i))
%      error('diagonal element L complex!');
    if L(i,i) == 0
        error('diagonal element L zero!');
    end
    for k = i+1:n
        L(k,i) = 1/L(i,i)*(A(k,i)-sum(L(k,1:i-1).*L(i,1:i-1)));
    end
end

end