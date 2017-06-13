%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function [ref,YunQ] = yunmodel(capses,statref,Ndata,par,CovMat,Indata,debug)

if strcmp(capses,'Jun12')
    ref.g = statref.g;
    ref.m = statref.m;
    sig.v_g = mean(var(Ndata.Static_glob_cam.g'));
    sig.v_m = mean(var(Ndata.Static_glob_cam.m'));

elseif strcmp(capses,'Aug12')
    ref.g = statref.g;
    ref.m = statref.m;
%                 sig.v_g = 100*mean(var(steadydata.cam_imu(:,par.accidxs)'));
%                 sig.v_m = mean(var(steadydata.cam_imu(:,par.accidxs)'));
else
    ref.g = mean(Ndata.L_lower_arm.g')';
    ref.m = mean(Ndata.L_lower_arm.m')';
end


dt = par.dt;%to be set as diff of captured time
tau = 1*[0.5 0.5 0.5];
D = [0.4 0.4 0.4];

P{1} = eye(7);
H = eye(7);

% Yunflds =
% {'R_lower_arm';'R_Shoulder';'L_upper_arm';'L_Shoulder';'L_lower_arm';'Static_glob_cam';'Trunk'};
Yunflds = {'L_upper_arm';'L_lower_arm'};


% Q_Yun = [0.5*0.04*(1-exp(-2*dt/tau(1)))/tau(1) 0 0 0 0 0 0
%      0 0.5*0.04*(1-exp(-2*dt/tau(2)))/tau(2) 0 0 0 0 0
%      0 0 0.5*0.04*(1-exp(-2*dt/tau(3)))/tau(3) 0 0 0 0
%      zeros(4,7)];


for i=1:length(Yunflds)

    fldname = Yunflds{i}; 

    R_Yun = 35*diag([CovMat.(fldname).w(1,1) CovMat.(fldname).w(2,2) CovMat.(fldname).w(3,3) 0.0000001 0.0000001 0.0000001 0.0000001]);%35

    sig.w_g = 1*mean(var(Ndata.(fldname).g));
    sig.w_m = mean(var(Ndata.(fldname).m));

    ref.(fldname).g = [mean(Ndata.(fldname).g')]';
    ref.(fldname).m = [mean(Ndata.(fldname).m')]';

%                 if strcmp(fldname,'L_lower_arm')
%                     [qquest.LUN.q_opt_q, qquest.LUN.R, qquest.LUN.res, qquest.LUN.q, qquest.LUN.dp] = questalg(ref.L_lower_arm, ref.L_upper_arm);
%                 end


    %init
    YunQ.(fldname).q_ref = questalg(ref.(fldname),statref);
    qN.(fldname) = questalg(ref.(fldname),ref.(fldname));
    wN.(fldname) = -[mean(Ndata.(fldname).w')]';
%                 wNglo = quatrot([-qN.(fldname)(1:3);qN.(fldname)(4)],wN.(fldname));
%                 wN.(fldname) = wNglo(1:3);


    x(:,1) = [wN.(fldname); qN.(fldname)(4); qN.(fldname)(1:3)]; %to be selected from data
    Q.(fldname).q_opt(:,1) = x(4:7,1);
    wmeas = zeros(3,1);
%                 qbuf(:,1)=zeros(4,1);
    qbuf(:,1)=qN.(fldname);
    bestidx = [0 0];
%                 curg = quatrot([-qbuf(1:3);qbuf(4)],Indata.(fldname).g(:,1));
%                 curm = quatrot([-qbuf(1:3);qbuf(4)],Indata.(fldname).m(:,1));
%             end
    j=1;
    for k=2:length(Indata.(fldname).g) 

        j=j+1;

        qcur.g = Indata.(fldname).g(:,j);
        qcur.m = Indata.(fldname).m(:,j);
%                     curg(:,j-1) = quatrot([-qbuf(1:3,j-1);qbuf(4,j-1)],Indata.(fldname).g(:,j));
%                     curm(:,j-1) = quatrot([-qbuf(1:3,j-1);qbuf(4,j-1)],Indata.(fldname).m(:,j));
%                     qcur.g = curg(1:3,j-1);
%                     qcur.m = curm(1:3,j-1);
        wmeas(:,j) = -Indata.(fldname).w(:,j);
%                     wglo = quatrot([-qbuf(1:3,j-1);qbuf(4,j-1)],wmeas(:,j));
%                     wmeas(:,j) = wglo(1:3);

        if strcmp(fldname,'L_lower_arm')

%                 g_out= quatrot(qquest.LUN.q_opt_q, qcur.g);
%                 m_out= quatrot(qquest.LUN.q_opt_q, qcur.m);
%                 wout = quatrot(qquest.LUN.q_opt_q, wmeas(:,j));
% 
%                 qin.(fldname).g = g_out(1:3);
%                 qin.(fldname).m = m_out(1:3);
%                 wmeas(:,j) = wout(1:3);
            qin.(fldname).g = qcur.g;
            qin.(fldname).m = qcur.m;
%                         wout(1:3) = wmeas(:,j);
        else
            qin.(fldname).g = qcur.g;
            qin.(fldname).m = qcur.m;
%                         wout(1:3) = wmeas(:,j);

        end


        [qquest.(fldname).q_opt_q(:,j),qquest.(fldname).R{j},qquest.(fldname).res{j}, qquest.(fldname).q{j}, qquest.(fldname).dp(j)] = questalg(qin.(fldname),ref.(fldname));

        dp = qquest.(fldname).dp(j);

        if j>2                
            qquest.(fldname).q_opt(:,j) = questdebug(qquest.(fldname).q{j},qquest.(fldname).q_opt(:,j-1),dp);                
        else
            qquest.(fldname).q_opt(:,j) = qquest.(fldname).q_opt_q(:,j);
        end


        KQ_Yun(j) = 0.02;%10+0*(1/(1-dp)^0.5 + 1/(1-dp)^2)/60;%0.02
        Q_Yun{j} = KQ_Yun(j)*[0.5*0.04*(1-exp(-2*dt/tau(1)))/tau(1) 0 0 0 0 0 0
                   0 0.5*0.04*(1-exp(-2*dt/tau(2)))/tau(2) 0 0 0 0 0
                   0 0 0.5*0.04*(1-exp(-2*dt/tau(3)))/tau(3) 0 0 0 0
                   zeros(4,3) 1.0e-07*eye(4)];

        qbuf(:,j) = qquest.(fldname).q_opt(:,j);

        vect = qquest.(fldname).q_opt(1:3,j);
        real = qquest.(fldname).q_opt(4,j);

        qbuf(:,j) = [vect; real];
        qmeas.(fldname).q_opt(2:4,j) = vect;
        qmeas.(fldname).q_opt(1,j) = real;



        measdata(:,j) = [wmeas(:,j) ; qmeas.(fldname).q_opt(:,j)];

%                     A{j-1} = [exp(-dt/tau(1)) 0 0 0 0 0 0
%                             0 exp(-dt/tau(2)) 0 0 0 0 0
%                             0 0 exp(-dt/tau(3)) 0 0 0 0
%                             -0.5*x(5,j-1)*dt -0.5*x(6,j-1)*dt -0.5*x(7,j-1)*dt 1 -0.5*x(1,j-1)*dt -0.5*x(2,j-1)*dt -0.5*x(3,j-1)*dt
%                             0.5*x(4,j-1)*dt -0.5*x(7,j-1)*dt 0.5*x(6,j-1)*dt 0.5*x(1,j-1)*dt 1 0.5*x(3,j-1)*dt -0.5*x(2,j-1)*dt
%                             0.5*x(7,j-1)*dt 0.5*x(4,j-1)*dt -0.5*x(5,j-1)*dt 0.5*x(2,j-1)*dt -0.5*x(3,j-1)*dt 1 0.5*x(1,j-1)*dt
%                             -0.5*x(6,j-1)*dt 0.5*x(5,j-1)*dt 0.5*x(4,j-1)*dt 0.5*x(3,j-1)*dt 0.5*x(2,j-1)*dt -0.5*x(1,j-1)*dt 1];

        A{j-1} = [exp(-dt/tau(1)*0) 0 0 0 0 0 0
                0 exp(-dt/tau(2)*0) 0 0 0 0 0
                0 0 exp(-dt/tau(3)*0) 0 0 0 0
                0 0 0 1 -0.5*x(1,j-1)*dt -0.5*x(2,j-1)*dt -0.5*x(3,j-1)*dt
                0 0 0 0.5*x(1,j-1)*dt 1 0.5*x(3,j-1)*dt -0.5*x(2,j-1)*dt
                0 0 0 0.5*x(2,j-1)*dt -0.5*x(3,j-1)*dt 1 0.5*x(1,j-1)*dt
                0 0 0 0.5*x(3,j-1)*dt 0.5*x(2,j-1)*dt -0.5*x(1,j-1)*dt 1];

        x_pre(:,j) = A{j-1}*x(:,j-1);
        Pproc{j} = A{j-1}*P{j-1}*A{j-1}';
        P_pre{j} = A{j-1}*P{j-1}*A{j-1}' + Q_Yun{j};

        K{j} = P_pre{j}*H'*inv(H*P_pre{j}*H' + R_Yun);      
        res.(fldname)(:,j) = (measdata(:,j)-H*x_pre(:,j));

        x(:,j) = x_pre(:,j) + K{j}*res.(fldname)(:,j);
        P{j} = (eye(7)-K{j}*H)*P_pre{j};

        x(4:7,j) = x(4:7,j)/norm(x(4:7,j));

        mytr.(fldname)(j) = trace(P{j});
        Q.(fldname).w(:,j) = x(1:3,j);
        Q.(fldname).q_opt(:,j) = x(4:7,j);

    end
end

for i=1:size(Indata.(fldname).g,2)
    YunQ.L_upper_arm.q_opt(:,i) = [Q.L_upper_arm.q_opt(2:4,i); Q.L_upper_arm.q_opt(1,i)];
    YunQ.L_lower_arm.q_opt(:,i) = [Q.L_lower_arm.q_opt(2:4,i); Q.L_lower_arm.q_opt(1,i)];
end


if debug

    figure
    subplot(2,2,1)
    plot(Indata.L_lower_arm.g'),grid
    subplot(2,2,2)
    plot(Indata.L_lower_arm.m'),grid
    subplot(2,2,3)
    plot(Indata.Static_glob_cam.g'),grid
    subplot(2,2,4)
    plot(Indata.Static_glob_cam.m'),grid

    figure
    subplot(3,1,1)
    plot(Indata.L_lower_arm.w(1,:));
    hold on
    plot(Q.L_lower_arm.w(1,:),'r'),grid
    subplot(3,1,2)
    plot(Indata.L_lower_arm.w(2,:));
    hold on
    plot(Q.L_lower_arm.w(2,:),'r'),grid
    subplot(3,1,3)
    plot(Indata.L_lower_arm.w(3,:));
    hold on
    plot(Q.L_lower_arm.w(3,:),'r'),grid



    figure
    q2plot =[Q.L_lower_arm.q_opt(2:4,:);Q.L_lower_arm.q_opt(1,:)];
    qvic2plot = [qmeas.L_lower_arm.q_opt(2:4,:);qmeas.L_lower_arm.q_opt(1,:)];
    for i=1:4
        subplot(4,1,i)
        plot(q2plot(i,:)),hold on
        plot(qvic2plot(i,:),'r')
        grid
    end

    figure
    q2plot =[Q.L_upper_arm.q_opt(2:4,:);Q.L_upper_arm.q_opt(1,:)];
    qvic2plot = [qmeas.L_upper_arm.q_opt(2:4,:);qmeas.L_upper_arm.q_opt(1,:)];
    for i=1:4
        subplot(4,1,i)
        plot(q2plot(i,:)),hold on
        plot(qvic2plot(i,:),'r')
        grid
    end

    figure
    plot(qquest.L_lower_arm.q_opt')
    hold on
    plot(Indata.L_lower_arm.g(1,:)/10,'k')
    grid

    figure
    subplot(3,2,1)
    hist(res.L_lower_arm(2,:))
    subplot(3,2,2)
    hist(res.L_lower_arm(3,:))
    subplot(3,2,3)
    hist(res.L_lower_arm(4,:))
    subplot(3,2,4)
    hist(res.L_lower_arm(5,:))
    subplot(3,2,5)
    hist(res.L_lower_arm(6,:))
    subplot(3,2,6)
    hist(res.L_lower_arm(7,:))
end
