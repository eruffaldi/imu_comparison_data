%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function [ref,q_out] = youngmodel(version,statref,Ndata,par,Indata,debug)



% clear res x A P
bodypar.L_upper_arm = [0.001*par.la*mean(Ndata.L_upper_arm.g')/norm(mean(Ndata.L_upper_arm.g'))]';
bodypar.L_lower_arm = [0.001*par.lf*mean(Ndata.L_lower_arm.g')/norm(mean(Ndata.L_lower_arm.g'))]';

Youngflds = {'L_upper_arm','L_lower_arm'};

if strfind (version,'pure')
    lingain = 0;
    %complementary constant
    k_comp = 100;
    %kalman filter cov mat
    Q_Y = 0.00000001*eye(4);
    R_Y = 0.00025*eye(4);%5
    C_Y = eye(4);

elseif strfind(version,'perfect')
    lingain = 1;
    %complementary constant
    k_comp = 50;
    %kalman filter cov mat
    Q_Y = 0.00000001*eye(4);
    R_Y = 0.00025*eye(4);%5
    C_Y = eye(4);
end

ref.L_lower_arm.g = [mean(Ndata.L_lower_arm.g')]';
ref.L_lower_arm.m = [mean(Ndata.L_lower_arm.m')]';

ref.L_upper_arm.g = [mean(Ndata.L_upper_arm.g')]';
ref.L_upper_arm.m = [mean(Ndata.L_upper_arm.m')]';




for i=1:length(Youngflds)

    fldname = Youngflds{i};

    parent = selparent(fldname);

    if strcmp(parent,'L_Shoulder')
        linacc.(parent) = zeros(3,length(Indata.(fldname).g));
    end

    q_out.(fldname).q_ref = questalg(ref.(fldname),statref);                

    % gyro preprocessing
    w = -Indata.(fldname).w;
    g = Indata.(fldname).g;
    m = Indata.(fldname).m;


    myfreq = 18;
    [a,b] = butter(7,myfreq/100);
    [a1,b1] = butter(7,0.3*myfreq/100);
    wfil(1,:) = filtfilt(a1,b1,w(1,:));
    wfil(2,:) = filtfilt(a,b,w(2,:));
    wfil(3,:) = filtfilt(a,b,w(3,:));
%                 gfil(1,:) = filtfilt(a,b,g(1,:));
%                 gfil(2,:) = filtfilt(a,b,g(2,:));
%                 gfil(3,:) = filtfilt(a,b,g(3,:));
%                 mfil(1,:) = filtfilt(a,b,m(1,:));
%                 mfil(2,:) = filtfilt(a,b,m(2,:));
%                 mfil(3,:) = filtfilt(a,b,m(3,:));
    gfil = g;
    mfil = m;
%                 wfil = w;

    dw = [zeros(1,3); diff(wfil')/par.dt]';
%                 dwfil = dw;
    dwfil(1,:) = filtfilt(a1,b1,dw(1,:));
    dwfil(2,:) = filtfilt(a1,b1,dw(2,:));
    dwfil(3,:) = filtfilt(a1,b1,dw(3,:));


    %initial quaternion
    qinit.g = [mean(Ndata.(fldname).g')]'; %loc
    qinit.m = [mean(Ndata.(fldname).m')]'; %loc
%                 sigin.(fldname).v_g = 100;
%                 sigin.(fldname).v_m = 1;
%                 sigin.(fldname).w_g = 100;
%                 sigin.(fldname).w_m = 1;

    [Qbuf.(fldname).q_opt,Qbuf.(fldname).R, Qbuf.(fldname).res, Qbuf.(fldname).dp] = questalg(qinit,ref.(fldname));
    qbuf = Qbuf.(fldname).q_opt;  
    Qbuf_init.(fldname) = Qbuf.(fldname).q_opt;

    % Kalman initialization

    
    datalen = size(Indata.(fldname).g,2);
    q_out_n = zeros(4,datalen);
    y = zeros(4,datalen);
    eps = zeros(4,datalen);
    qk = zeros(4,datalen);
    qk_p = zeros(4,datalen);
    A = cell(1,datalen);
    P_p = cell(1,datalen);
    K = cell(1,datalen);
    P = cell(1,datalen);
    
    x_out(:,1) = Qbuf_init.(fldname);
    P{1} = Q_Y;
    K{1} = zeros(size(x_out,1),6);


    for j=1:datalen

        if strfind(version,'complementary')

            [qd.(fldname)(:,j), W.(fldname){j}] = qtimeder(wfil(1:3,j),qbuf);
            q_p.(fldname)(:,j) = qbuf + qd.(fldname)(:,j)*par.dt;
            q_p.(fldname)(:,j) = q_p.(fldname)(:,j)/norm(q_p.(fldname)(:,j));

            parentacc.(fldname)(:,j) = linacc.(parent)(:,j);

%                         sqw.(fldname)(:,j) = wfil(:,j)'*wfil(:,j);

            [linacc.(fldname)(:,j), arel_loc.(fldname)(:,j), atot_glo.(fldname)(:,j)] = LinAccEst(parentacc.(fldname)(:,j),bodypar.(fldname),-wfil(:,j),-dwfil(:,j),qbuf);


            qinquest.(fldname).g(:,j) = gfil(:,j) + linacc.(fldname)(:,j) * lingain;
            qin.(fldname).g = qinquest.(fldname).g(:,j);
            qin.(fldname).m = mfil(:,j); 


            [Q.(fldname).q_opt_q(:,j),Q.(fldname).R{j},Q.(fldname).res{j}, Q.(fldname).q{j}, Q.(fldname).dp(j)] = questalg(qin.(fldname),ref.(fldname));

            dp = Q.(fldname).dp(j);

            if j>1                
                Q.(fldname).q_opt(:,j) = questdebug(Q.(fldname).q{j},Q.(fldname).q_opt(:,j-1),dp);                
            else
                Q.(fldname).q_opt(:,j) = Q.(fldname).q_opt_q(:,j);
            end


            %drift corr
            q_out.(fldname).q_opt(:,j) = q_p.(fldname)(:,j) + (Q.(fldname).q_opt(:,j) - q_p.(fldname)(:,j))/k_comp; %64 (8 pu/k)
            q_out.(fldname).q_opt(:,j) = q_out.(fldname).q_opt(:,j)/norm(q_out.(fldname).q_opt(:,j));
            q_out_n(:,j) = norm(q_out.(fldname).q_opt(:,j));

            qbuf = q_out.(fldname).q_opt(:,j);

            mein.g = gfil(:,j);
            mein.m = mfil(:,j);
            [qmeas.(fldname).q_opt_q(:,j),qmeas.(fldname).R{j},qmeas.(fldname).res{j}, qmeas.(fldname).q{j}, qmeas.(fldname).dp(j)] = questalg(mein,ref.(fldname));

            dp1 = qmeas.(fldname).dp(j);

            if j>1                
                qmeas.(fldname).q_opt(:,j) = questdebug(qmeas.(fldname).q{j},qmeas.(fldname).q_opt(:,j-1),dp1);                
            else
                qmeas.(fldname).q_opt(:,j) = qmeas.(fldname).q_opt_q(:,j);
            end
            
            

        elseif strfind(version,'kalman')

            % prediction and kalman gain                        
            Sw = skewmat(wfil(:,j));
            A{j} = 0.5*[Sw wfil(:,j);-wfil(:,j)' 0]*par.dt + eye(4);

            qk_p(:,j) = A{j}*qbuf;
            if j>1
                P_p{j} = A{j}*P{j-1}*A{j}' + Q_Y;
            else
                P_p{j} = P{1};
            end

            K{j} = P_p{j}*C_Y' / (C_Y*P_p{j}*C_Y' + R_Y);
            P{j} = (eye(4) - K{j}*C_Y) * P_p{j};

            %measurement
            parentacc.(fldname)(:,j) = linacc.(parent)(:,j);
            [linacc.(fldname)(:,j), arel_loc.(fldname)(:,j), atot_glo.(fldname)(:,j)] = LinAccEst(parentacc.(fldname)(:,j),bodypar.(fldname),-wfil(:,j),-dwfil(:,j),qbuf);

            qin.(fldname).g = gfil(:,j) + linacc.(fldname)(:,j) * lingain;
            qinquest.(fldname).g(:,j) = qin.(fldname).g;
            qin.(fldname).m = mfil(:,j);

            [Q.(fldname).q_opt_q(:,j),Q.(fldname).R{j},Q.(fldname).res{j}, Q.(fldname).q{j}, Q.(fldname).dp(j)] = questalg(qin.(fldname),ref.(fldname));

            dp = Q.(fldname).dp(j);

            if j>1                
                Q.(fldname).q_opt(:,j) = questdebug(Q.(fldname).q{j},Q.(fldname).q_opt(:,j-1),dp);                
            else
                Q.(fldname).q_opt(:,j) = Q.(fldname).q_opt_q(:,j);
            end

            %update
            y(:,j) = Q.(fldname).q_opt(:,j);
            eps(:,j) = y(:,j) - C_Y*qk_p(:,j);
            qk(:,j) = qk_p(:,j) + K{j}*eps(:,j);

            q_out.(fldname).q_opt(:,j) = qk(:,j)/norm(qk(:,j));
            res.(fldname)(:,j) = eps(:,j);
            qbuf = q_out.(fldname).q_opt(:,j);

            mein.g = gfil(:,j);
            mein.m = mfil(:,j);
            [qmeas.(fldname).q_opt_q(:,j),qmeas.(fldname).R{j},qmeas.(fldname).res{j}, qmeas.(fldname).q{j}, qmeas.(fldname).dp(j)] = questalg(mein,ref.(fldname));

            dp1 = qmeas.(fldname).dp(j);

            if j>1                
                qmeas.(fldname).q_opt(:,j) = questdebug(qmeas.(fldname).q{j},qmeas.(fldname).q_opt(:,j-1),dp1);                
            else
                qmeas.(fldname).q_opt(:,j) = qmeas.(fldname).q_opt_q(:,j);
            end

        end
    end
end


if debug
    figure

    for i=1:4
        subplot(4,1,i)
        plot(q_out.L_lower_arm.q_opt(i,:)),hold on
        plot(qmeas.L_lower_arm.q_opt(i,:),'r')
        plot(Q.L_lower_arm.q_opt(i,:),'g')
        grid
    end

    figure
    for i=1:4
        subplot(4,1,i)
        plot(q_out.L_upper_arm.q_opt(i,:)),hold on
        plot(qmeas.L_upper_arm.q_opt(i,:),'r')
        plot(Q.L_upper_arm.q_opt(i,:),'g')
        grid
    end

    figure
    for i=1:4
        subplot(2,2,i)
        hist(res.L_lower_arm(i,:));
    end     
        figure

        subplot(2,2,1)
        plot(wfil(1,:)),grid, hold on
        plot(w(1,:),'r')

        subplot(2,2,2)
        plot(wfil(2,:)),grid, hold on
        plot(w(2,:),'r')

        subplot(2,2,3)
        plot(wfil(3,:)),grid, hold on
        plot(w(3,:),'r')

        subplot(2,2,4)
        plot(dwfil'),grid

        figure

        subplot(2,1,1)
        plot(arel_loc.L_lower_arm'),grid

        subplot(2,1,2)
        plot(atot_glo.L_lower_arm'),grid

        figure
        plot(Q.L_lower_arm.dp)
end