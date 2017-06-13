%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function [Q, KQ, ref, Kdata] = zhumodel(stat_ref, Indata, CovMat, Ndata, version, side, debug)

% Indata = rmfield(Indata,'R_lower_arm');

dataflds = fields(Indata);

% stat_ref.g = mean(Ndata.Static_glob_cam.g')';
% stat_ref.m = mean(Ndata.Static_glob_cam.m')';

for i=1:length(dataflds)
    
    fldname = dataflds{i};        
        
        % Filter g and m vectors
        [Kdata.(fldname).g, Kdata.(fldname).m, Kdata.(fldname).eps,Kdata.(fldname).K,Kdata.(fldname).Ktrace] = zhukalman(Indata.(fldname).g, Indata.(fldname).m, Indata.(fldname).w, Indata.(fldname).dw, CovMat.(fldname),version);
        
        % Rotation estimate with respect to Npose local frame
        
        ref.(fldname).g = mean(Ndata.(fldname).g')';
        ref.(fldname).m = mean(Ndata.(fldname).m')';
        
        for j=1:size(Indata.(fldname).g,2)
%             j          
            qin.(fldname).g = Indata.(fldname).g(:,j);
            qin.(fldname).m = Indata.(fldname).m(:,j);
            
            Kqin.(fldname).g = Kdata.(fldname).g(:,j);
            Kqin.(fldname).m = Kdata.(fldname).m(:,j);
            
            sig.v_g =1*100;
            sig.v_m =1;
            sig.w_g =1*100;
            sig.w_m =1;
            
%             [KQ.(fldname).q_opt(:,j),KQ.(fldname).R{j}, KQ.(fldname).res_UA{j}] = questalg(Kqin.(fldname),ref.(fldname));
%             [Q.(fldname).q_opt(:,j),Q.(fldname).R{j}, Q.(fldname).res_UA{j}] = questalg(qin.(fldname),ref.(fldname));
%             [KQ.(fldname).q_opt(:,j),KQ.(fldname).R{j}, KQ.(fldname).res_UA{j}] = questalg(Kqin.(fldname),stat_ref);
%             [Q.(fldname).q_opt(:,j),Q.(fldname).R{j}, Q.(fldname).res_UA{j}] = questalg(qin.(fldname),stat_ref);
            [KQ.(fldname).q_opt_q(:,j),KQ.(fldname).R{j}, KQ.(fldname).res_UA{j},KQ.(fldname).q{j},KQ.(fldname).dp(j)] = questalg(Kqin.(fldname),stat_ref,sig);
            
            dp = KQ.(fldname).dp(j);  
            if j>1                
                KQ.(fldname).q_opt(:,j) = questdebug(KQ.(fldname).q{j},KQ.(fldname).q_opt(:,j-1),dp);                
            else
                KQ.(fldname).q_opt(:,j) = KQ.(fldname).q_opt_q(:,j);
            end

            [Q.(fldname).q_opt_q(:,j),Q.(fldname).R{j}, Q.(fldname).res_UA{j},Q.(fldname).q{j},Q.(fldname).dp(j)] = questalg(qin.(fldname),stat_ref);
            
            dp = Q.(fldname).dp(j); 
            
            if j>1 
%                 [size(Q.(fldname).q{j}); size(Q.(fldname).q_opt(:,j-1))]
                Q.(fldname).q_opt(:,j) = questdebug(Q.(fldname).q{j},Q.(fldname).q_opt(:,j-1),dp);                
            else
                Q.(fldname).q_opt(:,j) = Q.(fldname).q_opt_q(:,j);
            end
        end
        
        Q.(fldname).q_ref = [0 0 0 1]';%questalg(ref.(fldname),stat_ref);
        KQ.(fldname).q_ref = [0 0 0 1]';%questalg(ref.(fldname),stat_ref);
        
end


%debug and output
thUA=zeros(1,size(Indata.(fldname).g,2));
thLA=zeros(1,size(Indata.(fldname).g,2));
thLANK=zeros(1,size(Indata.(fldname).g,2));
for j=1:size(Indata.(fldname).g,2)
    thUA(j) = 2*acos(KQ.L_upper_arm.q_opt(4,j))*180/pi;
    thLA(j) = 2*acos(KQ.L_lower_arm.q_opt(4,j))*180/pi;
    thLANK(j) = 2*acos(Q.L_lower_arm.q_opt(4,j))*180/pi;
end


la = 260;
lf = 250;
p_shL = [0;0;0];
p0_uL = la * ref.L_upper_arm.g / norm(ref.L_upper_arm.g);
p0_lL = lf * ref.L_lower_arm.g / norm(ref.L_lower_arm.g);


for i=1:size(Indata.(fldname).g,2)
    
    Kqin.L_upper_arm.g = Kdata.L_upper_arm.g(:,i);
    Kqin.L_upper_arm.m = Kdata.L_upper_arm.m(:,i);
    Kqin.L_lower_arm.g = Kdata.L_lower_arm.g(:,i);
    Kqin.L_lower_arm.m = Kdata.L_lower_arm.m(:,i);
    

%     [US.q_opt_q(:,i),US.R{i}, US.res{i}, US.q{i},US.dp(i)] =
%     questalg(Kqin.L_upper_arm,ref.Static_glob_cam);
    [US.q_opt_q(:,i),US.R{i}, US.res{i}, US.q{i},US.dp(i)] = questalg(Kqin.L_upper_arm,stat_ref);
    
    dp = US.dp(i); 
    if i>1                
        US.q_opt(:,i) = questdebug(US.q{i},US.q_opt(:,i-1),dp);                
    else
        US.q_opt(:,i) = US.q_opt_q(:,i);
    end
    
    
%     [LS.q_opt_q(:,i),LS.R{i}, LS.res{i}, LS.q{i}, LS.dp(i)] = questalg(Kqin.L_lower_arm,ref.Static_glob_cam);
    [LS.q_opt_q(:,i),LS.R{i}, LS.res{i}, LS.q{i}, LS.dp(i)] = questalg(Kqin.L_lower_arm,stat_ref);

    dp = LS.dp(i); 
    if i>1                
        LS.q_opt(:,i) = questdebug(LS.q{i},LS.q_opt(:,i-1),dp);                
    else
        LS.q_opt(:,i) = LS.q_opt_q(:,i);
    end
    
end


if strcmp(side,'right')
    upfld = 'R_upper_arm';
    lofld = 'R_lower_arm';
else
    upfld = 'L_upper_arm';
    lofld = 'L_lower_arm';
end


if debug
    
    figure
    subplot(3,2,1)
    hold on
    plot(Indata.(lofld).g(1,:),'g')
    plot(Kdata.(lofld).g(1,:),'k'),grid

    subplot(3,2,2)
    hold on
    plot(Indata.(lofld).m(1,:),'g')
    plot(Kdata.(lofld).m(1,:),'k'),grid

    subplot(3,2,3)
    hold on
    plot(Indata.(lofld).g(2,:),'g')
    plot(Kdata.(lofld).g(2,:),'k'),grid

    subplot(3,2,4)
    hold on
    plot(Indata.(lofld).m(2,:),'g')
    plot(Kdata.(lofld).m(2,:),'k'),grid

    subplot(3,2,5)
    hold on
    plot(Indata.(lofld).g(3,:),'g')
    plot(Kdata.(lofld).g(3,:),'k'),grid

    subplot(3,2,6)
    hold on
    plot(Indata.(lofld).m(3,:),'g')
    plot(Kdata.(lofld).m(3,:),'k'),grid

    % figure
    % vx = KQ.(lofld).q_opt(1,:)./sin(0.5*thLA*pi/180);
    % vy = KQ.(lofld).q_opt(2,:)./sin(0.5*thLA*pi/180);
    % vz = KQ.(lofld).q_opt(3,:)./sin(0.5*thLA*pi/180);
    % for i=1:length(vx)
    %     nv(i) = norm([vx(i) vy(i) vz(i)]);
    % end
    % plot(vx,'b')
    % hold on
    % plot(vy,'g')
    % plot(vz,'r')
    % plot(Kdata.(lofld).g(1,:)/10,'k')
    % grid
    % 
    % 
    % 
    % 
    % figure
    % plot(KQ.(lofld).q_opt(1,:),'b')
    % hold on
    % plot(KQ.(lofld).q_opt(3,:),'g')
    % plot(KQ.(lofld).q_opt(2,:),'r')
    % plot(KQ.(lofld).q_opt(4,:),'c')
    % plot(Kdata.(lofld).g(1,:)/10,'k')

    figure
    for i=1:4
        subplot(4,1,i)
        plot(KQ.(lofld).q_opt(i,:),'b'),grid
        hold on
        plot(Q.(lofld).q_opt(i,:),'r')
    end



    figure
    subplot(3,2,1)
    hist(Kdata.(lofld).eps(1,:))
    title('g')
    subplot(3,2,3)
    hist(Kdata.(lofld).eps(2,:))
    subplot(3,2,5)
    hist(Kdata.(lofld).eps(3,:))
    subplot(3,2,2)
    title('m')
    hist(Kdata.(lofld).eps(4,:))
    subplot(3,2,4)
    hist(Kdata.(lofld).eps(5,:))
    subplot(3,2,6)
    hist(Kdata.(lofld).eps(6,:))
    
end
    