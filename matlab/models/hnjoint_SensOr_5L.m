% A novel 7 degrees of freedom model for upper limb kinematic reconstruction based on wearable sensors
% Lorenzo Peppoloni, Emanuele Ruffaldi,Alessandro Filippeschi - SSSA 2013

% function [ e ] = hnjoint_SensOr_5L( x, Y, q,params )
    function [eo] = hnjoint_SensOr_5L( x,Y,q,params )
    
    % params = [luaL lfaL m0 x1L p1L m0 x2L p2L];

    T0 = [-1 0 0;0 0 1;0 1 0];

    xr = x;
    
    g = [0 0 -1]';
    e = zeros(size(q,1),1);
    for j=1:size(q,1)

        
        qL = q(j,:);
%         qL=[q(j,1) x(7) q(j,3:4)];%***
%         qL=[q(j,1) x(7) q(j,3:4) x(8)];%****
        y = Y(:,j);
        %Compute parents to sensors transformations
        
%         % Torso sensor son of {1}
%         
%         t2 = qj(2)+xr(1);
%         y2 =  xr(2);
%         x2 = xr(3);
%         S1 = [cos(t2) -sin(t2) 0 0;sin(t2) cos(t2) 0 0;0 0 1 0;0 0 0 1];
%         S2 = [cos(y2) 0 sin(y2) 0; 0 1 0 0;-sin(y2) 0 cos(y2) 0; 0 0 0 1];
%         S3 = [1 0 0 0;0 cos(x2) -sin(x2) 0;0 sin(x2) cos(x2) 0; 0 0 0 1];
%         S0 = S1*S2*S3;
        
        % Left Arm Sensors sons of {4l} and {6l}
        
%         t = 1*qL(1)+xr(1);
        t = 1*qL(3)+xr(1);
        y2 =  xr(2);
        x2 =  xr(3);
        S1 = [cos(t) -sin(t) 0 0;sin(t) cos(t) 0 0;0 0 1 0;0 0 0 1];
        S2 = [cos(y2) 0 sin(y2) 0; 0 1 0 0;-sin(y2) 0 cos(y2) 0; 0 0 0 1];
        S3 = [1 0 0 0;0 cos(x2) -sin(x2) 0;0 sin(x2) cos(x2) 0; 0 0 0 1];
        S1L = S1*S2*S3;
        
%         t = 1*qL(3)+xr(4);
        t = 1*qL(5)+xr(4);
        y2 =  xr(5);
        x2 =  xr(6);
        S1 = [cos(t) -sin(t) 0 0;sin(t) cos(t) 0 0;0 0 1 0;0 0 0 1];
        S2 = [cos(y2) 0 sin(y2) 0; 0 1 0 0;-sin(y2) 0 cos(y2) 0; 0 0 0 1];
        S3 = [1 0 0 0;0 cos(x2) -sin(x2) 0;0 sin(x2) cos(x2) 0; 0 0 0 1];
        S2L = S1*S2*S3;
        
%         [T1, TR, TL] = DHCh([qj(1) qj(2) qR qL], params(1:2));
        TL = DHCh(qL, params(1:2));
 
%         Sg0 = T0*T1(1:3,1:3);
%         Sg1R = T0*T1(1:3,1:3)*TR(1:3,1:3)*TR(1:3,5:7)*TR(1:3,9:11)*S1R(1:3,1:3);
%         Sg2R = T0*T1(1:3,1:3)*TR(1:3,1:3)*TR(1:3,5:7)*TR(1:3,9:11)*TR(1:3,13:15)*TR(1:3,17:19)*S2R(1:3,1:3);
    
        Sg1L = T0*TL(1:3,1:3)*TL(1:3,5:7)*TL(1:3,9:11)*S1L(1:3,1:3);
        Sg2L = T0*TL(1:3,1:3)*TL(1:3,5:7)*TL(1:3,9:11)*TL(1:3,13:15)*TL(1:3,17:19)*S2L(1:3,1:3);
        
%         if j==1
%             [Sg1L Sg2L];
%         end
%         ddxs = [Sg0(1:3,1:3)'*g;Sg1R(1:3,1:3)'*g;Sg2R(1:3,1:3)'*g;Sg1L(1:3,1:3)'*g;Sg2L(1:3,1:3)'*g];
        ddxs = [Sg1L(1:3,1:3)'*g;Sg2L(1:3,1:3)'*g];
%         size(y), size(ddxs)
        e(j) = norm(y-ddxs);
        
    end
    clc
%     [ddxs y]

    eo = norm(e);
%      sprintf('q2=%d',x(7))
    
    
end




