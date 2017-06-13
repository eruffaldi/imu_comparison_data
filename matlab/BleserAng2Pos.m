%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function [ang,pos] = BleserAng2Pos(par,idxs)

% addpath('/home/norbert/filippeschi/simulation')
addpath('C:\Users\afilippeschi.RowingOnTheRoad\Documents\DFKI\repos\sim\simulation')

load('BleserUpper.mat')

la = par.la;
lf = par.lf;

% idxs=1:2500;
% 
% la = 260;
% lf = 270;

p_uL_0 = [0 0 la]';
p_lL_0 = [0 0 lf]';


p_ul = zeros(3,length(idxs));
p_ll1 = zeros(3,length(idxs));
assignin('base','UA_an',idxs)
for i=1:length(idxs)
    
    UA_an = r.Left.data(1:3,idxs(i));
    LA_an = [0; r.Left.data(4,idxs(i)); r.Left.data(5,idxs(i))]; 
    
    [R_UA,not_used_a] = RPYTrasf(UA_an,'zyx');
    [R_LA,not_used_b] = RPYTrasf(LA_an,'zyx');
    
    p_ul(:,i) = R_UA*p_uL_0;
    p_ll1(:,i) = R_LA*R_UA*p_lL_0;   
    
end

p_ll = p_ul + p_ll1;

pos.Lelbow=p_ul;
pos.Lwrist=p_ll;
ang.Lelbow = r.Left.data(4,idxs(i));

figure
plot(p_ul'),grid
figure
plot(p_ll1'),grid


% figure
% plot3(p_ul(1,:),p_ul(2,:),p_ul(3,:),'b')
% hold on
% plot3(p_ll(1,:),p_ll(2,:),p_ll(3,:),'r')
% grid