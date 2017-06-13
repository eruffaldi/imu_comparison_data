%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function [a_tot, a_rel_loc, a_tot_glo] = LinAccEst(a_par,o,w,dw,q)

% first local inertial, then global and add origin acceleration


q_l2g = [-q(1:3); q(4)];
q_g2l = q;



Sw = skewmat(w);
Sdw = skewmat(dw);



% a_rel_loc = (w'*o)*w - o * (w'*w) + cross(dw,o);


a_rel_loc = (Sdw + Sw*Sw)*o;

% if norm(a_par)==0
%     a_par_rel = a_par;
% else
%     a_par_rel = quatrot(q_g2l,a_par); %a parent in local frame
% end


% a_rel_glo = quatrot(q,a_rel_loc);


a_tot = a_rel_loc(1:3) + a_par; %a_tot in global frame


a_tot_glo = quatrot(q_l2g,a_tot);




