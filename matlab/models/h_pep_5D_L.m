% A novel 7 degrees of freedom model for upper limb kinematic reconstruction based on wearable sensors
% Alessandro Filippeschi, Emanuele Ruffaldi, Lorenzo Peppoloni - SSSA 2013
function [Y,pos] = h_pep_5D_L( x, params )

%%%params
%arms
luaL = params(1);
lfaL = params(2);

%S1
s_or_11 = params(3); s_or_12 = params(4); s_or_13 = params(5);
s_po_11 = params(6); s_po_12 = params(7); s_po_13 = params(8);
m0_11 = params(9); m0_12 = params(10); m0_13 = params(11);

%S2
s_or_21 = params(12); s_or_22 = params(13); s_or_23 = params(14);
s_po_21 = params(15); s_po_22 = params(16); s_po_23 = params(17);
m0_21 = params(18); m0_22 = params(19); m0_23 = params(20);

%model
Z0 = [0 0 1]';
q = x(1:3:15);
dq = x(2:3:15);
ddq = x(3:3:15);
T0 = [-1 0 0 0;0 0 1 0;0 1 0 0;0 0 0 1];
g0 = [0;0;-9.81];

%%%KIN Chain
% Frame T1 as children of T0 using q1 
ct = cos(q(1)+0); st = sin(q(1)+0);
ca = cos(pi/2); sa = sin(pi/2);
A1 = [ct -st*ca st*sa 0*ct; st ct*ca -ct*sa 0*st; 0 sa ca 0; 0 0 0 1];
T1 = T0*A1;
p = A1(1:3,1:3)'*A1(1:3,4);
Rpi = A1(1:3,1:3)';
omega1 = Rpi*(dq(1)*Z0);
domega1 = Rpi*(ddq(1)*Z0);
dx1 = cross2(omega1,p);
ddx1 = cross2(domega1,p)+cross2(omega1,dx1);

% Frame T2 as children of T1 using q2 
ct = cos(q(2)-pi/2); st = sin(q(2)-pi/2);
ca = cos(pi/2); sa = sin(pi/2);
A2 = [ct -st*ca st*sa 0*ct; st ct*ca -ct*sa 0*st; 0 sa ca 0; 0 0 0 1];
T2 = T1*A2;
p = A2(1:3,1:3)'*A2(1:3,4);
Rpi = A2(1:3,1:3)';
omega2 = Rpi*(omega1 + dq(2)*Z0);
domega2 = Rpi*(domega1 + dq(2)*cross2(omega1,Z0)+ddq(2)*Z0);
% dx2 = Rpi*dx1 + cross2(omega2,p);
ddx2 = Rpi*ddx1 + cross2(domega2,p)+cross2(omega2,cross2(omega2,p));

% Frame T3 as children of T2 using q3 
ct = cos(q(3)+pi/2); st = sin(q(3)+pi/2);
ca = cos(0); sa = sin(0);
A3 = [ct -st*ca st*sa luaL*ct; st ct*ca -ct*sa luaL*st; 0 sa ca 0; 0 0 0 1];
T3 = T2*A3;
p = A3(1:3,1:3)'*A3(1:3,4);
Rpi = A3(1:3,1:3)';
omega3 = Rpi*(omega2 + dq(3)*Z0);
domega3 = Rpi*(domega2 + dq(3)*cross2(omega2,Z0)+ddq(3)*Z0);
% dx3 = Rpi*dx2 + cross2(omega3,p);
ddx3 = Rpi*ddx2 + cross2(domega3,p)+cross2(omega3,cross2(omega3,p));
Rpi_s1 = Rpi;

% Frame T4 as children of T3 using q4 
ct = cos(q(4)+pi/2); st = sin(q(4)+pi/2);
ca = cos(pi/2); sa = sin(pi/2);
A4 = [ct -st*ca st*sa 0*ct; st ct*ca -ct*sa 0*st; 0 sa ca 0; 0 0 0 1];
T4 = T3*A4;
p = A4(1:3,1:3)'*A4(1:3,4);
Rpi = A4(1:3,1:3)';
omega4 = Rpi*(omega3 + dq(4)*Z0);
domega4 = Rpi*(domega3 + dq(4)*cross2(omega3,Z0)+ddq(4)*Z0);
% dx4 = Rpi*dx3 + cross2(omega4,p);
ddx4 = Rpi*ddx3 + cross2(domega4,p)+cross2(omega4,cross2(omega4,p));

% Frame T5 as children of T4 using q5 
ct = cos(q(5)+pi/2); st = sin(q(5)+pi/2);
ca = cos(pi/2); sa = sin(pi/2);
A5 = [ct -st*ca st*sa 0*ct; st ct*ca -ct*sa 0*st; 0 sa ca lfaL; 0 0 0 1];
T5 = T4*A5;
% p = A5(1:3,1:3)'*A5(1:3,4);
Rpi = A5(1:3,1:3)';
Rpi_s2 = Rpi;
% omega5 = Rpi*(omega4 + dq(5)*Z0);
% domega5 = Rpi*(domega4 + dq(5)*cross2(omega4,Z0)+ddq(5)*Z0);
% dx5 = Rpi*dx4 + cross2(omega5,p);
% ddx5 = Rpi*ddx4 + cross2(domega5,p)+cross2(omega5,cross2(omega5,p));

% SH Sensor S1 as T1001 children of T1 using q2 (sibling of T2)
p = [s_po_11 s_po_12 s_po_13]';
zs = q(3)+s_or_11;
ys = s_or_12;
xs = s_or_13;
Sa = [cos(zs) -sin(zs) 0 0;sin(zs) cos(zs) 0 0;0 0 1 0;0 0 0 1];
Sb = [cos(ys) 0 sin(ys) 0; 0 1 0 0;-sin(ys) 0 cos(ys) 0; 0 0 0 1];
Sc = [1 0 0 0;0 cos(xs) -sin(xs) 0;0 sin(xs) cos(xs) 0; 0 0 0 1];
Sd = [eye(3) p; zeros(1,3) 1];
% Sd = [1 0 0 s_po_11; 0 1 0 s_po_12; 0 0 1 s_po_13; 0 0 0 1];
A1001 = Sa*Sb*Sc*Sd;
% p = A1001(1:3,1:3)'*A1001(1:3,4);
% p = A1001(1:3,4);

Rpi = A1001(1:3,1:3)';
T_s11 = T3*A1001;
omega_s11 = Rpi*Rpi_s1*(omega2 + dq(3)*Z0);
domega_s11 = Rpi*Rpi_s1*(domega2 + dq(3)*cross2(omega2,Z0)+ddq(3)*Z0);
% dx1001 = Rpi*dx1 + cross2(omega1001,p);
ddx_s11 = Rpi*Rpi_s1*ddx2 + cross2(domega_s11,p)+cross2(omega_s11,cross2(omega_s11,p));
% assignin('base','T_s11',T_s11)
% SH Sensor S2 as T1002 children of T4 using q5 (sibling of T5)
p = [s_po_21 s_po_22 s_po_23]';
zs = q(5)+s_or_21;
ys = s_or_22;
xs = s_or_23;
Sa = [cos(zs) -sin(zs) 0 0;sin(zs) cos(zs) 0 0;0 0 1 0;0 0 0 1];
Sb = [cos(ys) 0 sin(ys) 0; 0 1 0 0;-sin(ys) 0 cos(ys) 0; 0 0 0 1];
Sc = [1 0 0 0;0 cos(xs) -sin(xs) 0;0 sin(xs) cos(xs) 0; 0 0 0 1];
Sd = [eye(3) p; zeros(1,3) 1];
% Sd = [1 0 0 s_po_21; 0 1 0 s_po_22; 0 0 1 s_po_23; 0 0 0 1];
A1002 = Sa*Sb*Sc*Sd;
% p = A1002(1:3,1:3)'*A1002(1:3,4);
% p = A1002(1:3,4);

Rpi = A1002(1:3,1:3)';
T_s12 = T5*A1002;
omega_s12 = Rpi*Rpi_s2*(omega4 + dq(5)*Z0);
domega_s12 = Rpi*Rpi_s2*(domega4 + dq(5)*cross2(omega4,Z0)+ddq(5)*Z0);
% dx1002 = Rpi*dx4 + cross2(omega1002,p);
ddx_s12 = Rpi*Rpi_s2*ddx4 + cross2(domega_s12,p)+cross2(omega_s12,cross2(omega_s12,p));

%%%SENSOR outputs
gg = 9.81;
Y = [...
omega_s11; (ddx_s11+T_s11(1:3,1:3)'*g0)/gg; T_s11(1:3,1:3)'*[m0_11,m0_12,m0_13]';
omega_s12; (ddx_s12+T_s12(1:3,1:3)'*g0)/gg; T_s12(1:3,1:3)'*[m0_21,m0_22,m0_23]'];

% Y = [...
% omega_s11; (ddx_s11+T_s11(1:3,1:3)'*g0)/gg;
% omega_s12; (ddx_s12+T_s12(1:3,1:3)'*g0)/gg];

pos.elbow = T3(1:3,4);
pos.wrist = T5(1:3,4);


end