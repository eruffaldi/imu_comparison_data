function [Rr,ny,e] = opt_or(x,y,R0)
% 
% clc
% R0=eye(3)
% th=45*pi/180;
% 
% x(:,1) = [1 0 0];
% x(:,2) = [1 1 0];
% x(:,3) = [-0.5 0.5*3^0.5 0];
% x(:,4) = [0 -1 0];
% 
% R45 = [cos(th) -sin(th) 0; cos(th) sin(th) 0; 0 0 1];
% 
% y = R45*x;
% r0 = [1 0 0 0 1 0 0 0 1]';
% clc

B = zeros(9,1);
for j=1:3
    for k=1:3      
        for i=1:size(x,2)            
            B(3*(j-1)+k) = B(3*(j-1)+k) + y(j,i)*x(k,i);
        end
    end
end

r0 = [R0(1:3,1); R0(1:3,2); R0(1:3,3)];


myfun = @(r)-r'*B;
r = fmincon(myfun,r0,[],[],[],[],-ones(9,1),ones(9,1),@opt_or_cons);

Rr = [r(1:3) r(4:6) r(7:9)]';
ny1 = Rr*x;
ny2 = -Rr*x;

e1 = 0;
e2 = 0;

for i=1:size(x,2)
    e1 = e1 + norm(y(:,i)-ny1(:,i));
    e2 = e2 + norm(y(:,i)-ny2(:,i));
end


if e1>e2
    Rr = -Rr;
    ny = ny2;
    e = e2;
else
    ny = ny1;
    e = e1;
end

function [c, ceq] = opt_or_cons(x)
    ceq(1) = x(1)*x(2)+x(4)*x(5)+x(7)*x(8);
    ceq(2) = x(1)*x(3)+x(4)*x(6)+x(7)*x(9);
    ceq(3) = x(2)*x(3)+x(5)*x(6)+x(8)*x(9);
    ceq(4) = x(1)^2+x(4)^2+x(7)^2-1;
    ceq(5) = x(2)^2+x(5)^2+x(8)^2-1;
    ceq(6) = x(3)^2+x(6)^2+x(9)^2-1;

    c=[];
