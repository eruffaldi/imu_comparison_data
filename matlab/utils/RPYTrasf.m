function [R,w_g] = RPYTrasf(an,rotord,dang)

for i=1:3
    c(i) = cos(an(i));
    s(i) = sin(an(i));
end

switch rotord
    case 'zyx'
        R3 = [1 0 0; 0 c(1) -s(1);0 s(1) c(1)];
        R2 = [c(2) 0  s(2);0 1 0; s(2) 0 c(2)];
        R1 = [c(3) -s(3) 0; s(3) c(3) 0; 0 0 1];
    case 'xyz'
        R3 = [c(1) -s(1) 0; s(1) c(1) 0; 0 0 1];
        R2 = [c(2) 0  s(2);0 1 0; -s(2) 0 c(2)];
        R1 = [1 0 0; 0 c(3) -s(3);0 s(3) c(3)];
        
end

R = R1*R2*R3;

if nargin>2
    switch rotord
    case 'zyx'
        w_g = [0;0;dang(3)] + R1*[0;dang(2);0] + R1*R2*[dang(1);0;0];
    otherwise
        w_g = zeros(3,1);
    end
else
    w_g = zeros(3,1);
    
end
    