%%% omega = [w_x w_y w_z] 
function R = exprot(omega)
            theta = norm(omega);
%             if theta~=0
%                 omega = omega/theta;
%             end
            A = sin(theta)/theta;
            B = (1-cos(theta))/(theta^2);
            C = (1-A)/(theta^2);
            S = skewmat(omega);
            
            R = eye(3) + A*S + B*S^2;
%             x = omega(1); y = omega(2); z = omega(3);
%             s = sin(theta/2); c = cos(theta/2);
%             R = [2*(x^2-1)+1 2*x*y*s^2-2*z*c*s 2*x*z*s^2+2*y*c*s
%                 ]
end