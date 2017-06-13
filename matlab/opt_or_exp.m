%%% x,y are 3 x n
function [r,R] = opt_or_exp(x,y,w0)


w0 = w0/norm(w0);
r = fminunc(@(p) lsrotvec(p,x,y),w0);

R = exprot(r);


end


function zo = lsrotvec(w,x,y)

R = exprot(w);
z = zeros(size(x,2),1);
zz = y- R*x;
zo = sum(sum(zz(1,:).^2+zz(2,:).^2+zz(3,:).^2));

end

