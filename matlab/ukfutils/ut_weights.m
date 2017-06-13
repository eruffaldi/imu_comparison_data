% A novel 7 degrees of freedom model for upper limb kinematic reconstruction based on wearable sensors
% Lorenzo Peppoloni, Emanuele Ruffaldi,Alessandro Filippeschi - SSSA 2013

function [WM,WC,c] = ut_weights(n,alpha,beta,k)

lambda = alpha^2*(k+n)-n;
	  

WM = zeros(2*n+1,1);
WC = zeros(2*n+1,1);

for j=1:2*n+1
  if j==1
    wm = lambda / (n + lambda);
    wc = lambda / (n + lambda) + (1 - alpha^2 + beta);
  else
    wm = 1 / (2 * (n + lambda));
    wc = wm;
  end
  WM(j) = wm;
  WC(j) = wc;
end

c = n + lambda;
