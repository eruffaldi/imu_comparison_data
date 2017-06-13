%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function y = pw_norm(x,dim)

if nargin<2
    dim=1;
end

if dim==1
    %l=size(x,dim);
    %y=zeros(l,1);
    y = sqrt(sum(x.^2,2));
else
    %y=zeros(1,l);
    %x = x';
    y = sqrt(sum(x.^2,1));
end

%for i=1:l
%    y(i)=norm(x(i,:));
%end