%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function camref = camerarefdata(thrun,datafolder)

clc

if strfind(thrun,'_')
    imagedir = [thrun(1:6) filesep thrun(end)];
else
    imagedir = thrun(1:6);
end

rawimid = fopen ([datafolder filesep imagedir filesep 'image.txt']);
rawimscan = fread(rawimid,'*char');
rawimnl = strfind(rawimscan','img');
rawimei = strfind(rawimscan','2012 6 28');
rawimef = strfind(rawimscan',' 0 0 0');


img = zeros(length(rawimnl),2);
datein = cell(length(rawimnl),1);
dateinstr = cell(length(rawimnl),1);
for i=1:length(rawimnl)
    % get image time in seconds from 00:00:00 of 2012 6 28
    dateinstr{i} = rawimscan(rawimei(i)+10:rawimef(i)-1);
    g = strfind(dateinstr{i}',' ');
    datein{i} = str2num(dateinstr{i}');
    h = length(datein{i})-g(end);
    img (i,1) = str2double(rawimscan(rawimnl(i)+3:rawimnl(i)+7)); 
    img (i,2) = 3600*datein{i}(1) + 60*datein{i}(2) + datein{i}(3) + datein{i}(4)/(10*h);
%     img (i,2) = str2double(rawimscan(rawimnl(i)+15:rawimet(i)))*1.0e-7;
end


a = fopen('run008_output.txt');

b = fscanf(a,'%s');

c = strfind(b,'/home');
d = strfind(b,'img0');

len = length(c);
e = cell(len);
for i=1:len-1
    
    e{i} = b(d(i)+3:c(i+1)-1);
    
end

e{len} = b(d(len)+3:end);

f.Shoulder = [];
f.Elbow = [];
f.Wrist = [];

for i=1:length(e)
    imgnum = str2double(e{i}(1:5));
    jointpos = e{i}(10:end-6);
    j_width = str2double(e{i}(end-5:end-3));
    j_height = str2double(e{i}(end-2:end));   
    f.(jointpos) = [f.(jointpos) [imgnum j_width j_height]'];
end

chkpoints = size(f.(jointpos),2);
ua = zeros(2,chkpoints);
fa = zeros(2,chkpoints);
nua = zeros(2,chkpoints);
nfa = zeros(2,chkpoints);


for i=1:chkpoints
    
    timeidxs = img(:,1)==f.Shoulder(1,i);
    
    camref.time(i) = (img(timeidxs,1)-img(1,1))*0.05;
    
    ua(:,i) = f.Shoulder(2:3,i) - f.Elbow(2:3,i);
    fa(:,i) = f.Elbow(2:3,i) - f.Wrist(2:3,i);
    
    nua(:,i) = ua(:,i)/norm(ua(:,i));
    nfa(:,i) = fa(:,i)/norm(fa(:,i));
    camref.img(i) = f.Shoulder(1,i);
    
    camref.ang.el(:,i) = acos(dot(nua(:,i),nfa(:,i)));
    
end







