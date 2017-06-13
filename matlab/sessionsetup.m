%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function [outfilename,steadydata,Tdata,ntdata,camref,alldata,allrundata,par,data,dataflds,side,Bidxs] = sessionsetup(capses,sesid,run)

if nargin<3
    run = [];
    Bidxs = 1024:10070;
else

    switch run  
        case 'L_EF'
            Bidxs = 1024:2624;
        case 'L_EP'
            Bidxs = 2625:3702;
        case 'L_SF'
            Bidxs = 3860-18:6490-18;
        case 'L_SA'
            Bidxs = 6491+31:8091+31;
        case 'L_SR'
            Bidxs = 8370:10070;
        otherwise
            Bidxs = 1024:10070;
    end
end

if strcmp(capses,'Aug12')
    
    mfilepath=fileparts(which(mfilename));
    addpath(fullfile(mfilepath,'../data'));
 
    
    
    steadydata = load ('UpS1.mat');
    steadydata = steadydata.q.imu;

    Tdata = load ('UpS4.mat');
    
    Tdata.imu = ExtractStructSeg(Tdata.q.imu,Tdata.q.seg.T);
    Tdata.vic = ExtractStructSeg(Tdata.q.vic,Tdata.q.seg.T);
    
    ntdata = ExtractStructSeg(Tdata.q.imu,Tdata.q.seg.N);

%     Bidxs = 1024:2624;
    if strfind(run,'L')
        side = 'left';
    else
        side = 'right';
    end

    filename = ['UpS' sesid '.mat'];
    outfilename = ['ResultsAug12_UpS' sesid '_' run '.mat'];
    % camref = camerarefdata(thrun,datafolder)
    camref = 0;

    alldata = load ([  filename]);
    allrundata = alldata;
    
    if isfield(alldata.q,'seg')
        if isempty(run)
            af = fieldnames(alldata.q.seg);
            ai = [];
            for i=1:length(af)
                ai = [ai; alldata.q.seg.(af{i})([1 end])];
            end
        alldata.imu = ExtractStructSeg(alldata.q.imu, min(ai(:,1)):max(ai(:,2)));
        alldata.vic = ExtractStructSeg(alldata.q.vic, min(ai(:,1)):max(ai(:,2)));
        end
            
        if isfield(alldata.q.seg,run)
            alldata.imu = ExtractStructSeg(alldata.q.imu,alldata.q.seg.(run));
            alldata.vic = ExtractStructSeg(alldata.q.vic,alldata.q.seg.(run));
        end
    else
        alldata.imu = alldata.q.imu;
        alldata.vic = alldata.q.vic;
    end

    switch filename       
        case {'UpS8.mat','UpS10.mat','UpS12.mat'}
            par.nidxs = 150:300;
            if strcmp(run,'L_SA')
                par.nidxs = 1:50;
            end
        case 'UpS20.mat'
            par.nidxs = 230:270;
        otherwise
            par.nidxs = 150:300;
    end

    par.timeidxs = 1;
    par.accidxs = 5:7;
    par.magidxs = 2:4;
    par.gyridxs = 11:13;
    par.dt = 0.01;
    
    par.la = 280;%270
    par.lf = 265;%260

    data = rmfield(alldata.imu,{'sensors','limbs','labels'});
    dataflds = fields(data);
    
    
elseif strcmp(capses,'sim')
    
%     addpath('/home/filippeschi/filippeschi/simulation')
    data=load('simdata.mat');

    Ndata.L_upper_arm.g = repmat(data.meas.a(1,:),size(data.meas.a(:,1),1),1)';
    Ndata.L_upper_arm.m = repmat(data.meas.m(1,:),size(data.meas.m(:,1),1),1)';
    Ndata.L_upper_arm.w = repmat(data.meas.g(1,:),size(data.meas.g(:,1),1),1)';
    
    Indata.L_upper_arm.g = repmat(data.meas.a(1,:),size(data.meas.a(:,1),1),1)';
    Indata.L_upper_arm.m = repmat(data.meas.m(1,:),size(data.meas.m(:,1),1),1)';
    Indata.L_upper_arm.w = repmat(data.meas.g(1,:),size(data.meas.g(:,1),1),1)';
    
    Indata.L_upper_arm.t = 0:0.01:length(data.meas.g(:,1));
    
    
    Ndata.L_lower_arm.g = data.meas.a(1:100,:)';
    Ndata.L_lower_arm.m = data.meas.m(1:100,:)';
    Ndata.L_lower_arm.w = data.meas.g(1:100,:)';
    
  
    Indata.L_lower_arm.g = data.meas.a';
    Indata.L_lower_arm.m = data.meas.m';
    Indata.L_lower_arm.w = data.meas.g';
    
    Indata.L_lower_arm.t = 0:0.01:length(data.meas.g(:,1));
      
    par.timeidxs = 1;
    par.accidxs = 5:7;
    par.magidxs = 2:4;
    par.gyridxs = 11:13;
    par.dt = 0.01;

    par.la = 260;
    par.lf = 260;
    
    camref = 0;
    
    side = 'left';
    
    steadydata = Ndata;
    ntdata = Ndata;
    Tdata = Ndata;
    alldata = Indata;
    data = Indata;
    dataflds = fields(data);
    
    outfilename = 'ResultsSimdata.mat';
        
end