%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function [Indata, CovMat, Ndata, statref, Tdata] = AugDataSetup(data,par,dataflds,kcov,steadydata, ntdata, iTdata, Moff)

% Indata = 0;
% CovMat = 0;
% Ndata = 0;
timeidxs = par.timeidxs;
accidxs = par.accidxs; 
magidxs = par.magidxs; 
gyridxs = par.gyridxs;


nidxs = par.nidxs;
dt = par.dt;

% static refernce with head mounted cam at beginning (session 1)
statref.g = mean(steadydata.cam_imu(:,par.accidxs))';
statref.m = mean(steadydata.cam_imu(:,par.magidxs))';
statref.m = statref.m - repmat(Moff.cam_imu,1,size(statref.m,2));

for i=1:length(dataflds)
    
    fldname = dataflds{i};
    
    if isnumeric(data.(fldname)) 
        
        if ~isfield(kcov,fldname)
            kcov.(fldname).meas.g = 1;
            kcov.(fldname).meas.m = 1;
            kcov.(fldname).meas.w = 1;
            kcov.(fldname).proc = 1;
        end
        
        % Covariance from N-pose data
%         nanidxs = sum(ntdata.(fldname)(:,2:10),2)==0;
%         ntdata.(fldname)(nanidxs,2:10) = nan(sum(nanidxs),9);
        
        Ndata.(fldname).g = ntdata.(fldname)(nidxs,accidxs)';
        Ndata.(fldname).m = ntdata.(fldname)(nidxs,magidxs)';
        Ndata.(fldname).m = Ndata.(fldname).m - repmat(Moff.(fldname),1,size(Ndata.(fldname).m,2));
        Ndata.(fldname).w = ntdata.(fldname)(nidxs,gyridxs)';
        Tdata.(fldname).g = iTdata.imu.(fldname)(:,accidxs)';
        Tdata.(fldname).m = iTdata.imu.(fldname)(:,magidxs)';
        Tdata.(fldname).m = Tdata.(fldname).m - repmat(Moff.(fldname),1,size(Tdata.(fldname).m,2));
        Tdata.(fldname).w = iTdata.imu.(fldname)(:,gyridxs)';
        CovMat.(fldname).g = kcov.(fldname).meas.g*cov(Ndata.(fldname).g');
        CovMat.(fldname).m = kcov.(fldname).meas.m*cov(Ndata.(fldname).m');
        CovMat.(fldname).w = kcov.(fldname).meas.w*cov(Ndata.(fldname).w');
        CovMat.(fldname).proc = kcov.(fldname).proc; %run8a diag([0.002 0.002 0.002]) 
        
        % remove drops and average from gyro
        if nargin<5
            Indata.(fldname).wav = [0 0 0];
        else
            Indata.(fldname).wav = mean(steadydata.(fldname)(:,gyridxs))';
        end
        

        for j=1:size(data.(fldname),1)
            if data.(fldname)(j,timeidxs)==0 && j>1 %norm(data.(fldname)(j,accidxs))==0 && j>1
                Indata.(fldname).g(:,j) = data.(fldname)(j-1,accidxs)';
                Indata.(fldname).m(:,j) = data.(fldname)(j-1,magidxs)'-Moff.(fldname);
                Indata.(fldname).w(:,j) = data.(fldname)(j-1,gyridxs)';
                Indata.(fldname).t(j) = data.(fldname)(j-1,timeidxs)+dt;
                                            
            else
                Indata.(fldname).g(:,j) = data.(fldname)(j,accidxs)';
                Indata.(fldname).m(:,j) = data.(fldname)(j,magidxs)'-Moff.(fldname);
                Indata.(fldname).w(:,j) = data.(fldname)(j,gyridxs)';
                Indata.(fldname).t(j) = data.(fldname)(j,timeidxs);
            end
        end
        
        Indata.(fldname).dw =[zeros(1,3); diff(Indata.(fldname).w')/dt]';
    end
    
end