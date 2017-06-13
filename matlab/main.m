%%% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%

addpath ukfutils
addpath utils
addpath models

%%
clear,clc, close all
tic

capses = 'Aug12'; % Capture Session
sesid = '10'; % the only worth
run = 'L_SA';

debug = 0; % more plots
animate = 0; % 
saveen = 0; % stores the results on matlab file ()



[outfilename,steadydata,oTdata,ntdata,camref,alldata,allrundata,par,data,dataflds,side,Bidxs] = sessionsetup(capses,sesid,run);


%% Define models    
models = {'zhu','yun','young','pep'}; %zhu, yun, young
% models = {'zhu','pep'}; %zhu, yun, young
% models = {'zhu','yun','young','pep'}; %zhu, yun, young
% models = {'pep'}; %zhu, yun, young
% Zhuver = {'original','firstorder','secondorder'}; % original, firstorder, secondorder
Zhuver = {'original'}; % original, firstorder, secondorder
Yunver = {'original'};
% Youngver = {'pure complementary','pure kalman','perfect complementary','perfect kalman'}; %perfect/pure vs. complementary/kalman
Youngver = {'pure complementary', 'perfect complementary'}; %perfect/pure vs. complementary/kalman
Pepver = {'original'};

Allids = {'Zhu_or','Zhu_fi','Zhu_se','Yun_or','You_pu_c','You_pu_k','You_pe_c','You_pe_k','Pep_or'}; %TO BE UPDATED if new models/versions are added

%% compute models

for M=1:length(models)
    
    model = models{M}
    
    if strcmp(model,'zhu')
        versions = Zhuver;
        resmod = 'zhu';
        lV = length(versions);  
    elseif strcmp(model,'yun')
        versions = Yunver;
        resmod = 'yun';
        lV = length(versions);
    elseif strcmp(model,'young')
        versions = Youngver;
        resmod = 'you';
        lV = length(versions);
    elseif strcmp(model,'pep')
        versions = Pepver;
        resmod = 'pep';
        lV = length(versions);
    elseif strcmp(model,'peps')
        versions = Pepver;
        resmod = 'peps';
        lV = length(versions);
    end
    
    for V=1:lV
        
        version = versions{V}

        %identifiers
        if strcmp(model,'zhu')
            switch version
                case 'original'
                    id = 'Zhu_or';
                case 'firstorder'
                    id = 'Zhu_fi';
                case 'secondorder'
                    id = 'Zhu_se';
            end

        elseif strcmp(model,'yun')
            switch version
                case 'original'
                    id = 'Yun_or';
            end

        elseif strcmp(model,'young')
            switch version
                case 'pure complementary'
                    id = 'You_pu_c';
                case 'pure kalman'
                    id = 'You_pu_k';
                case 'perfect complementary'
                    id = 'You_pe_c';            
                case 'perfect kalman'
                    id = 'You_pe_k';
            end
            
        elseif strcmp(model,'pep')
            switch version
                case 'original'
                    id = 'Pep_or';
            end
        elseif strcmp(model,'peps')
            switch version
                case 'original'
                    id = 'Peps_or';
            end
        end
        
        Kcov = sel_Cov_pars(dataflds,Allids,id);
        
        % correct magnetometer
        for i=1:length(dataflds)
            fld = dataflds{i};
            magdata = allrundata.q.imu.(fld)(:,2:4);
            magn = pw_norm(magdata);
            magdata = magdata(magn>0,:);
            [Moff.(fld), MRad.(fld)] = ellipsoid_fit(magdata);
        end
        
        if strcmp(capses,'Jun12')
            [Indata, CovMat, Ndata, statref] = JunDataSetup(data,par,dataflds,Kcov);%steadydata to be added as last input
        elseif strcmp(capses,'Aug12')
            [Indata, CovMat, Ndata, statref, Tdata] = AugDataSetup(data,par,dataflds,Kcov,steadydata, ntdata,oTdata,Moff);
        elseif strcmp(capses,'sim')
            Ndata = steadydata;
            Indata = data;
            statref.g = [mean(steadydata.L_lower_arm.g')]';
            statref.m = [mean(steadydata.L_lower_arm.m')]';
            CovMat.L_lower_arm.w = diag([0.00004 0.00004 0.00004]);
            CovMat.L_upper_arm.w = diag([0.00004 0.00004 0.00004]);
        end
        
        
%        [Rgm, Gra, Mag] = g2mRot(steadydata,Ndata,Tdata,'L_lower_arm',par);
       
% Indata contains all input data (most important !!!)
        time = Indata.L_lower_arm.t;
        
        toc

        if strcmp(model,'zhu')
            

            [OZhu.Q, OZhu.KQ, OZhu.ref, OZhu.Kdata] = zhumodel(statref,Indata, CovMat, Ndata, version,side, debug);

            % angles are not working (for august session -- check required) position is fine
            [Zhu.(id).ang, Zhu.(id).pos] = model_pos_ang(time,OZhu.ref,camref,OZhu.KQ,par,side,animate);
            title([model '_' version])
            drawnow

            toc

        
        % YUN MODEL
        elseif strcmp(model,'yun')
            
            

            
            [Yun_ref,YunQ] = yunmodel(capses,statref,Ndata,par,CovMat,Indata,debug);

            [Yun.(id).ang,Yun.(id).pos] = model_pos_ang(time,Yun_ref,camref,YunQ,par,side,animate);
            title([model '_' version])
            drawnow       
            toc


        % YOUNG
        elseif strcmp(model,'young') 
            
            

            [Young_ref,YoungQ] = youngmodel(version,statref,Ndata,par,Indata,debug);        

            [Young.(id).ang,Young.(id).pos] = model_pos_ang(time,Young_ref,camref,YoungQ,par,side,animate); 
            title([model '_' version])
            
            toc
         
        %PEP    
        elseif strcmp(model,'pep')
            debug=1;
            tic
            CovMat.covEps = Kcov.covEps;
            CovMat.covQ = Kcov.covQ;
            [Pep.(id).ang,Pep.(id).pos, Z, ukf_p, Pep.(id).cov ] = pepmodel(par,Ndata,Tdata,CovMat,Indata,debug,0);
            title([model '_' version])
            toc
        elseif strcmp(model,'peps')
            debug=1;
            
            CovMat.covEps = Kcov.covEps;
            CovMat.covQ = Kcov.covQ;
            [Pep.(id).ang,Pep.(id).pos, Z, ukf_p, Pep.(id).cov ] = pepmodel(par,Ndata,Tdata,CovMat,Indata,debug,1);
            title([model '_' version])
            toc
        end
        
        % Store for comparison     
        if strcmp(resmod, 'zhu')
            resdata.(model).(id) = Zhu.(id);
        end
        if strcmp(resmod, 'yun')
            resdata.(model).(id) = Yun.(id);
        end
        if strcmp(resmod, 'you')
            resdata.(model).(id) = Young.(id);
        end  
        if strcmp(resmod, 'pep')
            resdata.(model).(id) = Pep.(id);
        end  
        if strcmp(resmod, 'peps')
            resdata.(model).(id) = Pep.(id);
        end  

    end
end

toc
%% debug
% clc
% ndata = [(mean(Ndata.L_upper_arm.w'))'; (mean(Ndata.L_upper_arm.g')/9.81)'; (mean(Ndata.L_upper_arm.m'))'; (mean(Ndata.L_lower_arm.w'))'; (mean(Ndata.L_lower_arm.g')/9.81)'; (mean(Ndata.L_lower_arm.m'))'];
% thx = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
% [h_pep_5D_L(thx, ukf_p)  (mean(Z(:,1:50)')') ndata (mean(Z(:,265:266)')')]
% 
% thx = [pi/2 1 0 0 0 0 0 0 0 0 0 0 0 0 0];
% [h_pep_5D_L(thx, ukf_p) (mean(Z(:,197:199)')')]

%% get Gabi results

if strcmp(capses,'Aug12')
    [Bleser.or.ang,Bleser.or.pos] = BleserAng2Pos(par,Bidxs);
    resdata.Bleser.or = Bleser.or;
end

toc

% resdata.ref.time = time;
% resdata.ref.cam = camref;

%% Comapre data against reference
close all
clc

if isfield(alldata,'vic')
    resdata.ref.vic = alldata.vic;
    resdata = alignment(resdata,run);    
end

% resdata.pep.Pep_or.alig.RotMat
% resdata.pep.Pep_or.alig.RotMatExp
% =======
%     resdata = alignment(resdata);    
% else
%     resdata = alignment(resdata);
% end

%resdata.pep.Pep_or.alig.RotMat!op
%resdata.pep.Pep_or.alig.RotMatExp

% [resdata2, ods] = compute_err(resdata);
% >>>>>>> ca70b8ec41c63e6fb713f5ec707945500a8a7284

%% save data
% if saveen
%     save(outfilename,'resdata')
% end


%% load data and retrieve models and versions

% clear, clc
% 
% load models_pos_est_V_280415
% 
% resdata = rm_resdata_fld(resdata);

%% align estimation to vicon

resdata1 = alignment(resdata,run);

%% compute error measures
clc
clear ods
[resdata2, ods] = compute_err(resdata1);

%% plot data for each model
% plot_imumod_resdata(resdata2);


%% plot data from dataset
camready = 0;
plot_imumod_resds%(ods)


%% figures for TRO

%alignment
%clear, clc, close all
load model_res_290715_1314


npidxs = resdata2.pep.Pep_or.npidxs;
compidxs = resdata2.pep.Pep_or.compidxs;

Qv = resdata2.pep.Pep_or.alig.ref(compidxs,2:4);
Qt0 = resdata2.pep.Pep_or.pos.Lwrist';
Qt = Qt0-repmat(mean(Qt0(npidxs,:)),size(Qt0,1),1);
Qt = Qt(compidxs,:);
Qe = resdata2.pep.Pep_or.alig.posE(compidxs,:);

fs = 20;
ha = figure('color','none','Position',[1 1 1600 800],'defaulttextinterpreter','tex');

plot3(Qv(:,1),Qv(:,2),Qv(:,3),'Color',[1 0 0],'LineWidth',1.5)
hold on
plot3(Qt0(:,1),Qt0(:,2),Qt0(:,3),'Color',[0 0 0],'LineWidth',1)
plot3(Qt(:,1),Qt(:,2),Qt(:,3),'.k','MarkerSize',3.8)
plot3(Qe(:,1),Qe(:,2),Qe(:,3),'--k','LineWidth',1.5)
grid
xlabel('x [mm]','FontSize',fs), ylabel('y [mm]','FontSize',fs), zlabel('z [mm]','FontSize',fs)
legend({'OMC data X','IMU-based Y','IMU-based Y','IMU-based Z'},'Position', [0.75, 0.70, 0.16, 0.21])
set(gca,'FontSize',fs)



%% table EF
% clear, clc
run = 'L_EF';

switch run
    case 'L_EF'
%         load  model_res_300715_1041.mat
        
    case 'L_SA'
        load model_res_310715_0908SA.mat

    case 'L_SF'
        load model_res_300715_1956SF.mat
end
camready = 1;
% plot_imumod_resds

ods_per = ods(ods.period>0,:);

per_ds = dataset();
per_ds.model = ods_per.model;
per_ds.err_per = ods_per.error_per;
%per_ds.cor_per = ods_per.cor_per;
per_ds.period = ods_per.period;