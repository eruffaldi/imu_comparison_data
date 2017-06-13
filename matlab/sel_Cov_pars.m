%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function Kcov = sel_Cov_pars(dataflds,Allids,id)

for i=1:length(dataflds)
    
    thfld = dataflds{i};
    
    for j=1:length(Allids)
        
        if strfind(id,'Zhu_or')
            Kcov.(thfld).meas.g = 25;
            Kcov.(thfld).meas.m = 1;
            Kcov.(thfld).meas.w = 0;
            Kcov.(thfld).proc = 0.05*diag([0.002 0.002 0.002])*eye(3);%0.01
            
        elseif strfind(id,'Zhu_fi')
            Kcov.(thfld).meas.g = 25;
            Kcov.(thfld).meas.m = 1;
            Kcov.(thfld).meas.w = 1;%20
            Kcov.(thfld).proc = 0.01*diag([0.002 0.002 0.002])*eye(3);%0.01
            
        elseif strfind(id,'Zhu_se')
            Kcov.(thfld).meas.g = 25;%25
            Kcov.(thfld).meas.m = 1;%0.1
            Kcov.(thfld).meas.w = 1;%1
            Kcov.(thfld).proc = 0.01*diag([0.002 0.002 0.002])*eye(3);%0.01
            
%         elseif strfind(id,'Zhu_se')
%             Kcov.(thfld).meas.g = 0.01;%25
%             Kcov.(thfld).meas.m = 0.01;%0.1
%             Kcov.(thfld).meas.w = 1;%1
%             Kcov.(thfld).proc = 0.1*diag([0.002 0.002 0.002])*eye(3);%0.01
            
        elseif strfind(id,'Yun_or')
            Kcov.(thfld).meas.g = 10;
            Kcov.(thfld).meas.m = 0.1;
            Kcov.(thfld).meas.w = 1;
            Kcov.(thfld).proc = 0.01*diag([0.002 0.002 0.002])*eye(3);
            
        elseif strfind(id,'You_pu_c')
            Kcov.(thfld).meas.g = 10;
            Kcov.(thfld).meas.m = 1;
            Kcov.(thfld).meas.w = 0;
            Kcov.(thfld).proc = 0.001*diag([0.002 0.002 0.002])*eye(3);
            
        elseif strfind(id,'You_pu_k')
            Kcov.(thfld).meas.g = 10;
            Kcov.(thfld).meas.m = 1;
            Kcov.(thfld).meas.w = 0;
            Kcov.(thfld).proc = 0.001*diag([0.002 0.002 0.002])*eye(3);
            
        elseif strfind(id,'You_pe_c')
            Kcov.(thfld).meas.g = 10;
            Kcov.(thfld).meas.m = 1;
            Kcov.(thfld).meas.w = 0;
            Kcov.(thfld).proc = 0.001*diag([0.002 0.002 0.002])*eye(3);
            
        elseif strfind(id,'You_pe_k')
            Kcov.(thfld).meas.g = 10;
            Kcov.(thfld).meas.m = 1;
            Kcov.(thfld).meas.w = 0;
            Kcov.(thfld).proc = 0.001*diag([0.002 0.002 0.002])*eye(3);
            
        elseif strfind(id,'Pep_or')
            
            Kcov.(thfld).meas.g = 1;
            Kcov.(thfld).meas.m = 1;
            Kcov.(thfld).meas.w = 1;
            Kcov.(thfld).proc = 1*diag([0.002 0.002 0.002])*eye(3);
            Kcov.covEps = 1;
            Kcov.covQ = 1;
            
        elseif strfind(id,'Peps_or')
            
            Kcov.(thfld).meas.g = 1;
            Kcov.(thfld).meas.m = 1;
            Kcov.(thfld).meas.w = 1;
            Kcov.(thfld).proc = 1*diag([0.002 0.002 0.002])*eye(3);
            Kcov.covEps = 1;
            Kcov.covQ = 1;
                        
        end
    end
end
        


    