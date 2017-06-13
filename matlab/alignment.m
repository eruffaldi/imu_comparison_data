%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
% clear, clc, close all

function resdata = alignment(resdata,run)

models = fieldnames(resdata);

for j=1:length(models)
    
    model = models{j};
    
    if strcmp(model,'zhu') || strcmp(model,'yun') || strcmp(model,'young')  ||strcmp(model,'peps')|| strcmp(model,'pep') || strcmp(model,'Bleser')
    
        versions = fieldnames(resdata.(model));

        for k=1:length(versions)

            version = versions{k};

            q.m = resdata.(model).(version).pos.Lwrist';

            q.v = resdata.ref.vic.L_lower_arm;
            q.v2 = resdata.ref.vic.L_upper_arm;
            
            %%%% ALIGN DATA
            
            % manual N-pose
%             q.m = q.m - repmat(mean(q.m(1:70,:)),length(q.m),1);%(1:70,:)
            q.v = q.v - repmat(mean(q.v(1:70,:)),length(q.v),1);
            
            [nsel, npidxs,compidxs]= selidxs(run);

            q.m = q.m - repmat(mean(q.m(nsel,:)),length(q.m),1);%(1:70,:)
            resdata.(model).(version).npidxs = npidxs;
            resdata.(model).(version).compidxs = compidxs;
            % for i=1:length(q.m)
            %     q.mR(i,:) = [Rz*Rx*q.m(i,:)']';
            %     q.mR1(i,:) = [Rz1*Rx*q.m(i,:)']';
            % end


            % q.mR = q.mR - repmat(mean(q.mR(1:70,:)),length(q.mR),1);
            % q.mR1 = q.mR1 - repmat(mean(q.mR1(1:70,:)),length(q.mR1),1);
            
            R0 = [1 0 0; 0 -1 0; 0 0 -1];
            w0 = [1 1 0]*pi/2;
            
            if  strcmp(model,'pep')
                R0 = [1 0 0; 0 0 -1; 0 1 0];
                w0 = [-1 0 1]*1*pi;
            end
            w0    
%             R1 = [0 -1 0; -1 0 0; 0 0 -1];
%             R0=R1;
            tic;
            [Rr,remove_a,remove_b] = opt_or(q.m(compidxs,:)',q.v(compidxs,2:4)',R0);
%             trot =   toc
            tic;
            [re,Re] = opt_or_exp(q.m(compidxs,:)',q.v(compidxs,2:4)',w0);
%             texp = toc


            for i=1:length(q.m)
            %     q.mR(i,:) = [Rz*Rx*q.m(i,:)']';
                q.mR1(i,:) = [Rr*q.m(i,:)']';
                q.mRexp(i,:) = [Re*q.m(i,:)']';
            end
            
            resdata.(model).(version).alig.RotMat = Rr;
            resdata.(model).(version).alig.RotMatExp = Re;
            resdata.(model).(version).alig.pos_nob = q.m;
            resdata.(model).(version).alig.posR = q.mR1;
            resdata.(model).(version).alig.posE = q.mRexp;
            resdata.(model).(version).alig.ref = q.v;
            resdata.(model).(version).alig.eExp = norm(pw_norm(q.v(:,2:4)-q.mRexp));
            resdata.(model).(version).alig.eRot = norm(pw_norm(q.v(:,2:4)-q.mR1));
            
        end
    else
        continue
    end
end

end

function [nsel, npidxs, compidxs]= selidxs(run)

switch run
    case 'L_EF'
        npidxs = sort([1 1034 529 1271 1147 906 779 661 394 272 1601]);
        nsel = 271:273;
        compidxs = 50:1601;
    case 'L_SA'
        npidxs = sort([1 70 242 408 560 723 886 1053 1222 1422]);
        nsel = 721:725;
        compidxs = 50:1550;
    case 'L_SF'
        npidxs = sort([1 223 407 579 756 909 1085 1245 1403 1585 1776 1968 2144 2328]);
        nsel = 40:60;
        compidxs = 60:2631;
end
end
