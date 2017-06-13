%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%
function [resdata, ds] = compute_err(resdata)

ds = dataset();

models = fieldnames(resdata);

pp = 0;
for j=1:length(models)
    
    model = models{j};
    
    if strcmp(model,'zhu') || strcmp(model,'yun') || strcmp(model,'young')  || strcmp(model,'pep')||strcmp(model,'peps') || strcmp(model,'Bleser')
    
        versions = fieldnames(resdata.(model));

        for k=1:length(versions)
                        
            version = versions{k};
            
            cidxs = resdata.(model).(version).compidxs; 
            q.v = resdata.(model).(version).alig.ref;
            q.mR1 = resdata.(model).(version).alig.posR;
            q.mRexp = resdata.(model).(version).alig.posE;
            npidxs = resdata.(model).(version).npidxs;
%             [size(q.v) size(q.mR1) size(q.mRexp)]
            q.v = q.v(cidxs,:);
            q.mR1 = q.mR1(cidxs,:);
            q.mRexp = q.mRexp(cidxs,:);
%             [size(q.v) size(q.mR1) size(q.mRexp)]
            if cidxs(1)>1
                npidxs = npidxs-cidxs(1);
            end
            npidxs(npidxs<0)=1;
            
            qq = q.mR1;
            if resdata.(model).(version).alig.eExp < resdata.(model).(version).alig.eRot
                qq = q.mRexp;
            end
            
            %%% COMPUTE ERRORS
            
            % overall error and error over time with statistics
            
            
            e1 = zeros(size(q.mR1,1),1);
            for i=1:length(q.v)
            %     e(i,:) = norm(q.v(i,2:4) -q.mR(i,:));
                e1(i,:) =norm(q.v(i,2:4) -qq(i,:));
            end
            ave_e = mean(e1);
            med_e = median(e1);
            cor_x = corr2(q.v(:,2),qq(:,1));
            cor_y = corr2(q.v(:,3),qq(:,2));
            cor_z = corr2(q.v(:,4),qq(:,3));
            ave_cor = mean([cor_x,cor_y,cor_z]);

            resdata.(model).(version).err.error = e1;
            resdata.(model).(version).err.ave_e = ave_e;
            resdata.(model).(version).err.med_e = med_e;
            
            % overall overshoot
            resdata.(model).(version).err.overshoot = ((e1-ave_e).^2).^0.5;
            
            % cumulated overshoot
            resdata.(model).(version).err.std_e = std(e1);
            
            % maximum overshoot
            resdata.(model).(version).err.max_overshoot = max(((e1-ave_e).^2).^0.5);
            
            
            %error and overshoot over periods 
            nper = length(npidxs)-1;
             
            for i=1:nper
                
                pp = pp+1;
                
                cperx = corr2(q.v(npidxs(i):npidxs(i+1),2),qq(npidxs(i):npidxs(i+1),1));
                cpery = corr2(q.v(npidxs(i):npidxs(i+1),3),qq(npidxs(i):npidxs(i+1),2));
                cperz = corr2(q.v(npidxs(i):npidxs(i+1),4),qq(npidxs(i):npidxs(i+1),3));
                
                resdata.(model).(version).err.error_per(i) = mean(e1(npidxs(i):npidxs(i+1)));
                resdata.(model).(version).err.med_error_per(i) = median(e1(npidxs(i):npidxs(i+1)));
                resdata.(model).(version).cor_per(i) = mean([cperx cpery cperz]);
                resdata.(model).(version).err.std_per(i) = std(e1(npidxs(i):npidxs(i+1)));
                resdata.(model).(version).err.overshoot_per(i) = mean(((e1(npidxs(i):npidxs(i+1))-mean(e1(npidxs(i):npidxs(i+1)))).^2).^0.5);
                resdata.(model).(version).err.max_os_per(i) = max(((e1(npidxs(i):npidxs(i+1))-mean(e1(npidxs(i):npidxs(i+1)))).^2).^0.5);

                % fill the results dataset
                ds.model(pp,1) = {model};
                ds.version(pp,1) = {version};
                ds.error(pp,1) = {e1};
                ds.ave_e(pp,1) = ave_e;
                ds.med_e(pp,1) = med_e;
                ds.overshoot(pp,1) = {((e1-ave_e).^2).^0.5};
                ds.std_e(pp,1) = std(e1);
                ds.period(pp,1) = i-1;
                ds.per_idxs(pp,1) = {[npidxs(i) npidxs(i+1)]}; 
                ds.error_per(pp,1) = resdata.(model).(version).err.error_per(i);
                ds.med_per(pp,1) = resdata.(model).(version).err.med_error_per(i);
                ds.cor_per(pp,1) = resdata.(model).(version).cor_per(i);
                ds.std_per(pp,1) = resdata.(model).(version).err.std_per(i);
                ds.max_os_per(pp,1) = resdata.(model).(version).err.max_os_per(i);
                ds.corr(pp,1) = {[cor_x cor_y cor_z]};
                ds.ave_corr(pp,1) = ave_cor;
            
            end
            
        end
    else
        continue
    end
end

end