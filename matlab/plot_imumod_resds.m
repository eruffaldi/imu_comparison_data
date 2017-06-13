%
% Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
% Alessandro Filippeschi SSSA 2012-2016
%

% function [] = plot_imumod_resds(ds)
clc
if exist('camready')==0
    camready =0;
end

if camready ==1
    cidx = resdata2.pep.Pep_or.compidxs(1);
else
    cidx =1;
end

ds = ods;
ds_red = ds(ds.period==1,:);

ds_mod = ds;

%expand ds
mfldn = fieldnames(ds_mod);
ds_ae = dataset();
pp =1;
for i=1:length(ds_mod)
    
    lt = size(ds_mod.error,1);
    l = ds_mod.per_idxs{i};
    
    if i==1
        l(1) = cidx;
    end
        
    ll = diff(l);
    
    np = unique(ds_mod.period(i));
    
    if i==1
        eidx = pp+ll;
        rep = ll+1;
    else
        eidx = pp+ll-1;
        rep = ll;
    end
    
    for j=1:length(mfldn)
%         ll
%         eidx
%         [l(1) l(2)]
        if strcmp(mfldn{j},'error')==0 && strcmp(mfldn{j},'overshoot')==0 && strcmp(mfldn{j},'Properties')==0 && strcmp(mfldn{j},'period')==0
            
            if iscell(ds_mod.(mfldn{j})(i))
                ds_ae.(mfldn{j})(pp:eidx,1) = repmat(ds_mod.(mfldn{j})(i),rep,1);
            else
                ds_ae.(mfldn{j})(pp:eidx,1) = repmat(ds_mod.(mfldn{j})(i),rep,1);
            end
            
        elseif strcmp(mfldn{j},'error')==1 || strcmp(mfldn{j},'overshoot')==1
            
            if i>1
                ds_ae.(mfldn{j})(pp:eidx,1) = ds_mod.(mfldn{j}){i}(l(1)+1:l(2));
            else
                ds_ae.(mfldn{j})(pp:eidx,1) = ds_mod.(mfldn{j}){i}(cidx:l(2));
            end
        end
    end
    
    ds_ae.period(pp:eidx,1) = repmat(ds_mod.period(i),rep,1);
    
    pp=eidx+1;

end


%% plot 

if camready == 0

    close all
    clc
    h = figure('Position',[20 20 1700 800]);
    subplot(2,1,1)
    boxplot(ds_ae.error, ds_ae.version)

    subplot(2,1,2)
    boxplot(ds_ae.overshoot, ds_ae.version)

    mm = unique(ds_ae.model);
    vv = unique(ds_ae.version);

    me = max(ds_ae.error);
    mo = max(ds_ae.overshoot);

    h1 = figure('Position',[20 20 1700 800],'Name','error');
    h2 = figure('Position',[20 20 1700 800],'Name','overshot');
    for i=1:length(vv)

        thds = ds_ae(strcmp(ds_ae.version,vv{i}),:);
        set(0,'CurrentFigure',h1)
        subplot(2,4,i)
        boxplot(thds.error,thds.period)
        ylim([0 me])
        title(vv{i})

        set(0,'CurrentFigure',h2)
        subplot(2,4,i)
        boxplot(thds.overshoot,thds.period)
        title(vv{i})
        ylim([0 mo])
    end



    h3 = figure('Position',[20 20 1700 800],'Name','error_time');
    h4 = figure('Position',[20 20 1700 800],'Name','overshot_time');
    for i=1:length(vv)

        thds = ds_ae(strcmp(ds_ae.version,vv{i}),:);
        set(0,'CurrentFigure',h3)
        subplot(2,4,i)
        plot(thds.error)
        hold on
        plot(ones(size(thds.error))*thds.ave_e(1),'--r')
        plot(ones(size(thds.error))*thds.med_e(1),'--g')
        legend({'error (distance) [mm]','average of e', 'median of e'})
        ylim([0 me])
        title(vv{i})

        set(0,'CurrentFigure',h4)
        subplot(2,4,i)
        plot(thds.overshoot)
            hold on
        plot(ones(size(thds.overshoot))*mean(thds.overshoot),'--r')
        plot(ones(size(thds.overshoot))*median(thds.overshoot),'--g')
        legend({'overshoot(distance) [mm]','average of o', 'median of o'})
        title(vv{i})
        ylim([0 mo])
    end

else
    fs = 16;
    lw = 1.5;
    methods = {'Method 1','Method 2','Method 3-pure', 'Method 3-perfect', 'Method 4', 'Method 5'};
    close all
    clc
%     h = figure('Position',[20 20 1700 800]);
%     subplot(2,1,1)
%     boxplot(ds_ae.error, ds_ae.version)
% 
%     subplot(2,1,2)
%     boxplot(ds_ae.overshoot, ds_ae.version)

    mm = unique(ds_ae.model);
    vv = unique(ds_ae.version);

    me = max(ds_ae.error);
    mo = max(ds_ae.overshoot);

    h1 = figure('Position',[20 20 800 1000],'Name','error','color','none');
% %     h2 = figure('Position',[20 20 1700 800],'Name','overshot');
    for i=1:length(vv)
        
        switch vv{i}
            case 'Zhu_or'
                k=1;
            case 'Yun_or'
                k=2;     
            case 'You_pu_c'
                k=3;
            case 'You_pe_c'
                k=4;
            case 'or'
                k=5;
            case 'Pep_or'
                k=6;
        end

        thds = ds_ae(strcmp(ds_ae.version,vv{i}),:);
        thds = thds(thds.period>0,:);
        set(0,'CurrentFigure',h1)
        subplot(3,2,k)
        boxplot(thds.error,thds.period);
        hb1 = findobj(gca,'Type','text');
        hb2 = findobj(gca,'Type','line');
        set(hb1,'FontSize',fs)
        set(hb2,'LineWidth',lw)
        set(gca,'FontSize',fs)
        ylim([0 me])
        title(methods{k},'FontSize',fs)
        set(gca,'FontSize',fs)

%         set(0,'CurrentFigure',h2)
%         subplot(2,4,i)
%         boxplot(thds.overshoot,thds.period)
%         title(vv{i})
%         ylim([0 mo])
    end

export_fig('D:\Ale_DFKI_TRO\sim\tro_paper_loc\figures\f9_periods.pdf')

    h3 = figure('Position',[10 10 800 1000],'Name','error_time','color','none');
%     h4 = figure('Position',[20 20 1700 800],'Name','overshoot_time');
    for i=1:length(vv)
        
        switch vv{i}
            case 'Zhu_or'
                k=1;
            case 'Yun_or'
                k=2;     
            case 'You_pu_c'
                k=3;
            case 'You_pe_c'
                k=4;
            case 'or'
                k=5;
            case 'Pep_or'
                k=6;
        end
        thds = ds_ae(strcmp(ds_ae.version,vv{i}),:);
        set(0,'CurrentFigure',h3)
        subplot(3,2,k)
        plot((1:size(thds.error,1))*0.01,thds.error,'k')
        hold on
        plot((1:size(thds.error,1))*0.01, ones(size(thds.error))*thds.ave_e(1),'--k')
%         plot(ones(size(thds.error))*thds.med_e(1),'--k')
%         legend({'error (distance) [mm]','average of e'})
        ylim([0 me]), xlim([0 (size(thds.error,1)-150)*0.01])
        if k==1 || k==3 || k==5
            ylabel('E [mm]','FontSize',fs)
        end
%         xlabel('time [s]')
        title(methods{k},'FontSize',fs)
%         title(vv{i},'FontSize',fs)
        set(gca,'FontSize',fs)

%         set(0,'CurrentFigure',h4)
%         subplot(2,4,i)
%         plot(thds.overshoot)
%             hold on
%         plot(ones(size(thds.overshoot))*mean(thds.overshoot),'--r')
%         plot(ones(size(thds.overshoot))*median(thds.overshoot),'--g')
%         legend({'overshoot(distance) [mm]','average of o', 'median of o'})
%         title(vv{i})
%         ylim([0 mo])
    end
export_fig('D:\Ale_DFKI_TRO\sim\tro_paper_loc\figures\f8_error.pdf')
    
end




% end