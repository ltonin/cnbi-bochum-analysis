clearvars; clc; close all;

sublist = {'BOCH02', 'BOCH04', 'BOCH05'};
nsubjects = length(sublist);

spatialfilter = 'laplacian';
artifactrej   = 'none'; % {'FORCe', 'none'}

datapath  = ['analysis/' artifactrej '/performances/' spatialfilter '/'];
figdir      = 'figures/bci/performances/';

FirstDays = [10 5 5];

%% Evolution of performances over days
NumRows = 1;
NumCols = nsubjects;

fig1 = figure;
fig_set_position(fig1, 'Top');
for sId = 1:nsubjects
    csubject = sublist{sId};
    cfile   = [csubject '_performances_' spatialfilter '.mat'];
    util_bdisp(['[io] - Importing performances data for subject ' csubject ' from ' datapath]);
    cdata = load([datapath cfile]);
    
    cperf = cdata.performances;
    
    cdayperf = cperf.total.day;
    cdayId = 1:length(cdayperf);
    firstdays = FirstDays(sId);
    [cdaycorrtot, cdaypvaltot] = corr((cdayId)', cdayperf, 'rows', 'pairwise');
    [cdaycorrfirst, cdaypvalfirst] = corr((cdayId(1:firstdays))', cdayperf(1:firstdays), 'rows', 'pairwise');
    
    % Find runId classifier switch
    clflist = {};
    Cl = [];
    pclfId = 0;
    for rId = 1:length(cperf.classifiers)
        cclf = cperf.classifiers(rId).filename;
        ccId = find(contains(clflist, cclf));
        
        if (isempty(ccId) == true)
            clflist = cat(1, clflist, cclf);
            cclfId = pclfId + 1;
            pclfId = cclfId;
        else
            cclfId = pclfId;
        end
        Cl = cat(1, Cl, cclfId);
    end
    
    RunSwitchId = find(diff(Cl) == 1);
    DaySwitchId = nan(length(RunSwitchId), 1);
    for rsId = 1:length(RunSwitchId)
        DaySwitchId(rsId) = cperf.labels.samples.Dk(find(cperf.labels.samples.Rk == RunSwitchId(rsId), 1));
    end
    
    DaySwitchId = unique(DaySwitchId);
    
    
    subplot(NumRows, NumCols, sId);
    
    hold on;
    plot(cdayId, cdayperf, '-ko', 'MarkerFaceColor', 'k');
    
    FitTot = polyfit(cdayId,cdayperf,1); 
    plot(polyval(FitTot,cdayId), 'k', 'LineWidth', 2);
    
    FitFirst = polyfit(cdayId(1:firstdays),cdayperf(1:firstdays),1); 
    plot(polyval(FitFirst,cdayId(1:firstdays)), 'r', 'LineWidth', 2);
    hold off;

    set(gca, 'XTick', cdayId);
    grid on;
    ylim([0 100]);
    plot_vline(DaySwitchId + 0.5, '--k');
    plot_vline(firstdays, '--r', ['First ' num2str(firstdays) ' sessions']);
    

    
    text(0.5, 95, ['rho=' num2str(cdaycorrtot, '%5.3f') ', ' pval2text(cdaypvaltot)], 'FontWeight', 'bold', 'color', 'k');
    text(0.5, 90, ['rho=' num2str(cdaycorrfirst, '%5.3f') ', ' pval2text(cdaypvalfirst)], 'FontWeight', 'bold', 'color', 'r');

    xlabel('Session');
    ylabel('Performance [%]');
    title([csubject ' - Evolution of the performances']);
end

%% Comparison of the performances
NumRows = 1;
NumCols = nsubjects;

nperiods = 4;
nrunsperperiod = 5;

PerfMean = nan(nperiods, nsubjects);
PerfStd = nan(nperiods, nsubjects);

for sId = 1:nsubjects
    csubject = sublist{sId};
    cfile   = [csubject '_performances_' spatialfilter '.mat'];
    util_bdisp(['[io] - Importing performances data for subject ' csubject ' from ' datapath]);
    cdata = load([datapath cfile]);
    
    cperf = cdata.performances;
    
    crunperf = cperf.total.run;
    crunId = 1:length(crunperf);
    cnruns = length(crunperf);
    
    FirstSessions = FirstDays(sId);
    LastRunFirstPeriod = cperf.labels.samples.Rk(find(cperf.labels.samples.Dk == FirstSessions, 1, 'last'));
    perfperiods(:, 1) = [1 nrunsperperiod];
    perfperiods(:, 2) = [LastRunFirstPeriod-nrunsperperiod+1  LastRunFirstPeriod];
    perfperiods(:, 3) = [LastRunFirstPeriod+1 LastRunFirstPeriod+nrunsperperiod];
    perfperiods(:, 4) = [cnruns-nrunsperperiod+1 cnruns];
    
    
    for pId = 1:nperiods
        PerfMean(pId, sId) = nanmean(crunperf(perfperiods(1, pId):perfperiods(2, pId)));
        PerfStd(pId, sId) = nanstd(crunperf(perfperiods(1, pId):perfperiods(2, pId)));
    end
     
end

fig2 = figure;
hold on;
for sId = 1:nsubjects
    herr = errorbar([-0.273 -0.091 0.091 0.273]+ sId, PerfMean(:, sId), PerfStd(:, sId), 'k.');
end
hbar = bar(PerfMean', 'FaceColor', [0.5 0.5 0.5]);
set(hbar(2:3), 'EdgeColor', 'r');
hold off;
grid on;
set(gca, 'XTick', 1:nsubjects);
set(gca, 'XTickLabel', {'P1', 'P2', 'P3'});
ylim([0 110]);
ylabel('Performances [%]');
title('Performances comparison between first/last sessions');


%% Saving figures
filename1 = fullfile(figdir, 'all_bci_performance_evolution.pdf');
filename2 = fullfile(figdir, 'all_bci_performance_comparison.pdf');
util_bdisp(['[out] - Exporting bci performances to ' filename1]);
fig_export(fig1, filename1, '-pdf');
fig_export(fig2, filename2, '-pdf');

%% Functions

function pvaltxt = pval2text(pval)
    if(pval) > 0.05
        pvaltxt = ['p = ' num2str(pval, '%5.3f')];
    elseif pval <= 0.05 && pval > 0.01
        pvaltxt = 'p < 0.05';
    elseif pval <= 0.01 && pval > 0.001
        pvaltxt = 'p < 0.01';
    elseif pval <= 0.001 && pval > 0.0001
        pvaltxt = 'p < 0.001';
    elseif pval <= 0.0001
        pvaltxt = 'p < 0.0001';
    end
end
