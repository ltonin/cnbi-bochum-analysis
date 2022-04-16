clearvars; clc;

subject = 'BOCH02';

includepat  = {subject, 'mi', 'mi_bhbf', 'online'};
excludepat  = {'control', 'offline'};
spatialfilter = 'laplacian';
artifactrej   = 'none'; % {'FORCe', 'none'}
fisherpath    = ['analysis/' artifactrej '/discriminancy/' spatialfilter '/'];
latencypath   = 'analysis/latency/';
figdir      = ['figures/' artifactrej '/latency/'];

modalities  = {'offline','online'};
feedbacks   = {'fake', 'positive', 'full', 'unknown'};
protocols   = {'bci-calibration', 'bci-training', 'wheelchair-training', 'wheelchair-control', 'unknown'};

fisher  = load([fisherpath subject '_fisher2_laplacian.mat']);
latency = load([latencypath subject '_latency.mat']);

%% Computing run-based latency
util_disp('[proc] - Computing run-based latency');

runs  = unique(latency.labels.trial.Rk);
nruns = length(runs);
RunLatencyAvg = nan(nruns, 1);
RunLatencyStd = nan(nruns, 1);
rMk = nan(nruns, 1); rDk = nan(nruns, 1); rFk = nan(nruns, 1); rPk = nan(nruns, 1);
for rId = 1:nruns
    % Trials in the run AND correct
    index = latency.labels.trial.Rk == runs(rId) & latency.labels.trial.Xk == true;

    if(sum(index) == 0)
        continue;
    end

    RunLatencyAvg(rId) = nanmean(latency.latency(index));
    RunLatencyStd(rId) = nanstd(latency.latency(index));

    rMk(rId) = unique(latency.labels.trial.Mk(index));
    rDk(rId) = unique(latency.labels.trial.Dk(index));
    rFk(rId) = unique(latency.labels.trial.Fk(index));
    rPk(rId) = unique(latency.labels.trial.Pk(index));
end
days  = unique(rDk(isnan(rDk) == false));
ndays = length(days);

%% Computing day-based latency
util_disp('[proc] - Computing day-based latency');
DayLatencyTrial = compute_day_latency_ontrials(latency.latency, latency.labels);
[corrLatencyTrial, pvalLatencyTrial] = corr((1:length(DayLatencyTrial.average))', DayLatencyTrial.average, 'rows', 'pairwise');

%% Computing day-based latency
util_disp('[proc] - Computing day-based latency');
DayLatencyRun = compute_day_latency_onruns(RunLatencyAvg, rDk, rPk);
[corrLatencyRun, pvalLatencyRun] = corr((1:length(DayLatencyRun.average))', DayLatencyRun.average, 'rows', 'pairwise');

%% Computing Selected features fisher evolution
nruns = size(fisher.fisher, 2);
runselfisher = nan(nruns, 1);
selfisher = nan(ndays, 1);
for rId = 1:nruns
    cfeatures = proc_cnbifeature2bin(fisher.classifiers{rId}.features, fisher.freqgrid{rId});
    runselfisher(rId) = mean(fisher.fisher(cfeatures, rId), 1);
end

for dId = 1:ndays
    index = fisher.labels.Dk == days(dId);
    selfisher(dId) = mean(runselfisher(index));
end

[selcorr, selpval] = corr((1:ndays)', selfisher);

%% Computing latency-fisher correlation
[corrLatencyFisher, pvalLatencyFisher] = corr(runselfisher, RunLatencyAvg, 'rows', 'pairwise');

%% Figures
fig = figure;
fig_set_position(fig, 'All');

subplot(2, 2, 1);
[h1, h2, h3] = plot_day_latency(DayLatencyTrial.average, fisher.labels.NCs, corrLatencyTrial, pvalLatencyTrial, [1 9], unique(fisher.labels.Dl));
title([subject ' | Day Latency (trial-based)']);

subplot(2, 2, 2);
[h4, h5, h6] = plot_day_latency(DayLatencyRun.average, fisher.labels.NCs, corrLatencyRun, pvalLatencyRun, [1 9], unique(fisher.labels.Dl));
title([subject ' | Day Latency (run-based)']);

subplot(2, 2, 3)
[h7, h8] = plot_latency_fisher(RunLatencyAvg, runselfisher, corrLatencyFisher, pvalLatencyFisher, [0 max(RunLatencyAvg)+1], [0 0.5]);
title([subject ' | Latency vs. Fisher']);

%% Exporting figure
filename = fullfile(figdir, [subject '_latency_fisher.pdf']);
util_disp(['[out] - Exporting latency vs. fisher ' filename], 'b');
fig_export(fig, filename, '-pdf');

%% Functions

function dlatency = compute_day_latency_ontrials(latency, labels)
    
    days  = unique(labels.trial.Dk);
    ndays = length(days);
    average = nan(ndays, 1);
    stdev = nan(ndays, 1);
    for dId = 1:ndays
        % Trials in the day AND correct AND ( online | guided )
        index = labels.trial.Dk == days(dId) & labels.trial.Xk == true & ...
                (labels.trial.Pk == 2 | labels.trial.Pk == 3); % 'bci-training' | 'wheelchair-training' 
    
        average(dId) = nanmean(latency(index));
        stdev(dId) = nanstd(latency(index));
    end

    dlatency.average = average;
    dlatency.stdev   = stdev;
end

function dlatency = compute_day_latency_onruns(rlatency, rDk, rPk)
    days  = unique(rDk(isnan(rDk)==false));
    ndays = length(days);
    average = nan(ndays, 1);
    stdev = nan(ndays, 1);
    for dId = 1:ndays
        % Trials in the day AND correct AND ( online | guided )
        index = rDk == days(dId) & (rPk == 2 | rPk == 3); % 'bci-training' | 'wheelchair-training' 
    
        average(dId) = nanmean(rlatency(index));
        stdev(dId) = nanstd(rlatency(index));
    end

    dlatency.average = average;
    dlatency.stdev   = stdev;
end


function [h1, h2, h3] = plot_day_latency(latency, newclassifier, rval, pval, ylimits, xlabels)
    ndays = length(latency);
    hold on;
    h1 = scatter(1:ndays, latency, 'k', 'filled');
    h2 = plot(1:ndays, latency, 'k');
    hold off;
    h3 = lsline;
    ylim([ylimits(1) ylimits(2)])
    text(min(h1.XData), ylimits(2) - 1, ['r=' num2str(rval, '%4.2f') ', p=' num2str(pval, '%4.3f')], 'color', get(h1,'CData'));
    plot_vline(find(newclassifier)-0.1, 'k--');
    grid on;
    set(gca, 'xtick', 1:ndays);
    set(gca,'xticklabels', xlabels);
    ylabel('time [s]');
    xlabel('session');

end

function [h1, h2] = plot_latency_fisher(latency, fisher, rval, pval, xlimits, ylimits)
    h1 = scatter(latency, fisher, 'k', 'filled');
    xlim([xlimits(1) xlimits(2)]);
    ylim([ylimits(1) ylimits(2)]);
    h2 = lsline;
    text(max(h1.XData) - 2, max(h1.YData), ['r=' num2str(rval, '%4.2f') ', p=' num2str(pval, '%4.3f')], 'color', get(h1,'CData'));
    grid on;
    xlabel('time [s]');
    ylabel('fisher score')
end