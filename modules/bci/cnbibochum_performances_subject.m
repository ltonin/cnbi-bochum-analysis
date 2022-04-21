clearvars; clc;

subject = 'BOCH04';

includepat  = {subject, 'mi', 'mi_bhbf', 'online'};
excludepat  = {'control'};
% includepat  = {subject, 'mi', 'mi_bhbf'};
% excludepat  = {'ciao'};
spatialfilter = 'laplacian';
artifactrej   = 'none'; % {'FORCe', 'none'}
datapath    = ['analysis/' artifactrej '/psd/' spatialfilter '/'];
savedir     = ['analysis/' artifactrej '/performances/' spatialfilter '/'];
figdir      = 'figures/bci/performances/';

ClassEventId     = [773 771];
ClassNames       = {'both hands', 'both feet'};
CFeedbackEventId = 781;
CorrectEventId   = 897;
WrongEventId     = 898;
files = util_getfile3(datapath, '.mat', 'include', includepat, 'exclude', excludepat);
nfiles = length(files);
util_bdisp(['[io] - Found ' num2str(nfiles) ' files with the inclusion/exclusion criteria: (' strjoin(includepat, ', ') ') / (' strjoin(excludepat, ', ') ')']);

% Create analysis directory
util_mkdir('./', savedir);
util_mkdir('./', figdir);

%% Concatenate data
util_bdisp(['[io] - Importing ' num2str(nfiles) ' files from ' datapath ':']);
[F, events, labels, classifiers, settings] = cnbibochum_concatenate_labels(files);

NumSamples = size(F, 1);

%% Get general events
[~, TrialEvent] = proc_get_event2(CFeedbackEventId, NumSamples, events.POS, events.TYP, events.DUR);
[ClassLabels, ClassEvent] = proc_get_event2(ClassEventId, NumSamples, events.POS, events.TYP, events.DUR);

NumTrials  = length(TrialEvent.TYP);
Classes    = setdiff(unique(ClassLabels), 0);
NumClasses = length(Classes);

Ck = zeros(NumTrials, 1);
Mk = zeros(NumTrials, 1);
Rk = zeros(NumTrials, 1);
Dk = zeros(NumTrials, 1);
Nk = zeros(NumTrials, 1);
Pk = zeros(NumTrials, 1);
Fk = zeros(NumTrials, 1);

for trId = 1:NumTrials
    cstart = TrialEvent.POS(trId);
    cstop  = cstart + TrialEvent.DUR(trId) - 1;
   
    Ck(trId) = ClassEvent.TYP(trId);
    Mk(trId) = unique(labels.samples.Mk(cstart:cstop));
    Rk(trId) = unique(labels.samples.Rk(cstart:cstop));
    Dk(trId) = unique(labels.samples.Dk(cstart:cstop));
    Nk(trId) = unique(labels.samples.Nk(cstart:cstop));
    Pk(trId) = unique(labels.samples.Pk(cstart:cstop));
    Fk(trId) = unique(labels.samples.Fk(cstart:cstop));
end

Xk = events.TYP(events.TYP == CorrectEventId | events.TYP == WrongEventId);
Days = unique(labels.samples.Dk);
NumDays = length(Days);
Runs = unique(labels.samples.Rk);
NumRuns = length(Runs);
Dl = unique(labels.run.Dl);

%% Get labels per run
rCsK = zeros(NumRuns, 1);
rDk = zeros(NumRuns, 1);
prevclassifier = [];
currclassifier = [];
for rId = 1:NumRuns
    index = labels.samples.Rk == Runs(rId);

    rDk(rId) = unique(labels.samples.Dk(index));

    % Classifier
    currclassifier = classifiers(rId).filename;
    if(strcmp(prevclassifier, currclassifier) == 0)
        rCsK(rId) = 1;
        prevclassifier = currclassifier;
    end
    
end

%% Computing new classifier per day
NCs = false(NumDays, 1);
for dId = 1:NumDays
    index = rDk == Days(dId);
    if(sum(rCsK(index)) > 0)
        NCs(dId) = true;
    end
end


%% Compute performances per run (overall)
RunPerf      = nan(NumRuns, 1);

for rId = 1:NumRuns
   
    cindex = Rk == Runs(rId);
    cntrials = sum(cindex);
    
    RunPerf(rId) = 100*sum(Xk(cindex) == CorrectEventId)./cntrials;
end



%% Compute performances per day (overall and per class)
DayPerf      = nan(NumDays, 1);

for dId = 1:NumDays
   
    cindex = Dk == Days(dId);
    cntrials = sum(cindex);
    
    DayPerf(dId) = 100*sum(Xk(cindex) == CorrectEventId)./cntrials;
end

[DayCorr, DayPVal] = corr((1:NumDays)', DayPerf, 'rows', 'pairwise');

if(strcmp(subject, 'BOCH05'))
    DayPerf2 = nan(NumDays-3, 1);

    for dId = 1:NumDays-3
       
        cindex = Dk == Days(dId);
        cntrials = sum(cindex);
        
        DayPerf2(dId) = 100*sum(Xk(cindex) == CorrectEventId)./cntrials;
    end
    [DayCorr2, DayPVal2] = corr((1:NumDays-3)', DayPerf2, 'rows', 'pairwise');
end

%% Computer performance first last runs and ttest
NumSelRuns = 10;

AvgPerf(1) = mean(RunPerf(1:NumSelRuns));
AvgPerf(2) = mean(RunPerf(end-NumSelRuns+1:end));
StdPerf(1) = std(RunPerf(1:NumSelRuns));
StdPerf(2) = std(RunPerf(end-NumSelRuns+1:end));

[htest, ptest] = ttest(RunPerf(1:NumSelRuns),RunPerf(end-NumSelRuns+1:end));

if(strcmp(subject, 'BOCH05'))
    RunPerf2 = RunPerf(rDk <= NumDays -3);
    AvgPerf2(1) = mean(RunPerf2(1:NumSelRuns));
    AvgPerf2(2) = mean(RunPerf2(end-NumSelRuns+1:end));
    StdPerf2(1) = std(RunPerf2(1:NumSelRuns));
    StdPerf2(2) = std(RunPerf2(end-NumSelRuns+1:end));

    [htest2, ptest2] = ttest(RunPerf2(1:NumSelRuns),RunPerf2(end-NumSelRuns+1:end));
end

%% Plotting performances per run and per day
fig1 = figure;
fig_set_position(fig1, 'Top');

NumRows = 1;
NumCols = 2;

% Performances per run
subplot(NumRows, NumCols, 1);

hold on;
h1 = scatter(1:NumDays, DayPerf, 'k', 'filled');
h2 = plot(1:NumDays, DayPerf, 'k');
lsline;
hold off;
set(gca, 'xtick', 1:NumDays);
set(gca,'xticklabels', unique(Dl));
grid on;
ylim([0 100]);
plot_vline(find(NCs)-0.1, 'k-');
xlabel('Day');
ylabel('Percentage [%]');
title(['Performance over days (r=' num2str(DayCorr, '%4.3f') ', p<' num2str(DayPVal, '%10.9f') ')']);

subplot(NumRows, NumCols, 2);

hold on;
bar(AvgPerf);
errorbar(AvgPerf, StdPerf, '.k');
hold off;
ylim([0 110]);
set(gca, 'Xtick', 1:2);
set(gca, 'xticklabel', {['First ' num2str(NumSelRuns) ' runs'],['Last ' num2str(NumSelRuns) ' runs']});
grid on;
title(['pvalue = ' num2str(ptest, '%10.9f')]);

if(strcmp(subject, 'BOCH05'))
    fig2 = figure;
    hold on;
    bar(AvgPerf2);
    errorbar(AvgPerf2, StdPerf2, '.k');
    hold off;
    ylim([0 110]);
    set(gca, 'Xtick', 1:2);
    set(gca, 'xticklabel', {['First ' num2str(NumSelRuns) ' runs'],['Last ' num2str(NumSelRuns) ' runs']});
    grid on;
    title(['pvalue = ' num2str(ptest2, '%10.9f')]);
end

%% Saving figures
% filename1 = fullfile(figdir, [subject '_bci_performances.pdf']);
% util_bdisp(['[out] - Exporting bci performances to ' filename1]);
% fig_export(fig1, filename1, '-pdf');
