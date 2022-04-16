clearvars; clc;

subject = 'BOCH01';

includepat  = {subject, 'mi', 'mi_bhbf', 'online'};
excludepat  = {'guided', 'control'};
% includepat  = {subject, 'mi', 'mi_bhbf'};
% excludepat  = {'ciao'};
spatialfilter = 'laplacian';
artifactrej   = 'none'; % {'FORCe', 'none'}
datapath    = ['analysis/' artifactrej '/psd/' spatialfilter '/'];
savedir     = ['analysis/' artifactrej '/accuracy/' spatialfilter '/'];
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

U = log(F);
P = proc_reshape_ts_bc(U);
NumSamples = size(U, 1);
NumChans   = size(U, 3);
NumFreqs   = size(U, 2);
SrcFreqs   = settings.spectrogram.freqgrid;

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
HitDur = events.DUR(events.TYP == CFeedbackEventId);


%% Generic Labels
Runs    = unique(labels.samples.Rk);
Days    = unique(labels.samples.Dk);
Weeks   = unique(labels.samples.Wk);
Months  = unique(labels.samples.Nk);

NumRuns   = length(Runs);
NumDays   = length(Days);
NumWeeks  = length(Weeks);
NumMonths = length(Months);

%% Select runs to be removed
[rRmk, RemRuns] = cnbibochum_exclude_runs(subject, NumRuns);

Rmk = false(NumTrials, 1);
for trId = 1:NumTrials
   crunId = Rk(trId);
   
   Rmk(trId) = rRmk(crunId);
end


%% Compute duration per run (overall and per class)
RunHitDurAllAvg   = nan(NumRuns, 1);
RunHitDurAllStd   = nan(NumRuns, 1);
RunHitDurClassAvg = nan(NumRuns, NumClasses);
RunHitDurClassStd = nan(NumRuns, NumClasses);

for rId = 1:NumRuns
    cindex = Rk == Runs(rId) & Rmk == false & Xk == CorrectEventId;
    
    
    RunHitDurAllAvg(rId) = mean(HitDur(cindex)*settings.spectrogram.wshift);
    RunHitDurAllStd(rId) = std(HitDur(cindex)*settings.spectrogram.wshift);
    
    for cId = 1:NumClasses
        cindex_class = cindex & Ck == ClassEventId(cId); 
        
        RunHitDurClassAvg(rId, cId) = mean(HitDur(cindex_class)*settings.spectrogram.wshift);
        RunHitDurClassStd(rId, cId) = std(HitDur(cindex_class)*settings.spectrogram.wshift);
    end
end

%% Compute performances per day (overall and per class)
DayHitDurAllAvg   = nan(NumDays, 1);
DayHitDurAllStd   = nan(NumDays, 1);
DayHitDurClassAvg = nan(NumDays, NumClasses);
DayHitDurClassStd = nan(NumDays, NumClasses);

for dId = 1:NumDays
    cindex = Dk == Days(dId) & Rmk == false & Xk == CorrectEventId;
    
    DayHitDurAllAvg(dId) = mean(HitDur(cindex)*settings.spectrogram.wshift);
    DayHitDurAllStd(dId) = std(HitDur(cindex)*settings.spectrogram.wshift);
    
    for cId = 1:NumClasses
        cindex_class = cindex & Ck == ClassEventId(cId); 
        
        DayHitDurClassAvg(dId, cId) = mean(HitDur(cindex_class)*settings.spectrogram.wshift);
        DayHitDurClassStd(dId, cId) = std(HitDur(cindex_class)*settings.spectrogram.wshift);
    end
end

%% Correlation
[RunCorr, RunPVal] = corr((1:NumRuns)', RunHitDurAllAvg, 'rows', 'pairwise');

RunCorrClass = nan(NumClasses, 1);
RunPValClass = nan(NumClasses, 1);
for cId = 1:NumClasses
    [RunCorrClass(cId), RunPValClass(cId)] = corr((1:NumRuns)', RunHitDurClassAvg(:, cId), 'rows', 'pairwise');
end

[DayCorr, DayPVal] = corr((1:NumDays)', DayHitDurAllAvg, 'rows', 'pairwise');

DayCorrClass = nan(NumClasses, 1);
DayPValClass = nan(NumClasses, 1);
for cId = 1:NumClasses
    [DayCorrClass(cId), DayPValClass(cId)] = corr((1:NumDays)', DayHitDurClassAvg(:, cId), 'rows', 'pairwise');
end

%% Plotting performances per run and per day
fig1 = figure;
fig_set_position(fig1, 'Top');

NumRows = 1;
NumCols = 2;

% Performances per run
subplot(NumRows, NumCols, 1);

hold on;
plot(RunHitDurAllAvg, '.k', 'MarkerSize', 6);

plot(RunHitDurClassAvg, '.');
lsline;
hold off;
grid on;
xlabel('Run');
ylabel('Time [s]');
title(['Hit duration over runs (r=' num2str(RunCorr, '%4.3f') ', p<' num2str(RunPVal, '%4.3f') ')']);
legend('Average', ClassNames{1}, ClassNames{2}, 'Location', 'SouthEast');

% Performances per day
subplot(NumRows, NumCols, 2);

hold on;
plot(DayHitDurAllAvg, 'k', 'LineWidth', 1);
plot(DayHitDurClassAvg, '.');
lsline;
hold off;
grid on;
xlabel('Day');
ylabel('Time [s]');
title(['Hit duration over days (r=' num2str(DayCorr, '%4.3f') ', p<' num2str(DayPVal, '%4.3f') ')']);
legend('Average', ClassNames{1}, ClassNames{2}, 'Location', 'SouthEast');

sgtitle([subject '- Hit duration']);

%% Saving figures
filename1 = fullfile(figdir, [subject '_bci_hit_duration.pdf']);
util_bdisp(['[out] - Exporting bci hit duration to ' filename1]);
fig_export(fig1, filename1, '-pdf');
