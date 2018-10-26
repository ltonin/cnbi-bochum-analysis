clearvars; clc;

subject = 'BOCH01';

pattern   = [subject '*.online.mi.mi_bhbf'];
spatialfilter = 'laplacian';
datapath    = ['analysis/psd/' spatialfilter '/'];
savedir     = ['analysis/bci/' spatialfilter '/'];

ClassEventId     = [773 771];
CFeedbackEventId = 781;
files = util_getfile(datapath, '.mat', pattern);
nfiles = length(files);

% Create analysis directory
util_mkdir('./', savedir);


%% Concatenate data
util_bdisp(['[io] - Importing ' num2str(nfiles) ' files from ' datapath ':']);
[F, events, labels, classifiers, settings] = cnbibochum_concatenate_data(files);

U = log(F);
NumSamples = size(U, 1);
NumChans = size(U, 3);
NumFreqs = size(U, 2);


%% Get general events
[TrialLabels, TrialEvents] = proc_get_event2(CFeedbackEventId, NumSamples, events.POS, events.TYP, events.DUR);
[~, ClassEvents] = proc_get_event2(ClassEventId, NumSamples, events.POS, events.TYP, events.DUR);

NumTrials = length(TrialEvents.TYP);

Ck = zeros(NumSamples, 1);
Rk = zeros(NumSamples, 1);
Dk = zeros(NumSamples, 1);
Wk = zeros(NumSamples, 1);
Nk = zeros(NumSamples, 1);
Mk = zeros(NumSamples, 1);
Pk = zeros(NumSamples, 1);

for trId = 1:NumTrials
    cstart = TrialEvents.POS(trId);
    cstop  = cstart + TrialEvents.DUR(trId) - 1;
    cclass = ClassEvents.TYP(trId);
    Ck(cstart:cstop) = cclass;
    Rk(cstart:cstop) = unique(labels.Rk(cstart:cstop));
    Dk(cstart:cstop) = unique(labels.Dk(cstart:cstop));
    Wk(cstart:cstop) = unique(labels.Wk(cstart:cstop));
    Nk(cstart:cstop) = unique(labels.Nk(cstart:cstop));
    Mk(cstart:cstop) = unique(labels.Mk(cstart:cstop));
    Pk(cstart:cstop) = unique(labels.Pk(cstart:cstop));
end

%% Computing fisher score per run
util_bdisp(['[proc] - Computing fisher score per run']);
Runs = unique(labels.Rk);
NumRuns = length(Runs);
rfisher = nan(NumChans*NumFreqs, NumRuns);
rMk = zeros(NumRuns, 1);
rPk = zeros(NumRuns, 1);
rDk = zeros(NumRuns, 1);
rWk = zeros(NumRuns, 1);
rNk = zeros(NumRuns, 1);
for rId = 1:NumRuns
    cindex = Rk == Runs(rId);
    if sum(cindex) == 0
        continue;
    end
    rfisher(:, rId) = proc_fisher2(U(cindex, :, :), Ck(cindex));
    
    rMk(rId) = unique(Mk(cindex));
    rPk(rId) = unique(Pk(cindex));
    rDk(rId) = unique(Dk(cindex));
    rWk(rId) = unique(Wk(cindex));
    rNk(rId) = unique(Nk(cindex));
end

%% Computing fisher score per day
util_bdisp(['[proc] - Computing fisher score per day']);
Months = unique(labels.Dk);
NumMonths = length(Months);
dfisher = nan(NumChans*NumFreqs, NumMonths);
dDk = zeros(NumMonths, 1);
dWk = zeros(NumMonths, 1);
dNk = zeros(NumMonths, 1);
for dId = 1:NumMonths
    cindex = Dk == Months(dId);
    
    dfisher(:, dId) = proc_fisher2(U(cindex, :, :), Ck(cindex));
    
    dDk(dId) = unique(Dk(cindex));
    dWk(dId) = unique(Wk(cindex));
    dNk(dId) = unique(Nk(cindex));
end

%% Computing fisher score per week
util_bdisp(['[proc] - Computing fisher score per week']);
Weeks = unique(labels.Wk);
NumWeeks = length(Weeks);
wfisher = nan(NumChans*NumFreqs, NumWeeks);
wWk = zeros(NumWeeks, 1);
wNk = zeros(NumWeeks, 1);
for wId = 1:NumWeeks
    cindex = Wk == Weeks(wId);
    
    wfisher(:, wId) = proc_fisher2(U(cindex, :, :), Ck(cindex));
    
    wWk(wId) = unique(Wk(cindex));
    wNk(wId) = unique(Nk(cindex));
end

%% Computing fisher score per week
util_bdisp(['[proc] - Computing fisher score per month']);
Months = unique(labels.Nk);
NumMonths = length(Months);
mfisher = nan(NumChans*NumFreqs, NumMonths);
nNk = zeros(NumMonths, 1);
for nId = 1:NumMonths
    cindex = Nk == Months(nId);
    
    mfisher(:, nId) = proc_fisher2(U(cindex, :, :), Ck(cindex));
    nNk(nId) = unique(Nk(cindex));
end

%% Saving files

fisher.run   = rfisher;
fisher.day   = dfisher;
fisher.week  = wfisher;
fisher.month = mfisher;

tmplbl = labels;
labels = [];
labels.sample = tmplbl;

labels.run.Mk = rMk;
labels.run.Pk = rPk;
labels.run.Dk = rDk;
labels.run.Wk = rWk;
labels.run.Nk = rNk;

labels.day.Dk = dDk;
labels.day.Wk = dWk;
labels.day.Nk = dNk;

labels.week.Wk = wWk;
labels.week.Nk = wNk;

labels.month.Nk = nNk;

savepath = [savedir '/' subject '_fisher_score.mat'];
util_bdisp(['[out] - Saving bci online fisher in ' savepath]);
save(savepath, 'fisher', 'labels', 'classifiers', 'settings');
