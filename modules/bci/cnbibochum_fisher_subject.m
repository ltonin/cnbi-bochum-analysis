clearvars; clc;

subject = 'BOCH01';

includepat  = {subject, 'mi', 'mi_bhbf'};
excludepat  = {'guided', 'control'};
spatialfilter = 'laplacian';
artifactrej   = 'none'; % {'FORCe', 'none'}
datapath    = ['analysis/' artifactrej '/psd/' spatialfilter '/'];
savedir     = ['analysis/' artifactrej '/discriminancy/' spatialfilter '/'];

ClassEventId     = [773 771];
CFeedbackEventId = 781;
files = util_getfile3(datapath, '.mat', 'include', includepat, 'exclude', excludepat);
nfiles = length(files);
util_bdisp(['[io] - Found ' num2str(nfiles) ' files with the inclusion/exclusion criteria: (' strjoin(includepat, ', ') ') / (' strjoin(excludepat, ', ') ')']);

% Create analysis directory
util_mkdir('./', savedir);

%% Concatenate data
util_bdisp(['[io] - Importing ' num2str(nfiles) ' files from ' datapath ':']);
[F, events, labels, classifiers, settings] = cnbibochum_concatenate_data(files);

U = log(F);
NumSamples = size(U, 1);
NumChans   = size(U, 3);
NumFreqs   = size(U, 2);

%% Get general events
[~, TrialEvent] = proc_get_event2(CFeedbackEventId, NumSamples, events.POS, events.TYP, events.DUR);
[ClassLabels, ClassEvent] = proc_get_event2(ClassEventId, NumSamples, events.POS, events.TYP, events.DUR);

NumTrials  = length(TrialEvent.TYP);
Classes    = setdiff(unique(ClassLabels), 0);
NumClasses = length(Classes);

Ck = zeros(NumSamples, 1);
Rk = zeros(NumSamples, 1);
Dk = zeros(NumSamples, 1);
Wk = zeros(NumSamples, 1);
Nk = zeros(NumSamples, 1);
Mk = zeros(NumSamples, 1);
Fk = zeros(NumSamples, 1);
Pk = zeros(NumSamples, 1);

for trId = 1:NumTrials
    cstart = TrialEvent.POS(trId);
    cstop  = cstart + TrialEvent.DUR(trId) - 1;
    cclass = ClassEvent.TYP(trId);
    Ck(cstart:cstop) = cclass;
    Rk(cstart:cstop) = unique(labels.samples.Rk(cstart:cstop));
    Dk(cstart:cstop) = unique(labels.samples.Dk(cstart:cstop));
    Wk(cstart:cstop) = unique(labels.samples.Wk(cstart:cstop));
    Nk(cstart:cstop) = unique(labels.samples.Nk(cstart:cstop));
    Mk(cstart:cstop) = unique(labels.samples.Mk(cstart:cstop));
    Pk(cstart:cstop) = unique(labels.samples.Pk(cstart:cstop));
    Fk(cstart:cstop) = unique(labels.samples.Fk(cstart:cstop));
end

%% Generic Labels
Runs    = unique(labels.samples.Rk);
Days    = unique(labels.samples.Dk);
Weeks   = unique(labels.samples.Wk);
Months  = unique(labels.samples.Nk);

NumRuns   = length(Runs);
NumDays   = length(Days);
NumWeeks  = length(Weeks);
NumMonths = length(Months);

%% Balancing the classes per run
Bk = false(NumSamples, 1);
for rId = 1:NumRuns
    cindex = Rk == Runs(rId);
    
    cindex_classes = false(NumSamples, NumClasses);
    
    for cId = 1:NumClasses
        cidx_c = Ck == Classes(cId) & cindex;
        cindex_classes(:, cId) = cidx_c;
    end
    
    if(diff(sum(cindex_classes)) == 0)
        Bk(cindex) = true;
        continue;
    end
    
    [minlen, mincls] = min(sum(cindex_classes));
    othcls = setdiff(1:NumClasses, mincls);
    
    cidx = find(cindex_classes(:, othcls));
    %cidx_e = cidx(randperm(length(cidx), minlen));
    cidx_e = cidx(1:minlen);
    cindex_classes_e = cindex_classes;
    cindex_classes_e(:, othcls) = false;
    cindex_classes_e(cidx_e, othcls) = true;
  
    cindex_f = false(size(cindex_classes_e, 1), 1);
    
    Bk(cindex) = false;
    for cId = 1:NumClasses
        cindex_f = cindex_f | cindex_classes_e(:, cId);
        Bk(cindex_classes_e(:, cId)) = true;
    end
    
    
end

%% Computing fisher score per run
util_bdisp('[proc] - Computing fisher score per run');
nstd = [];

rfisher = nan(NumChans*NumFreqs, NumRuns);
rMk = zeros(NumRuns, 1);
rPk = zeros(NumRuns, 1);
rFk = zeros(NumRuns, 1);
rDk = zeros(NumRuns, 1);
rWk = zeros(NumRuns, 1);
rNk = zeros(NumRuns, 1);
for rId = 1:NumRuns
    cindex = Rk == Runs(rId);
    
    if sum(cindex) == 0 
        warning(['skip run: ' num2str(rId)]);
        
        continue;
    end
    rfisher(:, rId) = proc_fisher2(U(cindex & Ck > 0, :, :), Ck(cindex & Ck > 0), nstd, true);
    
    
    rMk(rId) = unique(Mk(cindex));
    rPk(rId) = unique(Pk(cindex));
    rFk(rId) = unique(Fk(cindex));
    rDk(rId) = unique(Dk(cindex));
    rWk(rId) = unique(Wk(cindex));
    rNk(rId) = unique(Nk(cindex));
end

%% Saving files

fisher.run   = rfisher;

% Samples labels
labels = [];

labels.samples.Bk = Bk;
labels.samples.Ck = Ck;
labels.samples.Rk = Rk;
labels.samples.Dk = Dk;
labels.samples.Wk = Wk;
labels.samples.Nk = Nk;
labels.samples.Mk = Mk;
labels.samples.Fk = Fk;
labels.samples.Pk = Pk;

% Run labels
labels.run.Mk = rMk;
labels.run.Fk = rFk;
labels.run.Pk = rPk;
labels.run.Dk = rDk;
labels.run.Wk = rWk;
labels.run.Nk = rNk;

savepath = [savedir '/' subject '_fisher_' settings.spatial.filter '.mat'];
util_bdisp(['[out] - Saving bci online fisher in ' savepath]);
save(savepath, 'fisher', 'labels', 'classifiers', 'settings');
