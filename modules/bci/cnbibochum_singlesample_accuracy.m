clearvars; clc;

subject = 'BOCH05';

includepat  = {subject, 'mi', 'mi_bhbf', 'online'};
excludepat  = {'guided', 'control'};
spatialfilter = 'laplacian';
artifactrej   = 'none'; % {'FORCe', 'none'}
datapath    = ['analysis/' artifactrej '/psd/' spatialfilter '/'];
savedir     = ['analysis/' artifactrej '/accuracy/' spatialfilter '/'];

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

Ck = zeros(NumSamples, 1);
tRk = zeros(NumTrials, 1);
for trId = 1:NumTrials
    cstart = TrialEvent.POS(trId);
    cstop  = cstart + TrialEvent.DUR(trId) - 1;
    cclass = ClassEvent.TYP(trId);
    Ck(cstart:cstop) = cclass;
    tRk(trId) = unique(labels.samples.Rk(cstart:cstop));
end

Xk = events.TYP(events.TYP == 897 | events.TYP == 898);

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
    cindex = labels.samples.Rk == Runs(rId);
    
    cindex_classes = false(NumSamples, NumClasses);
    
    for cId = 1:NumClasses
        cidx_c = Ck == Classes(cId) & cindex;
        cindex_classes(:, cId) = cidx_c;
    end
    
    if(diff(sum(cindex_classes)) == 0)
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

% %% Apply classifier
% util_bdisp('[proc] - Applying classifier (all runs together)');
% pp = 0.5*ones(NumSamples, 2);
% for sId = 1:NumSamples
%     util_disp_progress(sId, NumSamples, '            ');
%   
%     runId = labels.samples.Rk(sId);
%     
%     % Importing feature indexes used by the classifier
%     FeatureIdx = proc_cnbifeature2bin(classifiers(runId).features, SrcFreqs);   
% 
%     % Applying the classifier
%     [~, pp(sId, :)] = gauClassifier(classifiers(runId).gau.M, classifiers(runId).gau.C, P(sId, FeatureIdx));
% end


%% Computing single sample accuracy for each online

ConfMatr = zeros(NumClasses, NumClasses+1, NumRuns);
for rId = 1:NumRuns
    
    cindex = labels.samples.Rk == Runs(rId) & Ck > 0;
    
    if length(unique(Ck(cindex))) ~= 2
        continue;
    end
    
%     % Classifier identification
%     cclassifierId = unique(Xk(cindex));
%     
%     if cclassifierId == 0
%         continue;
%     end
%     
%     if(length(cclassifierId) ~= 1)
%         error('chk:cls', [num2str(length(cclassifierId)) ' classifiers found in the same run (' num2str(Runs(rId)) ')']);
%     end
    cclassifier = classifiers(rId);
    
    % Feature extraction
    findices  = cnbiproc_features2indices(cclassifier.features, settings.spectrogram.freqgrid);
    cfeatures = P(cindex, findices);
    reallb    = Ck(cindex);
    
    cprobs = zeros(size(cfeatures, 1), 2);
    for sId = 1:size(cfeatures, 1)
        [~, cprobs(sId, :)] = gauClassifier(cclassifier.gau.M, cclassifier.gau.C, cfeatures(sId, :));
    end
    
    ConfMatr(:, :, rId) = cnbiproc_confusionmat(reallb, cprobs, 'classes', (cclassifier.classes)', 'rejection', cclassifier.rejection);
end

%% Computing overall accuracy/rejection for each run
RunAccuracy  = nan(NumRuns, 1);
RunRejection = nan(NumRuns, 1);
for rId = 1:NumRuns
    RunAccuracy(rId) = trace(ConfMatr(:, 1:NumClasses, rId))/NumClasses;
    RunRejection(rId) = mean(ConfMatr(:, 3, rId));
end

%% Computing overall accuracy/rejection for each day

DayAccuracy  = nan(NumDays, 2);
DayRejection = nan(NumDays, 2);
for dId = 1:NumDays
    cruns = unique(labels.samples.Rk(labels.samples.Dk == Days(dId)));
    
    DayAccuracy(dId, 1) = mean(RunAccuracy(cruns));
    DayAccuracy(dId, 2) = std(RunAccuracy(cruns));
    
    DayRejection(dId, 1) = mean(RunRejection(cruns));
    DayRejection(dId, 2) = std(RunRejection(cruns));
end

RunPerf = nan(NumRuns, 1);

for rId = 1:NumRuns
    RunPerf(rId) = 100*sum(Xk(tRk == rId) == 897)./sum(tRk == rId);
    
end

%% Saving
% probabilities.raw = pp;
% labels.samples.Bk = Bk;
% labels.samples.Ck = Ck;
% 
% savepath = [savedir '/' subject '_probabilities_' settings.spatial.filter '.mat'];
% util_bdisp(['[out] - Saving bci online probabilities in ' savepath]);
% save(savepath, 'probabilities', 'labels', 'classifiers', 'settings');
