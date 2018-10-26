clearvars; clc;

subject = 'BOCH01';
experiment  = 'mi_wheelchair';

pattern   = [subject '.20181010.*.online.mi.mi_bhbf.*.'];

spatialfilter = 'laplacian';
datapath    = ['analysis/psd/' spatialfilter '/'];
savedir     = ['analysis/bci/' spatialfilter '/'];
figdir      = 'figures/';

ClassEventId     = [773 771];
NumClasses    = length(ClassEventId);
ClassLabels = {'mi\_both\_hands', 'mi\_both\_feet'};
CFeedbackEventId = 781;
files = util_getfile(datapath, '.mat', pattern);
nfiles = length(files);

% Create analysis directory
util_mkdir('./', savedir);


%% Concatenate data
util_bdisp(['[io] - Importing ' num2str(nfiles) ' files from ' datapath ' for subject ' subject ':']);
[F, events, labels, classifiers, settings] = cnbibochum_concatenate_data(files);

U = log(F);
P = proc_reshape_ts_bc(U);
NumSamples = size(F, 1);
NumChans = size(F, 3);
NumFreqs = size(F, 2);
NumFeatures = size(P, 2);
freqgrid = settings.spectrogram.freqgrid;


%% Get general events
[TrialLabels, TrialEvents] = proc_get_event2(CFeedbackEventId, NumSamples, events.POS, events.TYP, events.DUR);
[~, ClassEvents] = proc_get_event2(ClassEventId, NumSamples, events.POS, events.TYP, events.DUR);

NumTrials = length(TrialEvents.TYP);

Ck = zeros(NumSamples, 1);
Rk = zeros(NumSamples, 1);
rRk = zeros(NumTrials, 1);
for trId = 1:NumTrials
    cstart = TrialEvents.POS(trId);
    cstop  = cstart + TrialEvents.DUR(trId) - 1;
    cclass = ClassEvents.TYP(trId);
    Ck(cstart:cstop) = cclass;
    Rk(cstart:cstop) = unique(labels.Rk(cstart:cstop));
    rRk(trId) = unique(labels.Rk(cstart:cstop));
end

Xk = events.TYP(events.TYP == 897 | events.TYP == 898);

Runs = unique(labels.Rk);
NumRuns = length(Runs);

%% Computing CVA per classifier
util_bdisp('[proc] - Computing cva per run');
cva = nan(NumChans*NumFreqs, NumRuns);
for rId = 1:NumRuns
    % Get index for the current run
    cindex = Rk == rId;
    % Compute CVA
    cva(:, rId) = cva_tun_opt(P(cindex, :), Ck(cindex));
end

%% Compute canonical space projection per run (with selected features)
util_bdisp('[proc] - Computing canonical space projection per run (with selected features)');
cs  = cell(NumRuns, 1);
pdf = cell(NumClasses, NumRuns); 
for rId = 1:NumRuns
    % Get index for the current run
    cindex = Rk == rId;
    % Get features selected in this run
    FeatureIdx = proc_cnbifeature2bin(classifiers(rId).features, freqgrid);  
    
    % Compute canonical space
    [~, ~, ~, ~, cs{rId}] = cva_tun_opt(P(cindex, FeatureIdx), Ck(cindex));
    
    for cId = 1:NumClasses
        ccs = cs{rId};
        pdf{cId, rId} = gkdeb(ccs(Ck(cindex) == ClassEventId(cId)));
    end
end

%% Get the discriminant power of the selected features
dp = zeros(NumRuns, 1);
for rId = 1:NumRuns
    FeatureIdx = proc_cnbifeature2bin(classifiers(rId).features, freqgrid);  
    dp(rId) = sum(cva(FeatureIdx, rId));
end

%% Apply classifier
util_bdisp('[proc] - Applying classifier (all runs together)');
pp = 0.5*ones(NumSamples, 2);
for sId = 1:NumSamples
    runId = labels.Rk(sId);
    
    % Importing feature indexes used by the classifier
    FeatureIdx = proc_cnbifeature2bin(classifiers(runId).features, freqgrid);   

    % Applying the classifier
    [~, pp(sId, :)] = gauClassifier(classifiers(runId).gau.M, classifiers(runId).gau.C, P(sId, FeatureIdx));
end

%% Compute Single sample Accuracy
Accuracy  = nan(NumRuns, 1);
Rejection = nan(NumRuns, 1);
AccuracyClass  = nan(NumClasses, NumRuns);
RejectionClass = nan(NumClasses, NumRuns);
for rId = 1:NumRuns
    cindex = Rk == rId;
    cpp = pp(cindex, 1);
    
    % Get current rejection
    crejection = classifiers.rejection;
    
    % Get current rejected samples
    crejected_samples = cpp >= (1 - crejection) & cpp <= crejection;
     
    % Get current guessed samples (class)
    cguessed_samples = cpp;
    cguessed_samples(cpp>=0.5) = 773;
    cguessed_samples(cpp<0.5)  = 771;
    
    % Get current real labels (class)
    creal_labels = Ck(cindex);
    
    Accuracy(rId)  = sum(cguessed_samples(crejected_samples == false) == creal_labels(crejected_samples == false))./length(creal_labels(crejected_samples == false));
    Rejection(rId) = sum(crejected_samples)./length(crejected_samples);
    
    for cId = 1:NumClasses
        cindex_accuracy  = crejected_samples == false & creal_labels == ClassEventId(cId);
        cindex_rejection = crejected_samples & creal_labels == ClassEventId(cId);
        AccuracyClass(cId, rId)  = sum(cguessed_samples(cindex_accuracy) == creal_labels(cindex_accuracy))./length(creal_labels(cindex_accuracy));
        RejectionClass(cId, rId) = sum(cindex_rejection)./length(cindex_rejection);
    end
end

%% Compute trial accuracy
AccuracyTrial = nan(NumRuns, 1);
for rId = 1:NumRuns
    cindex = rRk == Runs(rId);
    cXk = Xk(cindex) == 897;
    AccuracyTrial(rId) = sum(cXk)./length(cXk);
end

%% Figures

fig1 = figure;
fig_set_position(fig1, 'All');

ReqFreqs = 4:48;
[SelFreqs, SelFreqsId] = intersect(freqgrid,ReqFreqs);
color = [0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0 0 0];
hcs = zeros(NumRuns, 1);
NumRows = NumRuns;
NumCols = 3;
clim = [0 2];
for rId = 1:NumRuns
    
    % Accuracy/Rejection plot
    subplot(NumRows, NumCols, 1 + (rId-1)*NumCols);
    superbar(100*[Accuracy(rId); AccuracyTrial(rId)], 'BarFaceColor', color, 'BarWidth', .4);

    set(gca, 'XTick', 1:2);
    set(gca, 'XTickLabel', {'single sample', 'trial'});
    ylim([0 110]);
    xlim([0.5 2.5]);
    grid on;
    ylabel('[%]');
    title(['Run ' num2str(rId) ' - Accuracy']);
   
    % Canonical space plot
    subplot(NumRows, NumCols, 2 + (rId-1)*NumCols);
    hold on;
    for cId = 1:NumClasses
        cpdf = pdf{cId, rId};
         plot(cpdf.x, cpdf.f, 'LineWidth', 2);
    end
    hcs(rId) = gca;
    hold off;
    legend(ClassLabels);
    grid on;
    title(['Run ' num2str(rId) ' - Canonical space']);
   
    % CVA plot
    subplot(NumRows, NumCols, 3 + (rId-1)*NumCols);
    cfisher = reshape(cva(:, rId), NumFreqs, NumChans);
    imagesc(SelFreqsId, 1:NumChans, cfisher(SelFreqsId, :)');
    FeatureIdx = proc_cnbifeature2bin(classifiers(runId).features, freqgrid);
    [freqId, chanId] = proc_bin2bc([NumFreqs NumChans], FeatureIdx);
    hold on;
    for fId = 1:length(freqId)
       plot(freqId(fId), chanId(fId), 'or', 'MarkerSize', 10, 'LineWidth', 4);
    end
    hold off;

    set(gca, 'XTick', 1:length(SelFreqsId));
    set(gca, 'XTickLabel', num2str(SelFreqs));
    set(gca, 'YTick', 1:NumChans);
    set(gca, 'YTickLabel', settings.data.lchannels);
    title(['Run ' num2str(rId) ' - CVA']);
   
end

plot_set_limits(reshape(hcs, numel(hcs), 1), 'both', 'minmax')

suptitle(['Subject ' subject]);


%% Figure correlation dp/accuracy
fig2 = figure;
plot(dp, Accuracy, 'o', 'LineWidth', 2, 'MarkerSize', 5);
[c, p] = corr(dp, Accuracy);
lsline;
grid on;
title(['Subject ' subject ' - Correlation=' num2str(c) ' p=' num2str(p)]);
xlabel('Discriminant power');
ylabel('Single sample accuracy [%]');

%% Saving figure
figfilename1 = [figdir '/' subject '_accuracy_analysis.pdf'];
util_bdisp(['[fig] - Saving figure in: ' figfilename1]);
fig_figure2pdf(fig1, figfilename1)


figfilename2 = [figdir '/' subject '_accuracy_dp_correlation.pdf'];
util_bdisp(['[fig] - Saving figure in: ' figfilename2]);
fig_figure2pdf(fig2, figfilename2) 

