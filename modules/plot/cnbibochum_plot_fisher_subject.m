clearvars; clc; close all;

subject = 'BOCH01';

spatialfilter = 'laplacian';
pattern   = [subject '_fisher_score.mat'];
datapath  = ['analysis/bci/' spatialfilter '/'];

%% Loading fisher score data
util_bdisp(['[io] - Importing fisher score data for subject ' subject ' from ' datapath]);
data = load([datapath pattern]);

FreqGrid = data.settings.spectrogram.freqgrid;
ReqFreqs = 1:48;
[SelFreqs, SelFreqIds] = intersect(FreqGrid, ReqFreqs);
ChannelLabels = data.settings.data.lchannels;
NumChans = data.settings.data.nchannels;
NumFreqs = length(FreqGrid);

Months = unique(data.labels.day.Nk);
NumMonths = length(Months);
Weeks = unique(data.labels.day.Wk);
NumWeeks = length(Weeks);
NumRuns = size(data.fisher.run, 2);

%% Computing the sum of the sisher score for the selected features per run
sfisher = nan(NumRuns, 1);
for rId = 1:NumRuns
   cfisher = data.fisher.run(:, rId);
   cfeatures = data.classifiers.features;
   cfeatureIdx = proc_cnbifeature2bin(cfeatures, FreqGrid);
   
   sfisher(rId) = sum(cfisher(cfeatureIdx));
end

%% Plotting

% DP maps per week
fig1 = figure;
fig_set_position(fig1, 'Top');

NumRows = 1;
NumCols = NumWeeks;
clim = [0 0.5];
for wId = 1:NumWeeks
    
    cnruns = sum(data.labels.run.Wk == wId);
    
    
    subplot(NumRows, NumCols, wId);
    cfisher = reshape(data.fisher.week(:, wId), NumFreqs, NumChans);
    
    imagesc(SelFreqs,1:NumChans,  cfisher(SelFreqIds, :)', clim);
    set(gca, 'YTick', 1:NumChans);
    set(gca, 'YTickLabel', ChannelLabels);
    
    xlabel('[Hz]');
    ylabel('channel');
    
    title(['Week ' num2str(wId) ' (NRuns=' num2str(cnruns) ')']);
end
suptitle([subject '- DP maps per week']);


% DP maps per month
fig2 = figure;
fig_set_position(fig2, 'Top');

NumRows = 1;
NumCols = NumMonths;
clim = [0 0.5];
for nId = 1:NumMonths
    
    cnruns = sum(data.labels.run.Nk == nId);
    
    
    subplot(NumRows, NumCols, nId);
    cfisher = reshape(data.fisher.month(:, nId), NumFreqs, NumChans);
    
    imagesc(SelFreqs,1:NumChans,  cfisher(SelFreqIds, :)', clim);
    set(gca, 'YTick', 1:NumChans);
    set(gca, 'YTickLabel', ChannelLabels);
    
    xlabel('[Hz]');
    ylabel('channel');
    
    title(['Month ' num2str(wId) ' (NRuns=' num2str(cnruns) ')']);
end
suptitle([subject '- DP maps per month']);
    

% Fisher score for selected featuers per run
fig3 = figure;

    
plot(sfisher, '.');
lsline;
[c, p] = corr((1:NumRuns)', sfisher, 'rows', 'pairwise');
title(['Fisher score selected features (corr=' num2str(c, '%3.2f') ', p<' num2str(p, '%3.2f') ')']);
grid on;
xlabel('Run');
ylabel('[]');
