clearvars; clc; close all;

subject = 'BOCH01';

spatialfilter = 'laplacian';
pattern   = [subject '_fisher_' spatialfilter '.mat'];
datapath  = ['analysis/none/discriminancy/' spatialfilter '/'];


%% Loading fisher score data
util_bdisp(['[io] - Importing fisher score data for subject ' subject ' from ' datapath]);
data = load([datapath pattern]);

Months      = unique(data.labels.day.Nk);
NumMonths   = length(Months);
Weeks       = unique(data.labels.day.Wk);
NumWeeks    = length(Weeks);
NumRuns     = size(data.fisher.run, 2);

SrcChans   = data.settings.data.lchannels;
SrcFreqs   = data.settings.spectrogram.freqgrid;
NumSrcChan = length(SrcChans);
NumSrcFreq = length(SrcFreqs);

%% Define required frequencies and channels
ReqFreqs = 1:48;
ReqChans = { 'F1',  'Fz',  'F2', ...
            'FC5', 'FC3', 'FC1', 'FCz', 'FC2', 'FC4', 'FC6', ...
             'C5',  'C3',  'C1',  'Cz',  'C2',  'C4',  'C6', ...
            'CP5', 'CP3', 'CP1',        'CP2', 'CP4', 'CP6', ...
             'P5',  'P3',  'P1',  'Pz',  'P2',  'P4',  'P6', ... 
            'POz'};

[SelFreqs, SelFreqIds] = intersect(SrcFreqs, ReqFreqs);
SelChanIds = proc_get_channel(ReqChans, SrcChans);
SelChans   = SrcChans(SelChanIds);

NumSelChans = length(SelChanIds);
NumSelFreqs = length(SelFreqIds);

% ReqLatChans = {'FC5', 'FC3', 'FC1', 'FC2', 'FC4', 'FC6', ...
%                 'C5',  'C3',  'C1',  'C2',  'C4',  'C6', ...
%                'CP5', 'CP3', 'CP1', 'CP2', 'CP4', 'CP6'};
ReqLatChans = {'C3',  'C1',  'C2',  'C4',  ...
               'CP3', 'CP1', 'CP2', 'CP4'};

ReqMedChans = {'FCz', 'Cz', 'Pz'};

LatChanIds = proc_get_channel(ReqLatChans, SrcChans);
LatChans   = SrcChans(LatChanIds);
MedChanIds = proc_get_channel(ReqMedChans, SrcChans);
MedChans   = SrcChans(MedChanIds);

ReqRangeFreq  = [6:14 18:28];
[RangeFreqs, RangeFreqIds] = intersect(SrcFreqs, ReqRangeFreq);


%% Select runs to be removed
switch(subject)
    case 'BOCH01'
        RemRuns = [86 87 91 92 95]; 
    case 'BOCH04'
        RemRuns = [4 5 6 7 17 18 32 33 34 35 40 43]; 
    otherwise
        RemRuns = [];
end

%% Computing the sum of the fisher score for the selected features per run
sfisher = nan(NumRuns, 1);
for rId = 1:NumRuns
    if (data.labels.run.Mk(rId) == 1) || ismember(rId, RemRuns)
        continue
    end
   cfisher = data.fisher.run(:, rId);
   cfeatures = data.classifiers(rId).features;
   cfeatureIdx = proc_cnbifeature2bin(cfeatures, SrcFreqs);
   
   sfisher(rId) = nanmean(cfisher(cfeatureIdx));
end

%% Computing the sum of the fisher score for the medial and lateral channels (in alpha/beta)
mlfisher = nan(2, NumRuns);

for rId = 1:NumRuns
    if (data.labels.run.Mk(rId) == 1) || ismember(rId, RemRuns)
        continue
    end
   cfisher = reshape(data.fisher.run(:, rId), NumSrcFreq, NumSrcChan);
   cfisher_m = nanmean(nanmean(cfisher(RangeFreqIds, MedChanIds)));
   cfisher_l = nanmean(nanmean(cfisher(RangeFreqIds, LatChanIds)));
   
   mlfisher(:, rId) = [cfisher_m cfisher_l];
end

%% Reshaping fisher scores and extract freqs and channels
wfisher = reshape(data.fisher.week, NumSrcFreq, NumSrcChan, NumWeeks);
wfisher = wfisher(SelFreqIds, SelChanIds, :);

% mfisher = reshape(data.fisher.month, NumSrcFreq, NumSrcChan, NumMonths);
% mfisher = mfisher(SelFreqIds, SelChanIds, :);



%% Plotting

% DP maps per week
fig1 = figure;
fig_set_position(fig1, 'All');


[NumRows, NumCols] = plot_subplot_size(NumWeeks, 'maxcols', 4);

for wId = 1:NumWeeks
    
    cnruns = sum(data.labels.run.Wk == wId);
    
    
    subplot(NumRows, NumCols, wId);
    cfisher = wfisher(:, :, wId);
    
    imagesc(cfisher');
    axis square
    set(gca, 'XTick', 1:NumSelFreqs);
    set(gca, 'XTickLabel', SelFreqs);
    set(gca, 'YTick', 1:NumSelChans);  
    set(gca, 'YTickLabel', SelChans);
    
    xlabel('[Hz]');
    ylabel('channel');
    
    title(['Week ' num2str(wId) ' (NRuns=' num2str(cnruns) ')']);
end
sgtitle([subject '- DP maps per week']);


% DP maps per month
fig2 = figure;
fig_set_position(fig2, 'Top');

% 
% [NumRows, NumCols] = plot_subplot_size(NumMonths, 'maxcols', 4);
% 
% for nId = 1:NumMonths
%     
%     cnruns = sum(data.labels.run.Nk == nId);
%     
%     
%     subplot(NumRows, NumCols, nId);
%     cfisher = mfisher(:, :, nId);
%     
%     imagesc(cfisher');
%     axis square
%     set(gca, 'XTick', 1:NumSelFreqs);
%     set(gca, 'XTickLabel', SelFreqs);
%     set(gca, 'YTick', 1:NumSelChans);
%     set(gca, 'YTickLabel', SelChans);
%     
%     xlabel('[Hz]');
%     ylabel('channel');
%     
%     title(['Month ' num2str(nId) ' (NRuns=' num2str(cnruns) ')']);
% end
sgtitle([subject '- DP maps per month']);
    

%% Fisher score for selected featuers per run and for medial and lateral channels
fig3 = figure;
fig_set_position(fig3, 'Top');

Cl = {data.classifiers.filename};
Clk = zeros(length(Cl), 1);
count = 0; 
for i = 2:length(Cl)
    if isequal(Cl{i}, Cl{i-1}) == false
        count = count+1; 
    end
    Clk(i) = count; 
end

ClIds = find(diff(Clk));


% Selected features
subplot(1, 2, 1);
p_sfisher = sfisher;
p_sfisher(isinf(sfisher)) = nan;

plot(1:NumRuns, p_sfisher, '.');
lsline;
[c, p] = corr((1:NumRuns)', p_sfisher, 'rows', 'pairwise');
plot_vline(ClIds, 'k');
title('Fisher score for selected features');
grid on;
legend(['selected features (corr=' num2str(c, '%3.2f') ', p<' num2str(p, '%3.2f') ')']);
xlabel('Run');
ylabel('[]');

% Medial/Lateral channels
subplot(1, 2, 2);  
p_mlfisher = mlfisher;
p_mlfisher(1, isinf(mlfisher(1, :))) = nan;
p_mlfisher(2, isinf(mlfisher(2, :))) = nan;
plot(1:NumRuns, p_mlfisher', '.');
lsline;
[c_m, p_m] = corr((1:NumRuns)', p_mlfisher(1, :)', 'rows', 'pairwise');
[c_l, p_l] = corr((1:NumRuns)', p_mlfisher(2, :)', 'rows', 'pairwise');
plot_vline(ClIds, 'k');
title('Fisher score for medial/lateral channels'); 
grid on;
legend(['medial (corr=' num2str(c_m, '%3.2f') ', p<' num2str(p_m, '%3.2f') ')'], ['lateral (corr=' num2str(c_l, '%3.2f') ', p<' num2str(p_l, '%3.2f') ')']);
xlabel('Run');
ylabel('[]');
