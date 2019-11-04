clearvars; clc; close all;

subject = 'BOCH01';

spatialfilter = 'laplacian';
artifactrej   = 'none'; % {'FORCe', 'none'}
pattern   = [subject '_fisher_' spatialfilter '.mat'];
datapath  = ['analysis/' artifactrej '/discriminancy/' spatialfilter '/'];

figdir = ['figures/' artifactrej '/discriminancy/' spatialfilter '/'];

% Create analysis directory
util_mkdir('./', figdir);

%% Loading fisher score data
util_bdisp(['[io] - Importing fisher score data for subject ' subject ' from ' datapath]);
data = load([datapath pattern]);

Months      = unique(data.labels.run.Nk);
NumMonths   = length(Months);
Weeks       = unique(data.labels.run.Wk);
NumWeeks    = length(Weeks);
NumRuns     = size(data.fisher.run, 2);

SrcChans   = data.settings.data.lchannels;
SrcFreqs   = data.settings.spectrogram.freqgrid;
NumSrcChan = length(SrcChans);
NumSrcFreq = length(SrcFreqs);

%% Define required frequencies and channels
ReqFreqs = 6:48;
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
ReqLatChans = {'FC3', 'FC1', 'FC2', 'FC4', ...
                'C3',  'C1',  'C2',  'C4',  ...
               'CP3', 'CP1', 'CP2', 'CP4'};

ReqMedChans = {'FCz', 'Cz', 'Pz'};

LatChanIds = proc_get_channel(ReqLatChans, SrcChans);
LatChans   = SrcChans(LatChanIds);
MedChanIds = proc_get_channel(ReqMedChans, SrcChans);
MedChans   = SrcChans(MedChanIds);

ReqRangeFreq  = [8:12 18:28];
[RangeFreqs, RangeFreqIds] = intersect(SrcFreqs, ReqRangeFreq);


%% Select runs to be removed
[Rmk, RemRuns] = cnbibochum_exclude_runs(subject, NumRuns);


%% Computing the sum of the fisher score for the selected features per run
sfisher = nan(NumRuns, 1);
for rId = 1:NumRuns
    if (data.labels.run.Mk(rId) == 1) || Rmk(rId) == true
        continue
    end
   cfisher = data.fisher.run(:, rId);
   
   cfeatures = data.classifiers(rId).features;
   
   if(isempty(cfeatures) == true)
       continue;
   end
   
   cfeatureIdx = proc_cnbifeature2bin(cfeatures, SrcFreqs);

          
   sfisher(rId) = nanmean(cfisher(cfeatureIdx));
end

%% Computing the sum of the fisher score for the medial and lateral channels (in alpha/beta)
mlfisher = nan(2, NumRuns);

for rId = 1:NumRuns
    if Rmk(rId) == true
        continue
    end
   cfisher = reshape(data.fisher.run(:, rId), NumSrcFreq, NumSrcChan);
   cfisher_m = nanmean(nanmean(cfisher(RangeFreqIds, MedChanIds)));
   cfisher_l = nanmean(nanmean(cfisher(RangeFreqIds, LatChanIds)));
   
   mlfisher(:, rId) = [cfisher_m cfisher_l];
end

%% Reshaping fisher scores and extract freqs and channels
rfisher = reshape(data.fisher.run, NumSrcFreq, NumSrcChan, NumRuns);
%rfisher = rfisher(SelFreqIds, SelChanIds, :);


%% Plot DP maps per week
fig1 = figure;
fig_set_position(fig1, 'All');


[NumRows, NumCols] = plot_subplot_size(NumWeeks, 'maxcols', 4);

for wId = 1:NumWeeks
    
    cindex = data.labels.run.Wk == wId & Rmk == false;
    
    if(sum(cindex) == 0)
        continue;
    end
    
    subplot(NumRows, NumCols, wId);
    cfisher = nanmean(rfisher(SelFreqIds, SelChanIds, cindex), 3);
    
    imagesc(cfisher');
    axis square
    set(gca, 'XTick', 1:NumSelFreqs);
    set(gca, 'XTickLabel', SelFreqs);
    set(gca, 'YTick', 1:NumSelChans);  
    set(gca, 'YTickLabel', SelChans);
    
    xlabel('[Hz]');
    ylabel('channel');
    
    title(['Week ' num2str(wId) ' (NRuns=' num2str(sum(cindex)) ')']);
end
sgtitle([subject '- DP maps per week']);


%% Plot topoplots maps per month
fig2 = figure;
fig_set_position(fig2, 'Top');


[NumRows, NumCols] = plot_subplot_size(NumMonths, 'maxcols', 4);

for nId = 1:NumMonths
    
    cindex = data.labels.run.Nk == nId & Rmk == false;
    
    if(sum(cindex) == 0)
        continue;
    end
    
    subplot(NumRows, NumCols, nId);
    load('antneuro32.mat');
    cfisher = squeeze(nanmean(nanmean(rfisher(RangeFreqIds, :, cindex), 1), 3));
    zfisher = min(cfisher)*ones(size(cfisher));
    zfisher([LatChanIds; MedChanIds]) = cfisher([LatChanIds; MedChanIds]);
    topoplot(zfisher, chanlocs, 'maplimits', [min(cfisher)-0.1 max(cfisher)+0.1], 'intsquare', 'off', 'conv', 'on', 'shading', 'interp');
    axis image
    
%     h = title(['Month ' num2str(nId) ' (NRuns=' num2str(sum(cindex)) ')']);
    h = text(0, -0.55, ['Month ' num2str(nId) ' (NRuns=' num2str(sum(cindex)) ')'], 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'FontSize', 10);

end
sgtitle([subject '- DP topoplots per month']);
    
%% Plot DP maps per month
fig3 = figure;
fig_set_position(fig3, 'Top');

[NumRows, NumCols] = plot_subplot_size(NumMonths, 'maxcols', 4);

for nId = 1:NumMonths
    
    cindex = data.labels.run.Nk == nId & Rmk == false;
    
    if(sum(cindex) == 0)
        continue;
    end
    
    subplot(NumRows, NumCols, nId);
    cfisher = nanmean(rfisher(SelFreqIds, SelChanIds, cindex), 3);
    
    imagesc(cfisher');
    axis square
    set(gca, 'XTick', 1:NumSelFreqs);
    set(gca, 'XTickLabel', SelFreqs);
    set(gca, 'YTick', 1:NumSelChans);
    set(gca, 'YTickLabel', SelChans);
    
    xlabel('[Hz]');
    ylabel('channel');
    
    title(['Month ' num2str(nId) ' (NRuns=' num2str(sum(cindex)) ')']);
end
sgtitle([subject '- DP maps per month']);

%% Plot Fisher score for selected featuers per run and for medial and lateral channels
fig4 = figure;
fig_set_position(fig4, 'Top');

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
ylim([0 0.5]);
xlim([1 NumRuns]);
plot_vline(ClIds, 'k--');
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
ylim([0 0.5]);
xlim([1 NumRuns]);
plot_vline(ClIds, 'k--');
title('Fisher score for medial/lateral channels'); 
grid on;
legend(['medial (corr=' num2str(c_m, '%3.2f') ', p<' num2str(p_m, '%3.2f') ')'], ['lateral (corr=' num2str(c_l, '%3.2f') ', p<' num2str(p_l, '%3.2f') ')']);
xlabel('Run');
ylabel('[]');


%% Saving figures
filename1 = fullfile(figdir, [subject '_discriminancy_map_week.pdf']);
filename2 = fullfile(figdir, [subject '_discriminancy_topoplot_month.pdf']);
filename3 = fullfile(figdir, [subject '_discriminancy_map_month.pdf']);
filename4 = fullfile(figdir, [subject '_discriminancy_evolution_run.pdf']);

util_bdisp(['[out] - Exporting discriminancy maps per week to ' filename1]);
fig_export(fig1, filename1, '-pdf');
util_bdisp(['[out] - Exporting discriminancy topoplots per month to ' filename2]);
fig_export(fig2, filename2, '-pdf', [], '-fillpage');
util_bdisp(['[out] - Exporting discriminancy maps per month to ' filename3]);
fig_export(fig3, filename3, '-pdf');
util_bdisp(['[out] - Exporting discriminancy evolution per run to ' filename4]);
fig_export(fig4, filename4, '-pdf');

