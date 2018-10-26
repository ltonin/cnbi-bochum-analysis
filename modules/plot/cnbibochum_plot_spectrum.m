clearvars; clc; close all;

subject = 'BOCH02';

pattern   = [subject '*.online.mi.mi_bhbf'];
spatialfilter = 'none';
datapath    = ['analysis/psd/' spatialfilter '/'];
savedir     = ['analysis/bci/' spatialfilter '/'];
figdir      = 'figures/';

Classes     = [773 771];
NumClasses  = length(Classes);
ClassNames  = {'both hands', 'both feet'};
CFeedbackEventId = 781;
FixationEventId = 786;
files = util_getfile(datapath, '.mat', pattern);
nfiles = length(files);
[montage, chanlabels] = proc_get_montage('eeg.antneuro.32.mi');

% Create analysis directory
util_mkdir('./', savedir);


%% Concatenate data
util_bdisp(['[io] - Importing ' num2str(nfiles) ' files for subject ' subject ' from ' datapath ':']);
[F, events, labels, classifiers, settings] = cnbibochum_concatenate_data(files);

U = log(F);
NumSamples = size(U, 1);
NumChans = size(U, 3);
NumFreqs = size(U, 2);
FreqGrid = settings.spectrogram.freqgrid;


%% Get general events
[TrialLabels, TrialEvents] = proc_get_event2(CFeedbackEventId, NumSamples, events.POS, events.TYP, events.DUR);
[~, ClassEvents] = proc_get_event2(Classes, NumSamples, events.POS, events.TYP, events.DUR);
[FixLabels, FixEvents] = proc_get_event2(FixationEventId, NumSamples, events.POS, events.TYP, events.DUR);

NumTrials = length(TrialEvents.TYP);

Ck = zeros(NumSamples, 1);
Rk = zeros(NumSamples, 1);
Dk = zeros(NumSamples, 1);
Wk = zeros(NumSamples, 1);
Pk = zeros(NumSamples, 1);

for trId = 1:NumTrials
    cstart = TrialEvents.POS(trId);
    cstop  = cstart + TrialEvents.DUR(trId) - 1;
    cclass = ClassEvents.TYP(trId);
    Ck(cstart:cstop) = cclass;
    Rk(cstart:cstop) = unique(labels.Rk(cstart:cstop));
    Dk(cstart:cstop) = unique(labels.Dk(cstart:cstop));
    Wk(cstart:cstop) = unique(labels.Wk(cstart:cstop));
    Pk(cstart:cstop) = unique(labels.Pk(cstart:cstop));
end



%% Plot
fig1 = figure;
fig_set_position(fig1, 'All');

ReqFreqs = 1:100;
[SelFreqs, SelFreqIds] = intersect(FreqGrid, ReqFreqs);

NumRows = 6;
NumCols = 7;

Index = labels.Dk >= 20;
h = [];
for chId = 1:NumChans
    plotIdx = find(montage' == chId);
    if(isempty(plotIdx))
        continue;
    end
    
    subplot(NumRows, NumCols, plotIdx);
    
    hold on;
    for cId = 1:NumClasses
        cindex = Ck == Classes(cId);
        ctspectrum = squeeze(mean(U(Index & cindex, SelFreqIds, chId), 1));
        plot(SelFreqs, ctspectrum, 'LineWidth', 2);
        h = cat(1, h, gca);
    end
    fixindex = FixLabels == FixationEventId;
    cfspectrum = squeeze(mean(U(Index & fixindex, SelFreqIds, chId), 1));
    plot(SelFreqs, cfspectrum, 'k--', 'LineWidth', 1);
    h = cat(1, h, gca);
    
    hold off;
    grid on;
    title(proc_get_channel(chId, chanlabels))
    
    if chId == proc_get_channel('POz', chanlabels)
        legend(ClassNames, 'fixation');
    end
    
end

plot_set_limits(h, 'y', 'minmax');
plot_set_limits(h, 'x', [SelFreqs(1) SelFreqs(end)]);

generaltitle = 'Last two weeks - log(PSD)';
suptitle([subject ' - ' generaltitle]);
