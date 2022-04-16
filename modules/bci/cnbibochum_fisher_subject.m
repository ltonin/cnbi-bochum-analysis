clearvars; clc;

subject = 'BOCH05';

includepat  = {subject, 'mi', 'mi_bhbf', 'online'};
excludepat  = {'control', 'offline'};
spatialfilter = 'laplacian';
artifactrej   = 'none'; % {'FORCe', 'none'}
datapath    = ['analysis/' artifactrej '/psd/' spatialfilter '/'];
savedir     = ['analysis/' artifactrej '/discriminancy/' spatialfilter '/'];
figdir      = ['figures/' artifactrej '/discriminancy/' spatialfilter '/'];

ClassEventId     = [773 771];
CFeedbackEventId = 781;
files = util_getfile3(datapath, '.mat', 'include', includepat, 'exclude', excludepat);
nfiles = length(files);
util_bdisp(['[io] - Found ' num2str(nfiles) ' files with the inclusion/exclusion criteria: (' strjoin(includepat, ', ') ') / (' strjoin(excludepat, ', ') ')']);

% Create analysis directory
util_mkdir('./', savedir);

%% Concatenate data
util_disp(['[io] - Importing ' num2str(nfiles) ' files from ' datapath ':'], 'b');
dayId = 0;
weekId = 0;
monthId = 0;
lastdate = '';
lastclassifier = '';
fisher = [];
Mk = zeros(nfiles, 1);
Fk = zeros(nfiles, 1);
Pk = zeros(nfiles, 1);
Dk = zeros(nfiles, 1);
Wk = zeros(nfiles, 1);
Nk = zeros(nfiles, 1);
CsK = zeros(nfiles, 1);
Dl = cell(nfiles, 1);
classifiers = cell(nfiles, 1);
freqgrid = cell(nfiles, 1);
for fId = 1:nfiles
    [cpath, cfilename, cext] = fileparts(files{fId});
    util_disp(['[io] - File ' num2str(fId) '/' num2str(nfiles) ': ' cfilename cext]);
    cdata = load(fullfile(cpath, [cfilename cext]));

    cF = log(cdata.psd);
    nsamples = size(cF,1);
    events = cdata.events;
    
    util_disp('[proc] - Extracting information');
    [cModality, cFeedback, cProtocol] = get_info_run(cdata.settings);
    dayId   = get_info_day(cdata.settings.date, lastdate, dayId);
    weekId  = get_info_week(cdata.settings.date, lastdate, weekId);
    monthId = get_info_month(cdata.settings.date, lastdate, monthId);
    newclassifier = get_info_classifier(cdata.classifier.filename, lastclassifier);
    classifiers{fId} = cdata.classifier;
    freqgrid{fId} = cdata.freqs;

    % Get events
    [~, TrialEvent] = proc_get_event2(CFeedbackEventId, nsamples, events.POS, events.TYP, events.DUR);
    [~, ClassEvent] = proc_get_event2(ClassEventId, nsamples, events.POS, events.TYP, events.DUR);
    
    Ck = zeros(nsamples, 1);
    ntrials = length(TrialEvent.TYP);
    for trId = 1:ntrials
        cstart = TrialEvent.POS(trId);
        cstop  = cstart + TrialEvent.DUR(trId) - 1;
        cclass = ClassEvent.TYP(trId);
        Ck(cstart:cstop) = cclass;
    end

    % Compute Fisher score
    util_disp('[proc] - Computing fisher score');
    fisher = cat(2, fisher, proc_fisher2(cF(Ck > 0, :, :), Ck(Ck > 0)));
    
    % Store labels
    Mk(fId) = cModality;
    Fk(fId) = cFeedback;
    Pk(fId) = cProtocol;
    Dk(fId) = dayId;
    Wk(fId) = weekId;
    Nk(fId) = monthId;
    Dl{fId} = cdata.settings.date;
    CsK(fId) = newclassifier;

    
    lastdate = cdata.settings.date;
    lastclassifier = cdata.classifier.filename;
    
end

%% Computing Medial and lateral fisher evolution
days = unique(Dk);
ndays = length(days);

SrcChans   = cdata.settings.data.lchannels;
SrcFreqs   = cdata.settings.spectrogram.freqgrid;
NumSrcChan = length(SrcChans);
NumSrcFreq = length(SrcFreqs);

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

roifisher = nan(2, ndays);
for dId = 1:ndays
    index = Dk == days(dId);
    
    cfisher = mean(fisher(:, index), 2);
    
    rfisher = reshape(cfisher, NumSrcFreq, NumSrcChan);
    rfisher_m = nanmean(nanmean(rfisher(RangeFreqIds, MedChanIds)));
    rfisher_l = nanmean(nanmean(rfisher(RangeFreqIds, LatChanIds)));
    
    roifisher(:, dId) = [rfisher_m rfisher_l];
end

[roicorr(1), roipval(1)] = corr((1:ndays)', roifisher(1, :)');
[roicorr(2), roipval(2)] = corr((1:ndays)', roifisher(2, :)');

%% Computing Selected features fisher evolution
nruns = size(fisher, 2);
runselfisher = nan(nruns, 1);
selfisher = nan(ndays, 1);
for rId = 1:nruns
    cfeatures = proc_cnbifeature2bin(classifiers{rId}.features, freqgrid{rId});
    runselfisher(rId) = mean(fisher(cfeatures, rId), 1);
end

for dId = 1:ndays
    index = Dk == days(dId);
    selfisher(dId) = mean(runselfisher(index));
end

[selcorr, selpval] = corr((1:ndays)', selfisher);

%% Computing new classifier
NCs = false(ndays, 1);
for dId = 1:ndays
    index = Dk == days(dId);
    if(sum(CsK(index)) > 0)
        NCs(dId) = true;
    end
end

%% Plotting evolution
fig1 = figure;
fig_set_position(fig1, 'Top');

subplot(1, 2, 1);
hold on;
h1 = scatter(1:ndays, roifisher', 'filled');
h2 = plot(1:ndays, roifisher');

h3 = lsline;

hold off;
for pId = 1:2
    set(h2(pId), 'Color', get(h1(pId),'CData'));
    set(h3(pId), 'Color', get(h1(pId),'CData'));
    text(min(h1(pId).XData), max(max(h1.YData))-0.05*(pId-1), ['r=' num2str(roicorr(pId), '%4.2f') ', p=' num2str(roipval(pId), '%4.3f')], 'color', get(h1(pId),'CData'));
end
grid on;
set(gca, 'xtick', 1:ndays);
set(gca,'xticklabels', unique(Dl));
xlim([0 ndays+1]);
ylimit = get(gca, 'YLim');
ylim([0 ylimit(2)])
ylim([0.05 0.55])
plot_vline(find(NCs)-0.1, 'k--');
title([subject  ' | Evolution discriminancy in ROIs']);
legend('medial', 'lateral');
ylabel('discriminancy')
xlabel('session')

subplot(1, 2, 2);
hold on;
h4 = scatter(1:ndays, selfisher, 'filled', 'k');
h5 = plot(1:ndays, selfisher);
h6 = lsline;

hold off;
set(h5, 'Color', get(h4,'CData'));
set(h6, 'Color', get(h4,'CData'));
text(min(h4.XData), max(h4.YData), ['r=' num2str(selcorr, '%4.2f') ', p=' num2str(selpval, '%4.3f')], 'color', get(h4,'CData'));
grid on;
set(gca, 'xtick', 1:ndays);
set(gca,'xticklabels', unique(Dl));
xlim([0 ndays+1]);
ylimit = get(gca, 'YLim');
ylim([0 ylimit(2)])
ylim([0.0 0.5])
plot_vline(find(NCs)-0.1, 'k--');
title([subject ' | Evolution discriminancy for selected features']);
ylabel('discriminancy')
xlabel('session')

%% Plotting topoplots
fig2 = figure;
fig_set_position(fig2, 'Top');
load('antneuro32.mat');

nselruns = 10;
cfisher_r = reshape(fisher, NumSrcFreq, NumSrcChan, nruns);
cfisher_pre = squeeze(nanmean(nanmean(cfisher_r(RangeFreqIds, :, 1:nselruns), 1), 3));
cfisher_post = squeeze(nanmean(nanmean(cfisher_r(RangeFreqIds, :, end-nselruns:end), 1), 3));
zfisher_pre = min(cfisher_pre)*ones(size(cfisher_pre));
zfisher_pre([LatChanIds; MedChanIds]) = cfisher_pre([LatChanIds; MedChanIds]);
zfisher_post = min(cfisher_post)*ones(size(cfisher_post));
zfisher_post([LatChanIds; MedChanIds]) = cfisher_post([LatChanIds; MedChanIds]);
%topoplot(zfisher, chanlocs, 'maplimits', [min(cfisher)-0.1 max(cfisher)+0.1], 'intsquare', 'off', 'conv', 'on', 'shading', 'interp');
subplot(1, 2, 1);
topoplot(zfisher_pre, chanlocs, 'maplimits', [min(zfisher_pre)-0.1 max(zfisher_pre)+0.1], 'intsquare', 'off', 'conv', 'on', 'shading', 'interp');
axis image
title([subject ' | First ' num2str(nselruns) ' runs']);

subplot(1, 2, 2);
topoplot(zfisher_post, chanlocs, 'maplimits', [min(zfisher_post)-0.1 max(zfisher_post)+0.1], 'intsquare', 'off', 'conv', 'on', 'shading', 'interp');
axis image
title([subject ' | Last ' num2str(nselruns) ' runs']);

%% Exporting figure
filename1 = fullfile(figdir, [subject '_discriminancy_evolution_day.pdf']);
util_disp(['[out] - Exporting discriminancy maps per week to ' filename1], 'b');
fig_export(fig1, filename1, '-pdf');

filename2 = fullfile(figdir, [subject '_discriminancy_topoplot_firstlast_runs.pdf']);
util_disp(['[out] - Exporting discriminancy topoplots to ' filename2], 'b');
fig_export(fig2, filename2, '-pdf');

%% Saving
labels.Mk = Mk;
labels.Fk = Fk;
labels.Pk = Pk;
labels.Dk = Dk;
labels.Wk = Wk;
labels.Nk = Nk;
labels.Dl = Dl;
labels.Csk = CsK;
labels.NCs = NCs;

savepath = [savedir '/' subject '_fisher2_' spatialfilter '.mat'];
util_bdisp(['[out] - Saving bci online fisher in ' savepath]);
save(savepath, 'fisher', 'labels', 'classifiers', 'freqgrid');

%% Functions
function [modality, feedback, protocol] = get_info_run(settings)

    if(strcmpi(settings.modality.name, 'unknown'))
        error(['Unknown modality type: ' settings.modality.name]);
    else
        [~, modality] = intersect(settings.modality.legend, settings.modality.name);
    end

    % Get run feebdack
    if(strcmpi(settings.feedback.name, 'unknown'))
        error(['Unknown feedback type: ' settings.feedback.name]);
    else
        [~, feedback] = intersect(settings.feedback.legend, settings.feedback.name);
    end
        
    % Get run protocol
    if(strcmpi(settings.protocol.name, 'unknown'))
        error(['Unknown protocol type: ' settings.protocol.name]);
    else
        [~, protocol] = intersect(settings.protocol.legend, settings.protocol.name);
    end

end

function dayId = get_info_day(currdate, lastdate, dayId)
    % Get day id
    cday = day(datetime(currdate, 'InputFormat', 'yyyyMMdd'));
    lday = day(datetime(lastdate, 'InputFormat', 'yyyyMMdd'));
    if isequal(cday, lday) == false
        dayId = dayId +1;
    end
end

function weekId = get_info_week(currdate, lastdate, weekId)
    
    % Get week id
    cweek = week(datetime(currdate, 'InputFormat', 'yyyyMMdd'));
    lweek = week(datetime(lastdate, 'InputFormat', 'yyyyMMdd'));
    if isequal(cweek, lweek) == false
        weekId = weekId +1;
    end
end

function monthId = get_info_month(currdate, lastdate, monthId)
    
    % Get month id
    cmonth = month(datetime(currdate, 'InputFormat', 'yyyyMMdd'));
    lmonth = month(datetime(lastdate, 'InputFormat', 'yyyyMMdd'));
    if isequal(cmonth, lmonth) == false
        monthId = monthId +1;
    end

end

function newclassifier = get_info_classifier(currclassifier, lastclassifier)
    newclassifier = false;
    if isequal(currclassifier, lastclassifier) == false
        newclassifier = true;
    end
end



