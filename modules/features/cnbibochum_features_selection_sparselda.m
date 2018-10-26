clearvars; clc; close all;

subject = 'aj1';

pattern   = [subject '*.offline.mi.mi_bhbf'];
spatialfilter = 'laplacian';
datapath    = ['analysis/psd/' spatialfilter '/'];

ClassEventId     = [773 771];
CFeedbackEventId = 781;
NumClasses = length(ClassEventId);
files = util_getfile(datapath, '.mat', pattern);
nfiles = length(files);

NumSelectedFeatures = 6;

%% Concatenate data
util_bdisp(['[io] - Importing ' num2str(nfiles) ' files from ' datapath ':']);
[F, events, labels, classifiers, settings] = cnbibochum_concatenate_data(files);

ChannelLabels = settings.data.lchannels;
FreqGrid = settings.spectrogram.freqgrid;

U = log(F);
% U = F;
NumSamples = size(U, 1);
NumChans = size(U, 3);
NumFreqs = size(U, 2);

%% Get general events
util_bdisp('[proc] - Extract events');
[~, CFbEvents] = proc_get_event2(CFeedbackEventId, NumSamples, events.POS, events.TYP, events.DUR);
[~, CueEvents] = proc_get_event2(ClassEventId, NumSamples, events.POS, events.TYP, events.DUR);

NumTrials = length(CFbEvents.TYP);

Ck = zeros(NumSamples, 1);
Rk = zeros(NumSamples, 1);
Dk = zeros(NumSamples, 1);

for trId = 1:NumTrials
    cstart = CFbEvents.POS(trId);
    cstop  = cstart + CFbEvents.DUR(trId) - 1;
    cclass = CueEvents.TYP(trId);
    Ck(cstart:cstop) = cclass;
    Rk(cstart:cstop) = unique(labels.Rk(cstart:cstop));
    Dk(cstart:cstop) = unique(labels.Dk(cstart:cstop));
end

%% Computing fisher score (all data together)
util_bdisp('[proc] - Computing fisher score (all run together)');
X = proc_reshape_ts_bc(U(Ck > 0, :, :));
X = normalize(X);
X = reshape(X, size(X, 1), NumFreqs, NumChans);
FisherScoreAll = proc_fisher2(X, Ck(Ck > 0));

[~, fs_FeatureIds] = sort(FisherScoreAll, 'descend');
fs_SelFeatureIds = fs_FeatureIds(1:NumSelectedFeatures);

%% Feature selection with sparse lda
util_bdisp('[proc] - Feature selection via sparse lda');
delta = 1e-3;                       % regularization parameter
stop = -NumSelectedFeatures;        %this says you want more or less 6 features (yes, you need the negative sign)
maxiter = 250;                      % maximum number of iterations
Q = 1;                              % request two discriminative directions
convergenceCriterion = 1e-6; 

X  = proc_reshape_ts_bc(U(Ck > 0, :, :));
% Xn = -1 + 2.*(X - min(X))./(max(X) - min(X));
[Xn, mu, d] = normalize(X);
%Xn = (Xn - mu*ones(size(X, 1), 1))./sqrt(d*ones(size(X, 1), 1));
Xk = Ck(Ck > 0);

Yk = zeros(size(Xn, 1), NumClasses);
for cId = 1:NumClasses
    Yk(Xk == ClassEventId(cId), cId) = 1;
end

[model.B, theta] = slda(Xn, Yk, delta, stop, Q, maxiter, convergenceCriterion, false); 

Weights = model.B;
slda_SelFeatureIds = find(Weights);



%% Convert features ids to channels and frequencies

[slda_FreqIds, slda_ChanIds] = proc_bin2bc([NumFreqs NumChans], slda_SelFeatureIds);
slda_SelFreqs = FreqGrid(slda_FreqIds);
slda_SelChans = ChannelLabels(slda_ChanIds);

[fs_FreqIds, fs_ChanIds] = proc_bin2bc([NumFreqs NumChans], fs_SelFeatureIds);
fs_SelFreqs = FreqGrid(fs_FreqIds);
fs_SelChans = ChannelLabels(fs_ChanIds);

%% Plot

fig1 = figure;
fig_set_position(fig1, 'Top');


imagesc(reshape(FisherScoreAll, [NumFreqs NumChans])');
hold on;
plot(slda_FreqIds, slda_ChanIds, 'vr', 'MarkerSize', 10, 'LineWidth', 3);
plot(fs_FreqIds, fs_ChanIds, '^g', 'MarkerSize', 10, 'LineWidth', 3);
hold off;

set(gca, 'XTick', 1:NumFreqs);
set(gca, 'XTickLabel', FreqGrid);
set(gca, 'YTick', 1:NumChans);
set(gca, 'YTickLabel', ChannelLabels);

xlabel('[Hz]');
ylabel('Channel');
legend('slda selected features', 'fisher score selected features');

title([subject ' - Fisher score map and selected features'])


