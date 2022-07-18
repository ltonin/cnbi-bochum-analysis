clearvars; clc;

subject = 'PERFECT';

includepat  = {subject, 'simulation', 'sharedcontrol'};
excludepat  = {};
depthlevel  = 3;

rootpath    = 'analysis/simulation/';
savedir     = 'analysis/simulation/';
figpath     = 'figures/simulation/';
recompute   = true;

ControlModality = [1 2];
ControlModalityLabel = {'enabled', 'disabled'};
nmodalities = length(ControlModality);

WpPosition(:, 1) = [-0.771 -1.479 1.570];
WpPosition(:, 2) = [ 0.256  2.733 3.141];
WpPosition(:, 3) = [-0.771 -1.479 4.712];
WpPosition(:, 4) = [-2.012 -5.655 3.141];

WpLabels = {'WP1', 'WP2', 'WP3', 'WP4'};

nwaypoints = size(WpPosition, 2);

%% Get datafiles
files = util_getfile3(rootpath, '.mat', 'include', includepat, 'exclude', excludepat, 'level', depthlevel);

nfiles = length(files);
if(nfiles > 0)
    util_bdisp(['[io] - Found ' num2str(nfiles) ' files with the inclusion/exclusion criteria: (' strjoin(includepat, ', ') ') / (' strjoin(excludepat, ', ') '), depth: ' num2str(depthlevel)]);
else
    error(['[io] - No files found with the inclusion/exclusion criteria: (' strjoin(includepat, ', ') ') / (' strjoin(excludepat, ', ') '), depth: ' num2str(depthlevel)]);
end

nruns = nfiles;

%% Loading the file
poses  = cell(nruns, 1);
cmdvel = cell(nruns, 1);
cmdevt = cell(nruns, 1);
t      = cell(nruns, 1);
Ck     = zeros(nruns, 1);

for fId = 1:nfiles
    cfullname = files{fId};
    [cfilepath, cfilename, cfileext] = fileparts(cfullname);
    
    util_bdisp(['[io] + Loading file ' num2str(fId) '/' num2str(nfiles)]);
    disp(['     |-File: ' cfullname]);

    cdata = load(cfullname);

    info = sim_getfile_info(cfullname);

    if(strcmp(info.control, 'enabled'))
        Ck(fId) = ControlModality(1);
    elseif (strcmp(info.control, 'disabled'))
        Ck(fId) = ControlModality(2);
    end

    samplerate = cdata.SampleRate;
    cmdvel{fId} = cdata.cmdvel;
    cmdevt{fId} = cdata.events;
    poses{fId}  = cdata.pose;
    t{fId}      = cdata.t;
end

%% Computing waypoint reached
wpdistance = cell(nruns, 1); 
wpreached  = zeros(nwaypoints, nruns);

for rId = 1:nruns
    cx = poses{rId}(:, 1);
    cy = poses{rId}(:, 2);
    
    [distance, index] = sim_find_minimum_distance(cx, cy, WpPosition(1:2, :));
    
    wpdistance{rId}.distance = distance;
    wpdistance{rId}.index    = index;

    wpreached(:, rId) = isnan(index) == false;
end

%% Find starting point
startIdx = nan(nruns, 1);

for rId = 1:nruns
    currvel_x = cmdvel{rId}(:, 1);
    startIdx(rId) = sim_find_start_index(currvel_x, 0.15);
end

%% Computing run per modalities
runPermod = cell(nmodalities, 1);
nrunPermod = zeros(nmodalities, 1);

for mId = 1:nmodalities
    runPermod{mId}  = find(Ck == ControlModality(mId));
    nrunPermod(mId) = length(runPermod{mId});
end

%% Computing duration
durations = nan(nwaypoints, nruns);
for rId = 1:nruns

    cstart = startIdx(rId);
    cindex = [cstart; wpdistance{rId}.index];
    
    cduration = nan(nwaypoints, 1);

    for wId = 2:(nwaypoints+1)
        cduration(wId-1) = (cindex(wId) - cindex(wId-1))./samplerate;
    end
    durations(:, rId) = cduration;
end

WpDurations  = zeros(nmodalities, nwaypoints);
RunDurations = zeros(nmodalities, nrunPermod(1));   %% Carefull
AvgDurations = zeros(nmodalities, 1);
StdDurations = zeros(nmodalities, 1);

for mId = 1:nmodalities

    WpDurations(mId, :)  = nanmean(durations(:, Ck == ControlModality(mId)), 2);
    RunDurations(mId, :) = nanmean(durations(:, Ck == ControlModality(mId)), 1);
    AvgDurations(mId) = nanmean(RunDurations(mId, :));
    StdDurations(mId) = nanstd(RunDurations(mId, :));
end

StatModDurations = ranksum(RunDurations(1, :), RunDurations(2, :));


%% Figures

% Results
fig1 = figure;
fig_set_position(fig1, 'Top');

nrows = 1;
ncols = 3;

subplot(nrows, ncols, 1);
bar(WpDurations');
%ylim([0 110]);
grid on;
set(gca, 'XTickLabel', WpLabels);
xlabel('waypoint');
ylabel('[s]');
title('Duration per waypoint');
legend('SC enabled', 'SC disabled');

subplot(nrows, ncols, 2);
bar(RunDurations');
%ylim([0 110]);
grid on;
xlabel('run');
ylabel('[s]');
title('Duration per run');
legend('SC enabled', 'SC disabled');

subplot(nrows, ncols, 3);
hold on;
bar(AvgDurations');
errorbar(1:nmodalities, AvgDurations', [0 0], StdDurations', 'k.');
hold off;

set(gca, 'XTick', 1:nmodalities);
set(gca, 'XTickLabel', {'SC enabled', 'SC disabled'});
title(['Averaged waypoint duration p=' num2str(StatModDurations)] );
xlabel('modality');
ylabel('[s]');
grid on;

sgtitle([subject ' simulation']);

figname = fullfile(figpath, [subject '_simulation_durations.pdf']);
util_disp(['[out] - Saving figure in: ' figname]);
fig_export(fig1, figname, '-pdf', 'landscape');
