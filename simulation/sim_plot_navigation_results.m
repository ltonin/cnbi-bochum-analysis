clearvars; clc;

sublist = {'BOCH02', 'BOCH04', 'BOCH05'};
nsubjects = length(sublist);

rootpath    = 'analysis/simulation/';
figpath     = 'figures/simulation/';

ControlModalities = [1 2];
nmodalities = length(ControlModalities);

%% Loading results
performances = [];
durations    = [];
commands     = [];
frechet      = [];
Sk = [];
Ck = [];
Wk = [];
nruns = [];
for sId = 1:nsubjects
    csubject = sublist{sId};
    util_bdisp(['[io] - Importing navigation results from ' rootpath ' for subject: ' csubject]);
    cdata = load([rootpath '/' csubject '_navigation_results.mat']);

    cnruns = size(cdata.performances, 2);
    cwaypoints = size(cdata.performances, 1);

    performances = cat(2, performances, cdata.performances);
    durations    = cat(2, durations, cdata.durations);
    commands     = cat(2, commands, cdata.commands);
    frechet      = cat(2, frechet, cdata.frechet);
    
    Ck = cat(1, Ck, cdata.RunControlK);
    Sk = cat(1, Sk, sId*ones(cnruns, 1));

    if isempty(nruns)
        nruns = cnruns;
    end

    if isequal(nruns, cnruns) == false
        warning('warn:runs', 'Different number of runs between subject');
    end
end

nwaypoints = size(performances, 1);

%% Reshaping performances, durations, commands, frechet

rperformances = reshape_data(performances);
rdurations    = reshape_data(durations);
rcommands     = reshape_data(commands);
rfrechet      = reshape_data(frechet);
rSk = reshape_label(nwaypoints, nruns, nsubjects, Sk);
rCk = reshape_label(nwaypoints, nruns, nsubjects, Ck);
rWk = repmat([1; 2; 3; 4], nruns*nsubjects, 1);

%% Computing average results

% Performances per waypoint
wpperformances = nan(nwaypoints, nsubjects, nmodalities);
for cId = 1:nmodalities
    for sId = 1:nsubjects
        cindex = Ck == cId & Sk == sId;
        wpperformances(:, sId, cId) = 100*nanmean(performances(:, cindex), 2);
    end
end

% Performances per run (mean, std)
runperformances = nan(nmodalities, nsubjects, 2);
for sId = 1:nsubjects
    for cId = 1:nmodalities
        cindex = Ck == cId & Sk == sId;
        runperformances(cId, sId, 1) = 100*nanmean(nanmean(performances(:, cindex), 1));
        runperformances(cId, sId, 2) = 100*nanstd(nanmean(performances(:, cindex), 1));
    end
end

% Durations per waypoint
wpdurations = nan(nwaypoints, nsubjects, nmodalities);
for cId = 1:nmodalities
    for sId = 1:nsubjects
        cindex = Ck == cId & Sk == sId;
        wpdurations(:, sId, cId) = nanmean(durations(:, cindex)/512, 2);
    end
end

% Durations per run (mean, std)
rundurations = nan(nmodalities, nsubjects, 2);
for sId = 1:nsubjects
    for cId = 1:nmodalities
        cindex = Ck == cId & Sk == sId;
        rundurations(cId, sId, 1) = nanmean(sum(durations(:, cindex)))/512;
        rundurations(cId, sId, 2) = nanstd(sum(durations(:, cindex)))/512;
    end
end

% Commands per waypoint
wpcommands = nan(nwaypoints, nsubjects, nmodalities);
for cId = 1:nmodalities
    for sId = 1:nsubjects
        cindex = Ck == cId & Sk == sId;
        wpcommands(:, sId, cId) = nanmean(commands(:, cindex), 2);
    end
end

% Commands per run (mean and std)
runcommands = nan(nmodalities, nsubjects, 2);
for sId = 1:nsubjects
    for cId = 1:nmodalities
        cindex = Ck == cId & Sk == sId;
        runcommands(cId, sId, 1) = nanmean(sum(commands(:, cindex)));
        runcommands(cId, sId, 2) = nanstd(sum(commands(:, cindex)));
    end
end

% Frechet per waypoint
wpfrechet = nan(nwaypoints, nsubjects, nmodalities);
for cId = 1:nmodalities
    for sId = 1:nsubjects
        cindex = Ck == cId & Sk == sId;
        wpfrechet(:, sId, cId) = nanmean(frechet(:, cindex), 2);
    end
end

% Frechet per run (mean and std)
runfrechet = nan(nmodalities, nsubjects, 2);
for sId = 1:nsubjects
    for cId = 1:nmodalities
        cindex = Ck == cId & Sk == sId;
        runfrechet(cId, sId, 1) = nanmean(sum(frechet(:, cindex)));
        runfrechet(cId, sId, 2) = nanstd(sum(frechet(:, cindex)));
    end
end

%% Figures

% Distributions per waypoint
fig1 = figure;
fig_set_position(fig1, 'All');

ncols = 2;
nrows = 4;

ControlLabels = {'with sharedcontrol', 'without sharedcontrol'};

% Performances
for cId = 1:nmodalities
    subplot(nrows, ncols, 1 + (cId-1));
    bar(wpperformances(:, :, cId));
    grid on;
    ylim([0 110]);
    title(['performances ' ControlLabels{cId}]);
    ylabel('[%]');
    set(gca, 'XTickLabel', {'wp1', 'wp2', 'wp3', 'wp4'});
end

% Durations
for cId = 1:nmodalities
    subplot(nrows, ncols, 3 + (cId-1));
    cindex = rCk == cId;
    boxplot(rdurations(cindex)/512, {rWk(cindex) rSk(cindex)}, 'factorseparator', 1, 'labelverbosity', 'major', 'colorgroup', rSk(cindex));
    grid on;
    ylabel('[s]');
    title(['durations ' ControlLabels{cId}]);
    ylim([30 110]);
end

% Commands
for cId = 1:nmodalities
    subplot(nrows, ncols, 5 + (cId-1));
    cindex = rCk == cId;
    boxplot(rcommands(cindex), {rWk(cindex) rSk(cindex)}, 'factorseparator', 1, 'labelverbosity', 'major', 'colorgroup', rSk(cindex));
    grid on;
    ylabel('[#]');
    title(['commands ' ControlLabels{cId}]);
    ylim([0 20]);
end

% Frechet
for cId = 1:nmodalities
    subplot(nrows, ncols, 7 + (cId-1));
    cindex = rCk == cId;
    boxplot(rfrechet(cindex), {rWk(cindex) rSk(cindex)}, 'factorseparator', 1, 'labelverbosity', 'minor', 'colorgroup', rSk(cindex));
    grid on;
    ylabel('[m]');
    title(['frechet ' ControlLabels{cId}]);
    ylim([0 2.0]);
end

sgtitle('distribution results pilots');

%% Average results per waypoint and per run
fig2 = figure;
fig_set_position(fig2, 'All');

ncols = 2;
nrows = 4;

% Average performances per waypoint
subplot(nrows, ncols, 1);
bar(squeeze(nanmean(wpperformances, 1)));
ylim([0 130]);
ylabel('[%]');
title('average performances per waypoint');
legend('with SC', 'without SC', 'location', 'best');
set(gca, 'XTick', [1 2 3]);
set(gca, 'XTickLabel', sublist);
grid on;

% Average performances per run
subplot(nrows, ncols, 2);
hold on;
bar(runperformances(:, :, 1)');
offset = [-0.15 0.15];
for cId = 1:nmodalities
    errorbar([1 2 3] + offset(cId), squeeze(runperformances(cId, :, 1)), [], squeeze(runperformances(cId, :, 2)), '.k');
end
hold off;
ylim([0 130]);
ylabel('[%]');
title('average performances per run');
set(gca, 'XTick', [1 2 3]);
set(gca, 'XTickLabel', sublist);
grid on;

% Average durations per waypoint
subplot(nrows, ncols, 3);
bar(squeeze(nanmean(wpdurations, 1)));
ylabel('[s]');
title('average durations per waypoint');
set(gca, 'XTick', [1 2 3]);
set(gca, 'XTickLabel', sublist);
grid on;

% Durations per run
subplot(nrows, ncols, 4);
hold on;
bar(rundurations(:, :, 1)');
offset = [-0.15 0.15];
for cId = 1:nmodalities
    errorbar([1 2 3] + offset(cId), squeeze(rundurations(cId, :, 1)), [], squeeze(rundurations(cId, :, 2)), '.k');
end
hold off;
ylabel('[s]');
title('average durations per run');
set(gca, 'XTick', [1 2 3]);
set(gca, 'XTickLabel', sublist);
grid on;

% Average commands per waypoint
subplot(nrows, ncols, 5);
bar(squeeze(nanmean(wpcommands, 1)));
ylabel('[#]');
ylim([0 15]);
title('average commands per waypoint');
set(gca, 'XTick', [1 2 3]);
set(gca, 'XTickLabel', sublist);
grid on;

% Commands per run
subplot(nrows, ncols, 6);
hold on;
bar(runcommands(:, :, 1)');
offset = [-0.15 0.15];
for cId = 1:nmodalities
    errorbar([1 2 3] + offset(cId), squeeze(runcommands(cId, :, 1)), [], squeeze(runcommands(cId, :, 2)), '.k');
end
hold off;
ylabel('[#]');
title('average commands per run');
set(gca, 'XTick', [1 2 3]);
set(gca, 'XTickLabel', sublist);
grid on;

% Average frechet per waypoint
subplot(nrows, ncols, 7);
bar(squeeze(nanmean(wpfrechet, 1)));
ylabel('[m]');
title('average frechet per waypoint');
set(gca, 'XTick', [1 2 3]);
set(gca, 'XTickLabel', sublist);
grid on;


% Frechet per run
subplot(nrows, ncols, 8);
hold on;
bar(runfrechet(:, :, 1)');
offset = [-0.15 0.15];
for cId = 1:nmodalities
    errorbar([1 2 3] + offset(cId), squeeze(runfrechet(cId, :, 1)), [], squeeze(runfrechet(cId, :, 2)), '.k');
end
hold off;
ylabel('[m]');
title('average frechet per run');
set(gca, 'XTick', [1 2 3]);
set(gca, 'XTickLabel', sublist);
grid on;
ylim([0 4.5]);

sgtitle('average results pilots');

figname1 = fullfile(figpath, 'simulation_results_distributions.pdf');
util_disp(['[out] - Saving figure in: ' figname1]);
fig_export(fig1, figname1, '-pdf', 'landscape');

figname2 = fullfile(figpath, 'simulation_results_average.pdf');
util_disp(['[out] - Saving figure in: ' figname2]);
fig_export(fig2, figname2, '-pdf', 'landscape');

%% Functions
function rdata = reshape_data(data)
    nwaypoints = size(data, 1);
    nruns      = size(data, 2);

    rdata = reshape(data, nruns*nwaypoints, 1);
end

function rk = reshape_label(nwp, nruns, nsubjects, k)
    rk = reshape(repmat(k', nwp, 1), nruns*nwp*nsubjects, 1);
end
