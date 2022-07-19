clearvars; clc;

subject = 'PERFECT';

rootpath    = 'analysis/simulation/';
figpath     = 'figures/simulation/';

ControlModalities = [1 2];
nmodalities = length(ControlModalities);

%% Loading results

util_bdisp(['[io] - Importing navigation results from ' rootpath ' for subject: ' subject]);
cdata = load([rootpath '/' subject '_navigation_results.mat']);

performances = cdata.performances;
durations    = cdata.durations;
commands     = cdata.commands;
frechet      = cdata.frechet;

nwaypoints = size(performances, 1);
nruns      = size(performances, 2);
nsubjects  = 1;

Ck = cdata.RunControlK;
Sk = ones(nruns, 1);

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
subplot(nrows, ncols, 1);
bar(squeeze(wpperformances));
grid on;
ylim([0 110]);
title('performances with/without sharedcontrol');
ylabel('[%]');
set(gca, 'XTickLabel', {'wp1', 'wp2', 'wp3', 'wp4'});

% Durations
subplot(nrows, ncols, 3);
boxplot(rdurations/512, {rWk rCk}, 'factorseparator', 1, 'labelverbosity', 'major', 'colorgroup', rCk);
grid on;
ylabel('[s]');
title('durations with/without sharedcontrol');

% Commands
subplot(nrows, ncols, 5);
boxplot(rcommands, {rWk rCk}, 'factorseparator', 1, 'labelverbosity', 'major', 'colorgroup', rCk);
grid on;
ylabel('[#]');
title('commands with/without sharedcontrol');

% Frechet
subplot(nrows, ncols, 7);
boxplot(rfrechet, {rWk rCk}, 'factorseparator', 1, 'labelverbosity', 'minor', 'colorgroup', rCk);
grid on;
ylabel('[m]');
title('frechet with/without sharedcontrol');

% Average performances per waypoint
subplot(nrows, ncols, 2);
hold on;
bar([squeeze(nanmean(wpperformances, 1)) squeeze(runperformances(:, :, 1))]) ;
offset = [-0.15 0.15];
for cId = 1:nmodalities
    errorbar(2 + offset(cId), squeeze(runperformances(cId, :, 1)), [], squeeze(runperformances(cId, :, 2)), '.k');
end
hold off;
ylim([0 130]);
ylabel('[%]');
title('average performances per waypoint and run');
legend('with SC', 'without SC', 'location', 'best');
grid on;
set(gca, 'XTick', [1 2]);
set(gca, 'XTickLabel', {'waypoint', 'run'});

% Average durations per waypoint
subplot(nrows, ncols, 4);
hold on;
bar([squeeze(nanmean(wpdurations, 1)) squeeze(rundurations(:, :, 1))]') ;
offset = [-0.15 0.15];
for cId = 1:nmodalities
    errorbar(2 + offset(cId), squeeze(rundurations(cId, :, 1)), [], squeeze(rundurations(cId, :, 2)), '.k');
end
hold off;
ylabel('[s]');
ylim([0 220]);
title('average durations per waypoint and run');
grid on;
set(gca, 'XTick', [1 2]);
set(gca, 'XTickLabel', {'waypoint', 'run'});

% Average commands per waypoint
subplot(nrows, ncols, 6);
hold on;
bar([squeeze(nanmean(wpcommands, 1)) squeeze(runcommands(:, :, 1))]') ;
offset = [-0.15 0.15];
for cId = 1:nmodalities
    errorbar(2 + offset(cId), squeeze(runcommands(cId, :, 1)), [], squeeze(runcommands(cId, :, 2)), '.k');
end
hold off;
ylabel('[#]');
ylim([0 40]);
title('average commands per waypoint and run');
grid on;
set(gca, 'XTick', [1 2]);
set(gca, 'XTickLabel', {'waypoint', 'run'});

% Average frechet per waypoint
subplot(nrows, ncols, 8);
hold on;
bar([squeeze(nanmean(wpfrechet, 1)) squeeze(runfrechet(:, :, 1))]') ;
offset = [-0.15 0.15];
for cId = 1:nmodalities
    errorbar(2 + offset(cId), squeeze(runfrechet(cId, :, 1)), [], squeeze(runfrechet(cId, :, 2)), '.k');
end
hold off;
ylabel('[m]');
ylim([0 1.5]);
title('average frechet per waypoint and run');
grid on;
set(gca, 'XTick', [1 2]);
set(gca, 'XTickLabel', {'waypoint', 'run'});

sgtitle('results PERFECT')


figname1 = fullfile(figpath, 'simulation_results_perfect.pdf');
util_disp(['[out] - Saving figure in: ' figname1]);
fig_export(fig1, figname1, '-pdf', 'landscape');

%% Functions
function rdata = reshape_data(data)
    nwaypoints = size(data, 1);
    nruns      = size(data, 2);

    rdata = reshape(data, nruns*nwaypoints, 1);
end

function rk = reshape_label(nwp, nruns, nsubjects, k)
    rk = reshape(repmat(k', nwp, 1), nruns*nwp*nsubjects, 1);
end
