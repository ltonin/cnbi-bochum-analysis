clearvars; clc;

sublist = {'BOCH02', 'BOCH04', 'BOCH05'};
nsubjects = length(sublist);

rootpath    = 'analysis/simulation/';
figpath     = 'figures/simulation/';


%% Loading results

nwaypoints = 4;
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

    rWk = reshape(repmat([1; 2; 3; 4], cnruns, 1), cnruns*cwaypoints, 1);
    rSk = sId*ones(cnruns*cwaypoints, 1);
    rCk = [ones(40, 1); 2*ones(40, 1)];

    rperformances = reshape(cdata.performances, cnruns*cwaypoints, 1);
    rdurations    = reshape(cdata.durations, cnruns*cwaypoints, 1);
    rcommands     = reshape(cdata.commands, cnruns*cwaypoints, 1);
    rfrechet      = reshape(cdata.frechet, cnruns*cwaypoints, 1);

    performances = cat(1, performances, rperformances);
    durations    = cat(1, durations, rdurations);
    commands     = cat(1, commands, rcommands);
    frechet      = cat(1, frechet, rfrechet);
    
    Ck = cat(1, Ck, rCk);
    Sk = cat(1, Sk, rSk);
    Wk = cat(1, Wk, rWk);

    if isempty(nruns)
        nruns = cnruns;
    end

    if isequal(nruns, cnruns) == false
        warning('warn:runs', 'Different number of runs between subject');
    end
end


fig1 = figure;
fig_set_position(fig1, 'All');

ncols = 3;
nrows = 4;

ControlLabels = {'with sharedcontrol', 'without sharedcontrol'};

% % Performances
for cId = 1:2
    subplot(nrows, ncols, cId);
    avgperformances = nan(4, nsubjects);
    for sId = 1:nsubjects
        for wId = 1:nwaypoints
            cindex = Ck == cId & Sk == sId & Wk == wId;
            avgperformances(wId, sId) = 100*nanmean(performances(cindex), 1);
        end
    end
    bar(avgperformances);
    grid on;
    ylim([0 110]);
    title(['performances ' ControlLabels{cId}]);
    ylabel('[%]');
    set(gca, 'XTickLabel', {'wp1', 'wp2', 'wp3', 'wp4'});
end

avgrunperf = nan(2, nsubjects);
subplot(nrows, ncols, 3);
for sId = 1:nsubjects
    for cId = 1:2
        cindex = Ck == cId & Sk == sId;
        avgrunperf(cId, sId) = 100*nanmean(performances(cindex));
    end
end
bar(avgrunperf');
ylim([0 110]);
ylabel('[%]');
title('average performances');
legend('with SC', 'without SC');
grid on;

% Durations
for cId = 1:2
    subplot(nrows, ncols, 3 + cId);
    cindex = Ck == cId;
    boxplot(durations(cindex)/512, {Wk(cindex) Sk(cindex)}, 'factorseparator', 1, 'labelverbosity', 'major', 'colorgroup', Sk(cindex));
    grid on;
    ylabel('[s]');
    title(['durations ' ControlLabels{cId}]);
    ylim([30 110]);
end

% Commands
for cId = 1:2
    subplot(nrows, ncols, 6 + cId);
    cindex = Ck == cId;
    boxplot(commands(cindex), {Wk(cindex) Sk(cindex)}, 'factorseparator', 1, 'labelverbosity', 'major', 'colorgroup', Sk(cindex));
    grid on;
    ylabel('[#]');
    title(['commands ' ControlLabels{cId}]);
    ylim([0 20]);
end

% Frechet
for cId = 1:2
    subplot(nrows, ncols, 9 + cId);
    cindex = Ck == cId;
    boxplot(frechet(cindex), {Wk(cindex) Sk(cindex)}, 'factorseparator', 1, 'labelverbosity', 'minor', 'colorgroup', Sk(cindex));
    grid on;
    ylabel('[m]');
    title(['frechet ' ControlLabels{cId}]);
    ylim([0 2.0]);
end



