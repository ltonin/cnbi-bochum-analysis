clearvars; clc; close all;

sublist = {'BOCH02', 'BOCH04', 'BOCH05'};
savedir = 'analysis/parkour/';
WayPointLabels = {'WP1', 'WP2' 'WP3', 'WP4'};

NumSubjects = length(sublist);

figdir = ['figures/parkour/'];

% Create analysis directory
util_mkdir('./', figdir);

Wp = [];
Dk = [];
Sk = [];
Dl = [];

for sId = 1:NumSubjects
    csubject = sublist{sId};
    util_bdisp(['[io] - Importing parkour data for subject: ' csubject]);
    [cwp, cdays] = cnbibochum_get_parkour_data(csubject);
    
    ncwp = size(cwp, 2);
    
    Wp = cat(2, Wp, cwp);
    
    Sk = cat(1, Sk, sId*ones(ncwp, 1));
    Dk = cat(1, Dk, cdays.index');
    
    cDl = [];
    for dId = 1:length(cdays.index)
        cday = cdays.index(dId);
        clabel = cdays.label{cday};
        cDl = cat(1, cDl, clabel);
    end
    
    Dl = cat(1, Dl, cDl);
end


NumWayPoints = size(Wp, 1);
NumTotRuns   = size(Wp, 2);

% Run performances
RunPerf = nan(NumTotRuns, 1);
for rId = 1:NumTotRuns
    RunPerf(rId) = 100*sum(Wp(:, rId))./NumWayPoints;
end

% WayPoints performance
WayPointPerfSubj = nan(NumWayPoints, NumSubjects);

for wId = 1:NumWayPoints
    for sId = 1:NumSubjects
        
        WayPointPerfSubj(wId, sId) = 100*sum(Wp(wId, Sk == sId), 2)./sum(Sk == sId); 
    end
end

% Run performances per subject
SubjAcc = nan(NumSubjects, 1);
SubjStd = nan(NumSubjects, 1);
for sId = 1:NumSubjects
   cindex = Sk == sId;
   
   SubjAcc(sId) = mean(RunPerf(cindex));
   SubjStd(sId) = std(RunPerf(cindex));
end

% Waypoints performances
WayPointAcc = nan(NumWayPoints, 1);
WayPointStd = nan(NumWayPoints, 1);
for wId = 1:NumWayPoints
    
    WayPointAcc(wId) = mean(WayPointPerfSubj(wId, :));
    WayPointStd(wId) = std(WayPointPerfSubj(wId, :));
end

% Evolution performances
MaxNumRuns = 0;
for sId = 1:NumSubjects
    MaxNumRuns = max(MaxNumRuns, sum(Sk == sId));
end

PerfEvoSubj = nan(MaxNumRuns, NumSubjects);
for sId = 1:NumSubjects
    cindex = Sk == sId;
    cnruns = sum(cindex);
    
    PerfEvoSubj(1:cnruns, sId) = RunPerf(cindex);
end

%% Plotting

fig1 = figure;
fig_set_position(fig1, 'All');

NumRows = 2;
NumCols = 2;

subplot(NumRows, NumCols, 1);
h = plot_barerrors(SubjAcc, SubjStd);
grid on;
set(gca, 'XTick', 1:NumSubjects);
set(gca, 'XTickLabel', sublist);
xlabel('Subject');
ylabel('Percentage [%]');
title('Overall performances');

subplot(NumRows, NumCols, 2);
plot_barerrors(WayPointAcc, WayPointStd);
grid on;
set(gca, 'XTick', 1:NumWayPoints);
set(gca, 'XTickLabel', WayPointLabels);
xlabel('WayPoints');
ylabel('Percentage [%]');
title('Overall WayPoints performances');


subplot(NumRows, NumCols, 3);
bar(WayPointPerfSubj)
grid on;
ylim([0 120]);
set(gca, 'XTick', 1:NumWayPoints);
set(gca, 'XTickLabel', WayPointLabels);
xlabel('WayPoints');
ylabel('Percentage [%]');
title('WayPoints performances per subject');
legend(sublist);

subplot(NumRows, NumCols, 4);
plot(PerfEvoSubj, '-o')
grid on;
ylim([0 120]);
xlabel('Run');
ylabel('Percentage [%]');
title('Evolution performances per subject');
legend(sublist, 'location', 'southeast');

sgtitle('Parkour performances');

%% Saving figure
filename1 = fullfile(figdir, ['group_parkour_overall_performances.pdf']);
util_bdisp(['[out] - Exporting parkour overall performances to ' filename1]);
fig_export(fig1, filename1, '-pdf');

