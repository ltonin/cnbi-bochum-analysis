clearvars; clc; close all;

sublist = {'BOCH02', 'BOCH04', 'BOCH05'};


include = {'mi', 'mi_bhbf', 'processed'};
excludepat  = {};
depthlevel  = 3;

datapath  = 'analysis/navigation/';
eventpath = 'analysis/navigation/new/waypoints/';
figpath   = './figures/parkour/new/';
do_save   = true;

% Create/Check for figures
util_mkdir(pwd, figpath);

% Default Waypoints values
StartEvent   = 500;
Waypoints    = [501 502 503 504];
NumWaypoints = length(Waypoints);

NumSubjects = length(sublist);

Wk = []; Tk = []; P = []; T = []; Vx = []; Rk = []; Xk = []; Nk = []; Sk = [];
for sId = 1:NumSubjects
    csubject = sublist{sId};
    files = util_getfile3(datapath, '.mat', 'include', [csubject include], 'exclude', excludepat, 'level', depthlevel);
    NumFiles = length(files);

    %% Concatenate files
    [nav, map, events, labels] = cnbibochum_navigation_concatenate_data(files, eventpath);

    % Extract data and events
    cWk  = proc_get_event3(Waypoints,  nav.T, events.POS, events.TYP, events.DUR);
    cTk  = proc_get_event3(StartEvent, nav.T, events.POS, events.TYP, events.DUR);
    cP   = nav.P;
    cT   = nav.T;
    cVx  = nav.Vx;
    cRk  = labels.Rk;
    cXk  = cnbibochum_get_data_validity(csubject, cRk, cWk);
    cNk  = support_get_navigation_labels(events, cRk);
    map = map(1);
    
    % Concatenate data
    Wk = cat(1, Wk, cWk);
    Tk = cat(1, Tk, cTk);
    P  = cat(1, P, cP);
    T  = cat(1, T, cT);
    Vx = cat(1, Vx, cVx);
    Rk = cat(1, Rk, cRk);
    Xk = cat(1, Xk, cXk);
    Nk = cat(1, Nk, cNk);
    Sk = cat(1, Sk, sId*ones(length(cWk), 1));
 
end

Subjects = unique(Sk);
NumSubjects = length(Subjects);

rRk = [];
rSk = [];
for sId = 1:NumSubjects
    cruns = unique(Rk(Sk == Subjects(sId)));
    cnruns = length(cruns);
    rRk = cat(1, rRk, cruns);
    rSk = cat(1, rSk, sId*ones(length(cruns), 1));
end

%% Duration per Waypoint
% DurPerWp = nan(NumWaypoints, NumRuns, NumSubjects);
DurPerWp = [];
for sId = 1:NumSubjects
    cruns = unique(Rk(Sk == Subjects(sId)));
    cnruns = length(cruns);
    
    cDurWp = nan(NumWaypoints, cnruns);
    for rId = 1:cnruns
        for wId = 1:NumWaypoints
            cindex = Rk == cruns(rId) & Wk == Waypoints(wId) & Xk == true & Nk == true & Sk == Subjects(sId);
            cstart = find(cindex, 1, 'first');
            cstop  = find(cindex, 1, 'last');
            if (isequal(cstart, cstop))
                cdur = nan;
            else
                cdur = T(cstop) - T(cstart);
            end

            [~, d2] = support_conditional_duration(T(cindex), Nk(cindex));
            cDurWp(wId, rId) = cdur;
        end
    end
    DurPerWp = cat(2, DurPerWp, cDurWp); 
end

%% Holding time per run
HoldPerRun = [];
HoldRunIdx = [];

for sId = 1:NumSubjects
    cruns = unique(Rk(Sk == Subjects(sId)));
    cnruns = length(cruns);
    
    cHoldRun = nan(cnruns, 1);
    cHoldIdx = cell(cnruns, 1);
    for rId = 1:cnruns
        cindex = Rk == cruns(rId) & Tk == 500 & Xk == true & Sk == Subjects(sId);

        cT  = T(cindex);
        cNk = Nk(cindex);
        cVx = Vx(cindex);

        % remove velocity less than 0
        cVx(cVx < 0) = nan;

        % Normalize velocity
        nvx = rescale(cVx);

        % Velocity value greater than mean/2
        cLk = ~(nvx < mean(nvx)./2);

        [cruntime, choldtime] = support_conditional_duration(cT, cNk & cLk);


        cHoldRun(rId) = 100*nansum(choldtime)./cruntime;
        cHoldIdx{rId} = find(~(cNk & cLk));
    end
    HoldPerRun = cat(1, HoldPerRun, cHoldRun);
    HoldRunIdx = cat(1, HoldRunIdx, cHoldIdx);
end


%% ANOVA tests
a = reshape(DurPerWp', numel(DurPerWp), 1);
b = repmat(rSk', 1, 4);
br = reshape(b, numel(b), 1);
[p, t, s] = anova1(a, br);
[c, m] = multcompare(s, 'display', 'on', 'alpha', 0.01, 'ctype', 'bonferroni');



%% Figure 1 | Distribution path duration
fig1 = figure;
fig_set_position(fig1, 'All');

NumRows = 6;
NumCols = 2;

subplot(NumRows, NumCols, [3 5 7 9]);
boxcolors = colororder;
boxplot((DurPerWp)', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
grid on;
xlabel('waypoint');
ylabel('[s]');
ylim([30 190]);
title('Distribution for all subjects');

for sId = 1:NumSubjects
    subplot(NumRows, NumCols, [2 4] + 4*(sId-1));
    boxcolors = colororder;
    boxplot((DurPerWp(:, rSk == Subjects(sId)))', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
    grid on;
    xlabel('waypoint');
    ylabel('[s]');
    ylim([30 190]);
    set(gca, 'YTick', 30:20:190);
    title(sublist(sId));
end
sgtitle('Distribution of path duration');

%% Figure 2 | Total path duration
fig2 = figure;
fig_set_position(fig2, 'All');

NumRows = 6;
NumCols = 2;

subplot(NumRows, NumCols, [3 5 7 9]);
meanDurPerWp = nan(NumWaypoints, NumSubjects);
for sId = 1:NumSubjects
    meanDurPerWp(:, sId) = nanmean(DurPerWp(:, rSk == Subjects(sId)), 2);
end
bar(meanDurPerWp', 'stack');
set(gca, 'XTickLabel', sublist);
ylim([0 500]);
grid on;
ylabel('[s]')
xlabel('subject');
title('Average path duration');
legend([repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)])

for sId = 1:NumSubjects
    subplot(NumRows, NumCols, [2 4] + 4*(sId-1));
    bar(DurPerWp(:, rSk == Subjects(sId))', 'stack');
    ylim([0 500]);
    grid on;
    ylabel('[s]')
    xlabel('run');
    title(sublist{sId});

    legend([repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)])
end

sgtitle('Path duration')

%% Figure 3 | Holding per run
fig3 = figure;
fig_set_position(fig3, 'All');


meanHoldPerRun = nan(NumSubjects, 1);
stdHoldPerRun  = nan(NumSubjects, 1);
steHoldPerRun  = nan(NumSubjects, 1);
for sId = 1:NumSubjects
    meanHoldPerRun(sId) = nanmean(HoldPerRun(rSk == Subjects(sId)));
    stdHoldPerRun(sId)  = nanstd(HoldPerRun(rSk == Subjects(sId)));
    steHoldPerRun(sId)  = nanstd(HoldPerRun(rSk == Subjects(sId)))./sqrt(sum(rSk == Subjects(sId)));
end

subplot(NumRows, NumCols, [3 5 7 9]);
h = bar(meanHoldPerRun);
hold on;
x = h.XEndPoints;
m = meanHoldPerRun;
e = steHoldPerRun;
errorbar(x(1, :), m(:, 1), e(:, 1), '.k');
hold off;
grid on;
ylabel('[%]')
xlabel('subject');
ylim([0 100]);
set(gca, 'XTickLabel', sublist);
title('Average percentage of time with reduced speed (vx<50%)');

for sId = 1:NumSubjects
    cruns = unique(rRk(rSk == Subjects(sId)));
    cnruns = length(cruns);
    subplot(NumRows, NumCols, [2 4] + 4*(sId-1));
    plot(1:cnruns, HoldPerRun(rSk == Subjects(sId)), 'o-');
    ylim([0 100]);
    
    set(gca, 'XTick', 1:cnruns);
    xlim([1 cnruns]);
    grid on;
    xlabel('run');
    ylabel('[%]');

    title(sublist{sId});
end

sgtitle('Time with reduced speed');

%% Figure 4 | Holding position
fig4 = figure;
fig_set_position(fig4, 'All');

for sId = 1:NumSubjects
    subplot(1, 3, sId);
    cnbibochum_show_map(map.info.x, map.info.y, map.data);
    
    cruns = unique(rRk(rSk == Subjects(sId)));
    cnruns = length(cruns);
    
    choldIdx = HoldRunIdx(rRk(rSk == Subjects(sId)));
    for rId = 1:cnruns
        hold on;
        cpoint = P(choldIdx{rId}, :) - (map.info.origin);
        plot(cpoint(:, 2), cpoint(:, 1), '.', 'MarkerSize', 8);
        hold off;
    end
    title(sublist{sId});
    HoldingRuns = find(cellfun(@(m) isempty(m) == false, choldIdx));
    legend([repmat('run ', length(HoldingRuns), 1) num2str((HoldingRuns))],'fontsize', 6, 'NumColumns', ceil(cnruns/3), 'location', 'southoutside'); 
    
end

sgtitle('Positions with reduced speed (vx<50%)');

%% Saving figures

if do_save == false
    return
end

figname1 = fullfile(figpath, 'group_parkour_duration_distribution.pdf');
figname2 = fullfile(figpath, 'group_parkour_duration_total.pdf');
figname3 = fullfile(figpath, 'group_parkour_duration_holding_percentage.pdf');
figname4 = fullfile(figpath, 'group_parkour_duration_holding_position.pdf');

util_disp(['[out] - Saving figure in: ' figname1]);
fig_export(fig1, figname1, '-pdf', 'landscape', '-fillpage');
util_disp(['[out] - Saving figure in: ' figname2]);
fig_export(fig2, figname2, '-pdf', 'landscape', '-fillpage');
util_disp(['[out] - Saving figure in: ' figname3]);
fig_export(fig3, figname3, '-pdf', 'landscape', '-fillpage');
util_disp(['[out] - Saving figure in: ' figname4]);
fig_export(fig4, figname4, '-pdf', 'landscape', '-fillpage');


%% Support functions

function Nk = support_get_navigation_labels(events, Rk)

    nsamples = length(Rk);
    Runs = unique(Rk);
    NumRuns = length(Runs);
    
    Nk = false(nsamples, 1);
    
    for rId = 1:NumRuns
        cstart = find(Rk == Runs(rId), 1, 'first');
        cstop  = find(Rk == Runs(rId), 1, 'last');
        
        cindex = events.POS >= cstart & events.POS <= cstop & (events.TYP == 1 | events.TYP == 2 | events.TYP == 3);
        evtTYP = events.TYP(cindex);
        evtPOS = events.POS(cindex);
        evtLBL = events.LBL(cindex);
        
        evt_pos_nav = evtPOS(evtTYP == 2);
        evt_pos_oth = evtPOS(evtTYP ~= 2);
        
        for nId = 1:length(evt_pos_nav)
            cnav_pos_start = evt_pos_nav(nId);
            cnav_pos_stop_idx  = find(evt_pos_oth > cnav_pos_start, 1, 'first');
            
            if( isempty(cnav_pos_stop_idx) == true)
                cnav_pos_stop = cstop;
            else
                cnav_pos_stop = evt_pos_oth(cnav_pos_stop_idx);
            end
            
            Nk(cnav_pos_start:cnav_pos_stop) = true;
            
        end
    end

end

function [d1, d2] = support_conditional_duration(T, Nk)

    d1 = 0;
    d2 = 0;

    cstart = find(diff([0; Nk; 0]) == 1);
    cstop  = find(diff([0; Nk; 0]) == -1);
    if(isempty(cstop) == false)
        cstop(end) = cstop(end) -1;
    end
    
    if(isequal(length(cstart), length(cstop)) == false)
        error('check length');
    end
    
    if(isempty(cstart))
        return;
    end
    
    d1 = 0;
    for tId = 1:length(cstart)
        d1 = d1 + T(cstop(tId)) - T(cstart(tId));
    end
    
    d2 = (T(end) - T(1)) - d1;

end
