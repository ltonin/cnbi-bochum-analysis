%clearvars; clc; close all;

sublist = {'BOCH02', 'BOCH04', 'BOCH05'};


include = {'mi', 'mi_bhbf', 'processed'};
excludepat  = {};
depthlevel  = 3;

datapath  = 'analysis/navigation/';
eventpath = 'analysis/navigation/new/waypoints/';
figpath   = './figures/parkour/new/';
do_save   = false;

% Create/Check for figures
util_mkdir(pwd, figpath);

% Default Waypoints values
StartEvent    = 500;
Waypoints     = [501 502 503 504];
NumWaypoints  = length(Waypoints);
CommandEvents = [26117 26118];
CommandLabels = {'left', 'right'};

NumSubjects = length(sublist);

Wk = []; Tk = []; P = []; T = []; Vx = []; Rk = []; Xk = []; Nk = []; Ck = []; Sk = [];
for sId = 1:NumSubjects
    csubject = sublist{sId};
    files = util_getfile3(datapath, '.mat', 'include', [csubject include], 'exclude', excludepat, 'level', depthlevel);
    NumFiles = length(files);

    %% Concatenate files
    [nav, map, events, labels] = cnbibochum_navigation_concatenate_data(files, eventpath);

    % Extract data and events
    cWk  = proc_get_event3(Waypoints,  nav.T, events.POS, events.TYP, events.DUR);
    cTk  = proc_get_event3(StartEvent, nav.T, events.POS, events.TYP, events.DUR);
    cCk  = proc_get_event3(CommandEvents, nav.T, events.POS, events.TYP, events.DUR);
    cP   = nav.P;
    cT   = nav.T;
    cVx  = nav.Vx;
    cRk  = labels.Rk;
    cXk  = cnbibochum_get_data_validity(csubject, cRk, cWk);
    cNk  = support_get_navigation_labels(events, cRk);
    cCk = support_filter_commands(cT, cCk, 1.5);
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
    Ck = cat(1, Ck, cCk);
    Sk = cat(1, Sk, sId*ones(length(cWk), 1));
 
end

Commands = setdiff(unique(Ck), 0);
NumCommands = length(Commands);
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

%% Commands per run and waypoint

CommandPerRuns = [];

for sId = 1:NumSubjects
    
    cruns = unique(Rk(Sk == Subjects(sId)));
    cnruns = length(cruns);
    
    ccommands = nan(NumCommands, NumWaypoints, cnruns);
    for rId = 1:cnruns
        for wId = 1:NumWaypoints
            cindex = Rk == cruns(rId) & Wk == Waypoints(wId) & Xk == true & Nk == true & Sk == Subjects(sId);

            if(sum(cindex) == 0)
                continue;
            end

            for cId = 1:NumCommands
               ccommands(cId, wId, rId) = nansum(cindex & Ck == Commands(cId));
            end
        end
    end
    
    CommandPerRuns = cat(3, CommandPerRuns, ccommands);
end

%% ANOVA tests
a = squeeze(sum(CommandPerRuns));
ar = reshape(a', numel(a), 1);
b = repmat(rSk', 1, 4);
br = reshape(b, numel(b), 1);
[p, t, s] = anova1(ar, br);
[c, m] = multcompare(s, 'display', 'on', 'alpha', 0.01, 'ctype', 'bonferroni');

%% Figure 1 | Distribution commands per waypoint

fig1 = figure;
fig_set_position(fig1, 'All');
NumRows = 6;
NumCols = 2;

subplot(NumRows, NumCols, [3 5 7 9]);
boxcolors = colororder;
boxplot(squeeze(sum(CommandPerRuns))', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :));
ylim([0 60]);
grid on;
xlabel('waypoint');
ylabel('[#]');
title('Distribution of number of commands per waypoint');

for sId = 1:NumSubjects
    subplot(NumRows, NumCols, [2 4] + 4*(sId-1));
    boxcolors = colororder;
    boxplot(squeeze(sum(CommandPerRuns(:, :, rSk == Subjects(sId))))', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :));
    ylim([0 60]);
    grid on;
    xlabel('waypoint');
    ylabel('[#]');
    title(sublist{sId});
end

sgtitle('Distibution of commands');

%% Figure 2 | Average number of commands per waypoint
fig2 = figure;
fig_set_position(fig2, 'All');
NumRows = 6;
NumCols = 2;

subplot(NumRows, NumCols, [3 5 7 9]);
h = bar(squeeze(nanmean(CommandPerRuns, 3))', 'grouped');
ylim([0 15]);
hold on;
x = [h(1).XEndPoints; h(2).XEndPoints];
m = squeeze(nanmean(CommandPerRuns, 3))';
e = squeeze(nanstd(CommandPerRuns, [], 3)./sqrt(size(CommandPerRuns, 3)))';
errorbar(x(1, :), m(:, 1), e(:, 1), '.k');
errorbar(x(2, :), m(:, 2), e(:, 2), '.k');
hold off;
set(gca, 'XTickLabel', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)]);
grid on;
ylabel('[#]')
xlabel('waypoint');
title('Average number of commands per waypoint');
legend(CommandLabels, 'location', 'northwest')

for sId = 1:NumSubjects
    subplot(NumRows, NumCols, [2 4] + 4*(sId-1));
    bar(squeeze(nansum(CommandPerRuns(:, :, rSk == Subjects(sId)), 2))', 'stack');
    grid on;
    grid on;
    ylabel('[#]')
    xlabel('run');
    title([sublist{sId} ' | # commands']);
    legend(CommandLabels)
end

sgtitle('Number of commands');

%% Figure 3 | Command positions
fig3 = figure;
fig_set_position(fig3, 'All');
NumRows = 1;
NumCols = 3;

for sId = 1:NumSubjects
    subplot(NumRows, NumCols, sId);
    cnbibochum_show_map(map.info.x, map.info.y, map.data);
    
    cruns = unique(rRk(rSk == Subjects(sId)));
    cnruns = length(cruns);
    for rId = 1:cnruns
        for cId = 1:NumCommands
        cIdx = find(Ck == Commands(cId) & Rk == cruns(rId) & Sk == Subjects(sId) & Xk == true & Nk == true);
        hold on;
        cpoint = P(cIdx, :) - (map.info.origin);
        plot(cpoint(:, 2), cpoint(:, 1), '.', 'color', boxcolors(cId, :), 'MarkerSize', 4);
        hold off;
        end
    end
    title(sublist{sId});
    legend(CommandLabels, 'fontsize', 6, 'orientation', 'horizontal');
end
sgtitle('Position of the commands');

%% Saving figures

if do_save == false
    return
end

figname1 = fullfile(figpath, 'group_parkour_commands_distribution.pdf');
figname2 = fullfile(figpath, 'group_parkour_commands_number.pdf');
figname3 = fullfile(figpath, 'group_parkour_commands_position.pdf');

util_disp(['[out] - Saving figure in: ' figname1]);
fig_export(fig1, figname1, '-pdf', 'landscape', '-fillpage');
util_disp(['[out] - Saving figure in: ' figname2]);
fig_export(fig2, figname2, '-pdf', 'landscape', '-fillpage');
util_disp(['[out] - Saving figure in: ' figname3]);
fig_export(fig3, figname3, '-pdf', 'landscape', '-fillpage');


%% Functions

function nCk = support_filter_commands(T, Ck, timetolerance)

    POS = find(Ck == 26117 | Ck == 26118);
    TYP = Ck(Ck == 26117 | Ck == 26118);
    TIM = T(POS);
    
    MSK = [true; diff(TIM) > timetolerance | diff(TYP) ~=0];

    nCk = Ck;
    nCk(POS(~MSK)) = 0;
    
end

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
