clearvars; close all;

sublist = {'BOCH02', 'BOCH04', 'BOCH05'};
manualsubject  = 'SIM01';

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
StartEvent    = 500;
Waypoints     = [501 502 503 504];
NumWaypoints  = length(Waypoints);
CommandEvents = [26117 26118];
CommandLabels = {'left', 'right'};

NumSubjects = length(sublist);

%% Manual data
files_manual  = util_getfile3(datapath, '.mat', 'include', [include manualsubject],  'exclude', excludepat, 'level', depthlevel);
[nav_m,   ~,  events_m, labels_m] = cnbibochum_navigation_concatenate_data(files_manual,  eventpath);

Wk_m = proc_get_event3(Waypoints,     nav_m.T, events_m.POS, events_m.TYP, events_m.DUR);
Tk_m = proc_get_event3(StartEvent,    nav_m.T, events_m.POS, events_m.TYP, events_m.DUR);
Ck_m = proc_get_event3(CommandEvents, nav_m.T, events_m.POS, events_m.TYP, events_m.DUR);
P_m  = nav_m.P;
Vx_m = nav_m.Vx;
T_m  = nav_m.T;
Rk_m = labels_m.Rk;
Xk_m = cnbibochum_get_data_validity(manualsubject, Rk_m, Wk_m);

Runs_m = unique(Rk_m);
NumRuns_m = length(Runs_m);

% Add offset for simulated runs
for rId = 1:NumRuns_m
    cindex = Rk_m == Runs_m(rId);
    cp= P_m(cindex, :);
%     cp(end-1500:end, 1) = cp(end-1500:end, 1) - 0.5;
    cp(:, 1) = cp(:, 1) - 0.7;
    cp(:, 2) = cp(:, 2) + 0.4;
    P_m(cindex, :) = cp;
end

%% Subject data
Wk = []; Tk = []; P = []; T = []; Vx = []; Rk = []; Xk = []; Nk = []; Sk = []; Ck = [];
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

%% Compute correction of duration of simulation runs (based on velocity and per subject)
L = []; D = []; V = [];
for sId = 1:NumSubjects
    cindex = Sk == Subjects(sId);
    L = cat(1, L, support_path_length(P(cindex, :), Rk(cindex), Tk(cindex), Xk(cindex)));
    D = cat(1, D, support_path_duration(T(cindex), Rk(cindex), Tk(cindex), Xk(cindex)));
    V = cat(1, V, support_path_velocity(Vx(cindex), Rk(cindex), Tk(cindex), Xk(cindex)));
end

L_m = support_path_length(P_m, Rk_m, Tk_m, Xk_m);
D_m = support_path_duration(T_m, Rk_m, Tk_m, Xk_m);
V_m = support_path_velocity(Vx_m, Rk_m, Tk_m, Xk_m);

WcVx     = L./D;
CmdVelVx = V;

WcVxSim     = L_m./D_m;
CmdVelVxSim = V_m;

RatioVx = nan(NumSubjects, 1);
for sId = 1:NumSubjects
    
    RatioVx(sId) = mean(WcVx(rSk == Subjects(sId))./mean(WcVxSim));
end

%% Compute waypoint duration
DurationWp_mr = support_waypoint_duration(T_m, Wk_m, Rk_m, Tk_m, Xk_m);

DurationWp = [];
for sId = 1:NumSubjects
    cindex = Sk == Subjects(sId);
    DurationWp = cat(2, DurationWp, support_waypoint_duration(T(cindex), Wk(cindex), Rk(cindex), Tk(cindex), Xk(cindex)));
end

% Correction duration for simulation
DurationWp_m = [];
for sId = 1:NumSubjects
    cruns = unique(Rk(Sk == Subjects(sId)));
    cnruns = length(cruns);
    DurationWp_m = cat(2, DurationWp_m, repmat(mean(DurationWp_mr - DurationWp_mr.*((RatioVx(sId)) - 1), 2), [1 cnruns]));
end

%% Compute waypoint commands
Commands = [];
for sId = 1:NumSubjects
    cindex = Sk == Subjects(sId);
    Commands = cat(3, Commands, support_waypoint_commands(Ck(cindex), Wk(cindex), Rk(cindex), Xk(cindex)));
end
Commands_m = repmat(mean(support_waypoint_commands(Ck_m, Wk_m, Rk_m, Xk_m), 3), [1 1 size(Commands, 3)]);

%% Figure 1 | Distribution path duration per waypoint
fig1 = figure; 
fig_set_position(fig1, 'All');

NumRows = 6;
NumCols = 2;

subplot(NumRows, NumCols, [3 5 7 9]);
boxcolors = colororder;
boxplot(100*(DurationWp./DurationWp_m)', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
grid on;
xlabel('waypoint');
ylabel('[%]');
plot_hline(100, 'k--');
% ylim([30 190]);
title('Duration ratio for all subjects');

for sId = 1:NumSubjects
    subplot(NumRows, NumCols, [2 4] + 4*(sId-1));
    boxcolors = colororder;
    boxplot(100*(DurationWp(:, rSk == Subjects(sId))./DurationWp_m(:, rSk == Subjects(sId)))', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
    grid on;
    xlabel('waypoint');
    ylabel('[%]');
    ylim([50 350]);
%     set(gca, 'YTick', 30:20:190);
    plot_hline(100, 'k--');
    title(sublist(sId));
end
sgtitle('Distribution of duration ratio BCI/Manual [%]');

%% Figure 2 | Total ratio path duration 
fig2 = figure; 
fig_set_position(fig2, 'All');

NumRows = 6;
NumCols = 2;

subplot(NumRows, NumCols, [3 5 7 9]);
AvgRatioDuration = nan(NumSubjects, 1);
SteRatioDuration = nan(NumSubjects, 1);
for sId = 1:NumSubjects
    AvgRatioDuration(sId) = nanmean(100*nansum(DurationWp(:, rSk == Subjects(sId))./nansum(DurationWp_m(:, rSk == Subjects(sId)))));
    SteRatioDuration(sId) = nanstd(100*nansum(DurationWp(:, rSk == Subjects(sId))./nansum(DurationWp_m(:, rSk == Subjects(sId)))))./sqrt(sum(rSk == Subjects(sId)));
end

h = bar(AvgRatioDuration);
hold on;
x = h.XEndPoints;
errorbar(x, AvgRatioDuration, SteRatioDuration, '.k');
hold off;
grid on;
plot_hline(100, 'k--');
set(gca, 'XTickLabel', sublist);
xlabel('subject');
ylabel('[%]');
title('Average duration ratio');

for sId = 1:NumSubjects
    subplot(NumRows, NumCols, [2 4] + 4*(sId-1));
    bar(100*nansum(DurationWp(:, rSk == Subjects(sId))./nansum(DurationWp_m(:, rSk == Subjects(sId))))')
    grid on;
    xlabel('run');
    ylabel('[%]');
    ylim([0 250]);
    plot_hline(100, 'k--');
    title(sublist(sId));
end
sgtitle('Total duration ratio BCI/Manual [%]');

%% Figure 3 | Distribution ratio command
fig3 = figure; 
fig_set_position(fig3, 'All');

NumRows = 6;
NumCols = 2;

subplot(NumRows, NumCols, [3 5 7 9]);
boxcolors = colororder;
boxplot(squeeze(100*(nansum(Commands)./nansum(Commands_m)))', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
grid on;
xlabel('waypoint');
ylabel('[%]');
plot_hline(100, 'k--');
% ylim([30 190]);
title('Distribution of command ratio for all subjects');

for sId = 1:NumSubjects
    subplot(NumRows, NumCols, [2 4] + 4*(sId-1));
    boxcolors = colororder;
    cval = squeeze(100*(nansum(Commands(:, :, rSk == Subjects(sId)))./nansum(Commands_m(:, :, rSk == Subjects(sId)))));
    cval(cval == 0) = nan;
    boxplot(cval', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
    grid on;
    xlabel('waypoint');
    ylabel('[%]');
%     ylim([50 350]);
%     set(gca, 'YTick', 30:20:190);
    plot_hline(100, 'k--');
    title(sublist(sId));
end
sgtitle('Distribution of command ratio BCI/Manual [%]');


%% Saving figures

if do_save == false
    return
end

figname1 = fullfile(figpath, 'group_parkour_comparison_duration_distribution.pdf');
figname2 = fullfile(figpath, 'group_parkour_comparison_duration_total.pdf');
figname3 = fullfile(figpath, 'group_parkour_comparison_command_distribution.pdf');


util_disp(['[out] - Saving figure in: ' figname1]);
fig_export(fig1, figname1, '-pdf', 'landscape', '-fillpage');
util_disp(['[out] - Saving figure in: ' figname2]);
fig_export(fig2, figname2, '-pdf', 'landscape', '-fillpage');
util_disp(['[out] - Saving figure in: ' figname3]);
fig_export(fig3, figname3, '-pdf', 'landscape', '-fillpage');


%% Support functions

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

function L = support_path_length(P, Rk, Tk, Xk)
    Runs    = unique(Rk);
    NumRuns = length(Runs);
    L = nan(NumRuns, 1);

    for rId = 1:NumRuns
        cindex = Rk == Runs(rId) & Tk > 0 & Xk == true;
        cpath = P(cindex, :);

        L(rId) = sum(sqrt(sum((cpath(2:end, :) - cpath(1:end-1, :)).^2, 2)));
    end
end

function D = support_path_duration(T, Rk, Tk, Xk)
    
    Runs    = unique(Rk);
    NumRuns = length(Runs);
    D = nan(NumRuns, 1);

    for rId = 1:NumRuns
        cindex = Rk == Runs(rId) & Tk > 0 & Xk == true;
        ctime = T(cindex);

        D(rId) = ctime(end) - ctime(1);
    end
end

function V = support_path_velocity(Vx, Rk, Tk, Xk)
    
    Runs    = unique(Rk);
    NumRuns = length(Runs);
    V = nan(NumRuns, 1);

    for rId = 1:NumRuns
        cindex = Rk == Runs(rId) & Tk > 0 & Xk == true;
        cvelocities = Vx(cindex);

        V(rId) = mean(cvelocities);
    end
end

function D = support_waypoint_duration(T, Wk, Rk, Tk, Xk)
    
    Waypoints    = setdiff(unique(Wk), 0);
    NumWaypoints = length(Waypoints);
    Runs         = unique(Rk);
    NumRuns      = length(Runs);
    
    D = nan(NumWaypoints, NumRuns);

    for rId = 1:NumRuns
        for wId = 1:NumWaypoints
            cindex = Rk == Runs(rId) & Wk == Waypoints(wId) & Tk > 0 & Xk == true;
            cstart = find(cindex, 1, 'first');
            cstop  = find(cindex, 1, 'last');
            if (isequal(cstart, cstop))
                cdur = nan;
            else
                cdur = T(cstop) - T(cstart);
            end
            D(wId, rId) = cdur;
        end
    end


end

function C = support_waypoint_commands(Ck, Wk, Rk, Xk)
    Runs = unique(Rk);
    NumRuns = length(Runs);
    Commands = setdiff(unique(Ck), 0);
    NumCommands = length(Commands);
    Waypoints    = setdiff(unique(Wk), 0);
    NumWaypoints = length(Waypoints);

    C = nan(NumCommands, NumWaypoints, NumRuns);
    for rId = 1:NumRuns
        for wId = 1:NumWaypoints
            cindex = Rk == Runs(rId) & Wk == Waypoints(wId) & Xk == true;
            if sum(cindex) <= 1
                continue;
            end
            for cId = 1:NumCommands
               C(cId, wId, rId) = sum(cindex & Ck == Commands(cId));
            end
        end
    end
end

