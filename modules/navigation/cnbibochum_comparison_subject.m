clearvars; clc;

subject = 'BOCH05';
manual  = 'SIM01';

include = {'mi', 'mi_bhbf', 'processed'};
excludepat  = {};
depthlevel  = 3;

datapath  = 'analysis/navigation/';
eventpath = 'analysis/navigation/new/waypoints/';
figpath   = './figures/parkour/new/';

files_subject = util_getfile3(datapath, '.mat', 'include', [include subject], 'exclude', excludepat, 'level', depthlevel);
files_manual  = util_getfile3(datapath, '.mat', 'include', [include manual],  'exclude', excludepat, 'level', depthlevel);
NumFiles = length(files_subject);

StartEvent    = 500;
Waypoints     = [501 502 503 504];
NumWaypoints  = length(Waypoints);
CommandEvents = [26117 26118];
CommandLabels = {'left', 'right'};

%% Concatenate files
[nav,   map,  events,   labels]   = cnbibochum_navigation_concatenate_data(files_subject, eventpath);
[nav_m,   ~,  events_m, labels_m] = cnbibochum_navigation_concatenate_data(files_manual,  eventpath);

% Extract data and event for subject
Wk = proc_get_event3(Waypoints, nav.T, events.POS, events.TYP, events.DUR);
Tk = proc_get_event3(StartEvent, nav.T, events.POS, events.TYP, events.DUR);
Ck = proc_get_event3(CommandEvents, nav.T, events.POS, events.TYP, events.DUR);
P  = nav.P;
T  = nav.T;
Vx = nav.Vx;
Rk = labels.Rk;
Dk = labels.Dk;
Ck = support_filter_commands(T, Ck, 1.5);
Xk = cnbibochum_get_data_validity(subject, Rk, Wk);
Nk  = support_get_navigation_labels(events, Rk);

% Extract data and event for manual
Wk_m = proc_get_event3(Waypoints,     nav_m.T, events_m.POS, events_m.TYP, events_m.DUR);
Tk_m = proc_get_event3(StartEvent,    nav_m.T, events_m.POS, events_m.TYP, events_m.DUR);
Ck_m = proc_get_event3(CommandEvents, nav_m.T, events_m.POS, events_m.TYP, events_m.DUR);
P_m  = nav_m.P;
T_m  = nav_m.T;
Vx_m = nav_m.Vx;
Rk_m = labels_m.Rk;
Xk_m = cnbibochum_get_data_validity(manual, Rk_m, Wk_m);
Nk_m = true(length(Rk_m), 1);


Runs = unique(Rk);
NumRuns = length(Runs);
Days = unique(Dk);
NumDays = length(Days);

Runs_m = unique(Rk_m);
NumRuns_m = length(Runs_m);

% Add offset for simulated runs
P_mo = P_m;
for rId = 1:NumRuns_m
    cindex = Rk_m == Runs_m(rId);
    cp= P_m(cindex, 1);
    cp(end-1500:end, 1) = cp(end-1500:end, 1) - 0.5;
    P_m(cindex, 1) = cp;
end

%% Compute correction of duration of simulation runs (based on velocity)
L = support_compute_path_length(P, Rk, Tk, Xk, Nk);
D = support_compute_path_duration(T, Rk, Tk, Xk, Nk);
V = support_compute_path_velocity(Vx, Rk, Tk, Xk, Nk);

L_m = support_compute_path_length(P_m, Rk_m, Tk_m, Xk_m, Nk_m);
D_m = support_compute_path_duration(T_m, Rk_m, Tk_m, Xk_m, Nk_m);
V_m = support_compute_path_velocity(Vx_m, Rk_m, Tk_m, Xk_m, Nk_m);

WcVx     = L./D;
CmdVelVx = V;

WcVxSim     = L_m./D_m;
CmdVelVxSim = V_m;

RatioVx = mean(WcVx./mean(WcVxSim));

%% Compute waypoint duration
DurationWp = support_compute_waypoint_duration(T, Wk, Rk, Tk, Xk, Nk);
DurationWp_raw = support_compute_waypoint_duration(T_m, Wk_m, Rk_m, Tk_m, Xk_m, Nk_m);

% Correction duration for simulation
DurationWp_m = DurationWp_raw - DurationWp_raw.*(RatioVx - 1);

%% Compute waypoint commands
Commands   = support_compute_waypoint_commands(Ck, Wk, Rk, Xk, Nk);
Commands_m = support_compute_waypoint_commands(Ck_m, Wk_m, Rk_m, Xk_m, Nk_m);

%% Figure

fig1 = figure;
fig_set_position(fig1, 'All');
NumRows = 2;
NumCols = 2;

subplot(NumRows, NumCols, 1);
boxcolors = colororder;
boxplot(100*(DurationWp./repmat(mean(DurationWp_m, 2), [1 NumRuns]))', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
grid on;
xlabel('waypoint');
plot_hline(100, 'k--');
ylabel('[%]');
title([subject '| Distribution of duration ratios: 100*(BCI/Manual)']);

subplot(NumRows, NumCols, 3);
bar(100*(DurationWp./repmat(mean(DurationWp_m, 2), [1 NumRuns]))', 'grouped');
% plot(100*(DurationWp./DurationWp_m)', 'o-');
plot_hline(100, 'k--');
daychanges = find(diff(Dk))+1;
for dId = 1:length(daychanges)
    plot_vline(Rk(daychanges(dId))-0.5, 'k');
end
grid on;
ylabel('[%]')
xlabel('run');
title([subject ' | Ratios BCI/manual duration per run']);
legend([repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)])

subplot(NumRows, NumCols, 2);
boxcolors = colororder;
boxplot(100*squeeze(nansum(Commands)./nansum(repmat(nanmean(Commands_m, 3), [1 1 NumRuns])))', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :));
grid on;
plot_hline(100, 'k--');
xlabel('waypoint');
ylabel('[%]');
title([subject '| Distribution of command ratios: 100*(BCI/Manual)']);

subplot(NumRows, NumCols, 4);
h = bar(100*squeeze(nansum(Commands)./nansum(repmat(nanmean(Commands_m, 3), [1 1 NumRuns])))', 'grouped');
% plot(100*squeeze(sum(Commands)./sum(Commands_m))', 'o-');
plot_hline(100, 'k--');
% set(gca, 'XTickLabel', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)]);
grid on;
ylabel('[5]')
xlabel('run');
title([subject ' | Ratios BCI/manual number commands per run']);
legend([repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)])

%% Saving figure
figname = fullfile(figpath, [subject '_parkour_comparison_manual.pdf']);
util_disp(['[out] - Saving figure in: ' figname]);
fig_export(fig1, figname, '-pdf', 'landscape', '-fillpage');


%% Functions

function nCk = support_filter_commands(T, Ck, timetolerance)

    POS = find(Ck == 26117 | Ck == 26118);
    TYP = Ck(Ck == 26117 | Ck == 26118);
    TIM = T(POS);
    
    MSK = [true; diff(TIM) > timetolerance | diff(TYP) ~=0];

    nCk = Ck;
    nCk(POS(~MSK)) = 0;
    
end

function L = support_compute_path_length(P, Rk, Tk, Xk, Nk)
    Runs    = unique(Rk);
    NumRuns = length(Runs);
    L = nan(NumRuns, 1);

    for rId = 1:NumRuns
        cindex = Rk == Runs(rId) & Tk > 0 & Xk == true & Nk == true;
        cpath = P(cindex, :);

        L(rId) = sum(sqrt(sum((cpath(2:end, :) - cpath(1:end-1, :)).^2, 2)));
    end
end

function D = support_compute_path_duration(T, Rk, Tk, Xk, Nk)
    
    Runs    = unique(Rk);
    NumRuns = length(Runs);
    D = nan(NumRuns, 1);

    for rId = 1:NumRuns
        cindex = Rk == Runs(rId) & Tk > 0 & Xk == true;
        ctime = T(cindex);
        cNk   = Nk(cindex);
        cruntime = support_conditional_duration(ctime, cNk);

        D(rId) = cruntime;
    end
end

function D = support_compute_waypoint_duration(T, Wk, Rk, Tk, Xk, Nk)
    
    Waypoints    = setdiff(unique(Wk), 0);
    NumWaypoints = length(Waypoints);
    Runs         = unique(Rk);
    NumRuns      = length(Runs);
    
    D = nan(NumWaypoints, NumRuns);

    for rId = 1:NumRuns
        for wId = 1:NumWaypoints
            cindex = Rk == Runs(rId) & Wk == Waypoints(wId) & Tk > 0 & Xk == true;
            
            cT = T(cindex);
            cNk = Nk(cindex);
            cruntime = support_conditional_duration(cT, cNk);
%             cstart = find(cindex, 1, 'first');
%             cstop  = find(cindex, 1, 'last');
%             if (isequal(cstart, cstop))
%                 cdur = nan;
%             else
%                 cdur = T(cstop) - T(cstart);
%             end
            D(wId, rId) = cruntime;
        end
    end


end

function V = support_compute_path_velocity(Vx, Rk, Tk, Xk, Nk)
    
    Runs    = unique(Rk);
    NumRuns = length(Runs);
    V = nan(NumRuns, 1);

    for rId = 1:NumRuns
        cindex = Rk == Runs(rId) & Tk > 0 & Xk == true & Nk == true;
        cvelocities = Vx(cindex);

        V(rId) = mean(cvelocities);
    end
end

function C = support_compute_waypoint_commands(Ck, Wk, Rk, Xk, Nk)
    Runs = unique(Rk);
    NumRuns = length(Runs);
    Commands = setdiff(unique(Ck), 0);
    NumCommands = length(Commands);
    Waypoints    = setdiff(unique(Wk), 0);
    NumWaypoints = length(Waypoints);

    C = nan(NumCommands, NumWaypoints, NumRuns);
    for rId = 1:NumRuns
        for wId = 1:NumWaypoints
            cindex = Rk == Runs(rId) & Wk == Waypoints(wId) & Xk == true & Nk == true;
            if sum(cindex) <= 1
                continue;
            end
            for cId = 1:NumCommands
               C(cId, wId, rId) = sum(cindex & Ck == Commands(cId));
            end
        end
    end
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
