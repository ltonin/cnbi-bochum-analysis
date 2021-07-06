clearvars; clc;

subject = 'BOCH02';

include = {subject, 'mi', 'mi_bhbf', 'processed'};
excludepat  = {};
depthlevel  = 3;

datapath  = 'analysis/navigation/';
eventpath = 'analysis/navigation/new/waypoints/';
figpath   = './figures/parkour/new/';
do_save   = true;

files = util_getfile3(datapath, '.mat', 'include', include, 'exclude', excludepat, 'level', depthlevel);
NumFiles = length(files);

% Create/Check for figures
util_mkdir(pwd, figpath);

% Default Waypoints values
StartEvent   = 500;
Waypoints    = [501 502 503 504];
NumWaypoints = length(Waypoints);

%% Concatenate files
[nav, map, events, labels] = cnbibochum_navigation_concatenate_data(files, eventpath);

% Extract data and events
Wk  = proc_get_event3(Waypoints, nav.T, events.POS, events.TYP, events.DUR);
Tk  = proc_get_event3(500, nav.T, events.POS, events.TYP, events.DUR);
P   = nav.P;
T   = nav.T;
Vx  = nav.Vx;
Rk  = labels.Rk;
Dk  = labels.Dk;
Xk  = cnbibochum_get_data_validity(subject, Rk, Wk);
Nk  = support_get_navigation_labels(events, Rk);
map = map(1);

Runs = unique(Rk);
NumRuns = length(Runs);
Days = unique(Dk);
NumDays = length(Days);

%% Duration per Waypoint
DurPerWp = nan(NumWaypoints, NumRuns);

for rId = 1:NumRuns
    for wId = 1:NumWaypoints
        cindex = Rk == Runs(rId) & Wk == Waypoints(wId) & Xk == true;
        cstart = find(cindex, 1, 'first');
        cstop  = find(cindex, 1, 'last');
        if (isequal(cstart, cstop))
            cdur = nan;
        else
            cdur = T(cstop) - T(cstart);
        end
        
        [~, d2] = support_conditional_duration(T(cindex), Nk(cindex));
        DurPerWp(wId, rId) = cdur;
    end
end

%% Holding time per run
HoldPerRun = nan(NumRuns, 1);
HoldPerRunIdx = cell(NumRuns, 1);
for rId = 1:NumRuns
    cindex = Rk == Runs(rId) & Tk == 500;
    
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
    
    
    HoldPerRun(rId) = 100*nansum(choldtime)./cruntime;
    HoldPerRunIdx{rId} = find(~(cNk & cLk));
end


%% Figure

fig1 = figure;
fig_set_position(fig1, 'All');
NumRows = 3;
NumCols = 4;

subplot(NumRows, NumCols, [1 2]);
boxcolors = colororder;
boxplot((DurPerWp)', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
grid on;
xlabel('waypoint');
ylabel('[s]');
ylim([30 190]);
title([subject '| Distribution of path duration per waypoint']);

subplot(NumRows, NumCols, [5 6]);
bar(DurPerWp', 'stack');
ylim([0 500]);
daychanges = find(diff(Dk))+1;
for dId = 1:length(daychanges)
    plot_vline(Rk(daychanges(dId))-0.5, 'k');
end
grid on;
ylabel('[s]')
xlabel('run');
title([subject ' | Total path duration']);

legend([repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)])

subplot(NumRows, NumCols, [9 10])
plot(1:NumRuns, HoldPerRun, 'o-');
ylim([0 100]);
for dId = 1:length(daychanges)
    plot_vline(Rk(daychanges(dId))-0.5, 'k');
end
set(gca, 'XTick', 1:NumRuns);
xlim([1 NumRuns]);
grid on;
xlabel('run');
ylabel('[%]');

title([subject ' | Time with reduced speed (vx<50%)']);

subplot(NumRows, NumCols, [3 4 7 8 11 12])
cnbibochum_show_map(map.info.x, map.info.y, map.data);
for rId = 1:NumRuns
    hold on;
    cpoint = P(HoldPerRunIdx{rId}, :) - (map.info.origin);
    plot(cpoint(:, 2), cpoint(:, 1), 'o');
    hold off;
end
title([subject ' | Positions with reduced speed (vx<50%)']);
HoldingRuns = find(cellfun(@(m) isempty(m) == false, HoldPerRunIdx));
legend([repmat('run ', length(HoldingRuns), 1) num2str((HoldingRuns))]);


%% Saving figure

if do_save == false
    return
end

figname = fullfile(figpath, [subject '_parkour_duration.pdf']);
util_disp(['[out] - Saving figure in: ' figname]);
fig_export(fig1, figname, '-pdf', 'landscape', '-fillpage');


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

