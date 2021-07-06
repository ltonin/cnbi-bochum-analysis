clearvars; clc; close all;

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
P_m  = nav_m.P;
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

%% Compute reference trajectory (manual)
Tj_m    = reshape_full_trajectories(P_m, Rk_m, Tk_m);
Tj_ref  = nanmean(Tj_m, 3);
Twj_m   = reshape_wp_trajectories(P_m, Wk_m, Rk_m, Tk_m);
Twj_ref = cellfun(@(m) mean(m, 3), Twj_m, 'UniformOutput', false);

%% Compute Frechet distance for each waypoint

WpFrechet = [];
for sId = 1:NumSubjects
    cruns = unique(Rk(Sk == Subjects(sId)));
    cnruns = length(cruns);
    cWpFrechet = nan(NumWaypoints, cnruns);
    for wId = 1:NumWaypoints
        for rId = 1:cnruns
            cindex = Rk == cruns(rId) & Tk > 0 & Wk == Waypoints(wId) & Xk == true & Nk == true & Sk == Subjects(sId);
            if sum(cindex) <= 1
                continue;
            end
            cpath = P(cindex, :);
            rpath = Twj_ref{wId};
            cWpFrechet(wId, rId) = proc_frechet_distance(cpath, rpath);

        end
    end
    
    WpFrechet = cat(2, WpFrechet, cWpFrechet);
end

%% Compute image map trajectories
FieldSize = [map.info.x(end) map.info.y(end)];
MapResolution = 0.1;
MapSize = ceil(FieldSize./MapResolution);

Maps = [];
for sId = 1:NumSubjects
    cruns = unique(Rk(Sk == Subjects(sId)));
    cnruns = length(cruns);
    cMaps = nan(MapSize(1), MapSize(2), cnruns);

    for rId = 1:cnruns
       cindex = Rk == cruns(rId) & Tk > 0 & Nk == true & Sk == Subjects(sId);
       cMaps(:, :, rId) = cnbibochum_traj2map(P(cindex, :) - map.info.origin, FieldSize, MapResolution);
    end
    Maps = cat(3, Maps, cMaps);
end

% Compute average trajectory
Tj = cell(NumSubjects, 1);
Tj_avg = cell(NumSubjects, 1);

for sId = 1:NumSubjects
    cindex = Sk == Subjects(sId);
    Tj{sId} = reshape_full_trajectories(P(cindex, :), Rk(cindex), Tk(cindex));
    Tj_avg{sId} = nanmean(Tj{sId}, 3);
end

%% ANOVA tests
ar = reshape(WpFrechet', numel(WpFrechet), 1);
b = repmat(rSk', 1, 4);
br = reshape(b, numel(b), 1);
[p, t, s] = anova1(ar, br);
[c, m] = multcompare(s, 'display', 'on', 'alpha', 0.05, 'ctype', 'bonferroni');

%% Figure 1 | Distribution frechet
fig1 = figure;
fig_set_position(fig1, 'All');
NumRows = 6;
NumCols = 2;

subplot(NumRows, NumCols, [3 5 7 9]);
boxcolors = colororder;
boxplot((WpFrechet)', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
grid on;
xlabel('waypoint');
ylabel('[m]');
ylim([0 3.5]);
title('Distribution Frechet distance per waypoint');

for sId = 1:NumSubjects
    subplot(NumRows, NumCols, [2 4] + 4*(sId-1));
    boxcolors = colororder;
    boxplot((WpFrechet(:, rSk == Subjects(sId)))', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
    grid on;
    xlabel('waypoint');
    ylabel('[m]');
    ylim([0 3.5]);
    title('Distribution Frechet distance per waypoint');
    title(sublist{sId});
    
end
sgtitle('Distribution Frechet');

%% Figure 2 | Total frechet distance
fig2 = figure;
fig_set_position(fig2, 'All');
NumRows = 6;
NumCols = 2;

meanFrechet = nan(NumWaypoints, NumSubjects);
stdFrechet  = nan(NumWaypoints, NumSubjects);
steFrechet  = nan(NumWaypoints, NumSubjects);
for sId = 1:NumSubjects
   meanFrechet(:, sId) = nanmean(WpFrechet(:, rSk == Subjects(sId)), 2); 
   stdFrechet(:, sId)  = nanstd(WpFrechet(:, rSk == Subjects(sId)), [], 2); 
   steFrechet(:, sId)  = nanstd(WpFrechet(:, rSk == Subjects(sId)), [], 2)./sqrt(sum(rSk == Subjects(sId))); 
end

subplot(NumRows, NumCols, [3 5 7 9]);
h = bar(meanFrechet', 'stack');
grid on;
set(gca, 'XTickLabel', sublist);
ylabel('[m]')
xlabel('subject');
ylim([0 7]);
title('Average frechet per waypoint');
legend([repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)])

for sId = 1:NumSubjects
    subplot(NumRows, NumCols, [2 4] + 4*(sId-1));
    bar(WpFrechet(:, rSk == Subjects(sId))', 'stacked');
    grid on;
    ylabel('[m]')
    xlabel('run');
    ylim([0 7]);
    title(sublist{sId});
    legend([repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)], 'location', 'northwest')
end

sgtitle('Frechet distance');

%% Figure 3 | Hitmaps
fig3 = figure;
fig_set_position(fig3, 'All');
NumRows = 3;
NumCols = 5;

subplot(NumRows, NumCols, [1 2 6 7 11 12]);
hold on;
PMaps = mean(Maps, 3)./max(Maps, [], 3);
PMaps(PMaps == 0) = nan;
h1 = imagesc(map.info.y, map.info.x, PMaps, [0 0.5]); 
set(gca, 'YDir', 'normal', 'XDir', 'reverse'); 
axis image;

colors = colororder;
linecolors(1, :) = colors(2, :);
linecolors(2, :) = colors(3, :);
linecolors(3, :) = colors(5, :);
for sId = 1:NumSubjects
    cmtracking = Tj_avg{sId} - map.info.origin;
    plot(cmtracking(:, 2), cmtracking(:, 1), 'color', linecolors(sId, :), 'LineWidth', 1);
end
cmtracking_ref = Tj_ref - map.info.origin;
plot(cmtracking_ref(:, 2), cmtracking_ref(:, 1), 'k', 'LineWidth', 1);

Tmap = double(map.data);
% Tmap(Tmap == 100) = 1;
Tmap(Tmap ~= 100) = nan;
h2 = imagesc(map.info.y, map.info.x, Tmap);
set(h2, 'AlphaData', ~isnan(Tmap));

hold off;
 
grid minor;
xlabel('[m]')
ylabel('[m]');
title('HitMap over all trajectories and average');
legend([sublist 'manual'], 'fontsize', 6);

idx(:, 1) = [3 8 13];
idx(:, 2) = [4 9 14];
idx(:, 3) = [5 10 15];
for sId = 1:NumSubjects
    subplot(NumRows, NumCols, idx(:, sId));
    hold on;
    PMaps = mean(Maps(:, :, rSk == Subjects(sId)), 3)./max(Maps(:, :, rSk == Subjects(sId)), [], 3);
    PMaps(PMaps == 0) = nan;
    h1 = imagesc(map.info.y, map.info.x, PMaps, [0 0.5]); 
    set(gca, 'YDir', 'normal', 'XDir', 'reverse'); 
    axis image;
    
    cmtracking = Tj_avg{sId} - map.info.origin;
    plot(cmtracking(:, 2), cmtracking(:, 1), 'color', linecolors(sId, :), 'LineWidth', 1);
    cmtracking_ref = Tj_ref - map.info.origin;
    plot(cmtracking_ref(:, 2), cmtracking_ref(:, 1), 'k', 'LineWidth', 1);
    Tmap = double(map.data);
    Tmap(Tmap ~= 100) = nan;
    h2 = imagesc(map.info.y, map.info.x, Tmap);
    set(h2, 'AlphaData', ~isnan(Tmap));
    hold off;

    grid minor;
    xlabel('[m]')
    ylabel('[m]');
    title(sublist{sId});
end

sgtitle('Hitmap of trajectories')

%% Saving figures

if do_save == false
    return
end

figname1 = fullfile(figpath, 'group_parkour_frechet_distribution.pdf');
figname2 = fullfile(figpath, 'group_parkour_frechet_total.pdf');
figname3 = fullfile(figpath, 'group_parkour_frechet_hitmaps.pdf');

util_disp(['[out] - Saving figure in: ' figname1]);
fig_export(fig1, figname1, '-pdf', 'landscape', '-fillpage');
util_disp(['[out] - Saving figure in: ' figname2]);
fig_export(fig2, figname2, '-pdf', 'landscape', '-fillpage');
util_disp(['[out] - Saving figure in: ' figname3]);
fig_export(fig3, figname3, '-pdf', 'landscape', '-fillpage');


%% Support functions

function Tj = reshape_full_trajectories(P, Rk, Tk)

    Runs    = unique(Rk);
    NumRuns = length(Runs);
    
    % Find max length
    maxlen = 0;
    for rId = 1:NumRuns
        cindex = Rk == Runs(rId) & Tk > 0;
        maxlen = max(length(Rk(cindex)), maxlen);
    end

	Tj = nan(maxlen, 2, NumRuns);
    for rId = 1:NumRuns
        cindex = Rk == Runs(rId) & Tk > 0;
        clength = sum(cindex);
        ctraj = P(cindex, :);
        Tj(:, :, rId) =  interp1(1:clength, ctraj, linspace(1, clength, maxlen));
    end
end

function Twj = reshape_wp_trajectories(P, Wk, Rk, Tk)
    
    Waypoints    = setdiff(unique(Wk), 0);
    NumWayPoints = length(Waypoints);
    Runs         = unique(Rk);
    NumRuns      = length(Runs);
    
    Twj = cell(NumWayPoints, 1);
    for wId = 1:NumWayPoints
        % Find max length
        maxlen = 0;
        for rId = 1:NumRuns
            cindex = Rk == Runs(rId) & Tk > 0 & Wk == Waypoints(wId);
            maxlen = max(length(Rk(cindex)), maxlen);
        end

        Tj = nan(maxlen, 2, NumRuns);
        for rId = 1:NumRuns
            cindex = Rk == Runs(rId) & Tk > 0 & Wk == Waypoints(wId);
            clength = sum(cindex);
            ctraj = P(cindex, :);
            Tj(:, :, rId) =  interp1(1:clength, ctraj, linspace(1, clength, maxlen));
        end
        
        Twj{wId} = Tj;
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

