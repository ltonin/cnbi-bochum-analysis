clearvars; clc;

subject = 'BOCH04';
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
P  = nav.P;
T  = nav.T;
Rk = labels.Rk;
Dk = labels.Dk;
Xk = cnbibochum_get_data_validity(subject, Rk, Wk);
Nk  = support_get_navigation_labels(events, Rk);
map = map(1);

% Extract data and event for manual
Wk_m = proc_get_event3(Waypoints,     nav_m.T, events_m.POS, events_m.TYP, events_m.DUR);
Tk_m = proc_get_event3(StartEvent,    nav_m.T, events_m.POS, events_m.TYP, events_m.DUR);
P_m  = nav_m.P;
T_m  = nav_m.T;
Rk_m = labels_m.Rk;
Xk_m = cnbibochum_get_data_validity(manual, Rk_m, Wk_m);

Runs = unique(Rk);
NumRuns = length(Runs);
Days = unique(Dk);
NumDays = length(Days);

Runs_m = unique(Rk_m);
NumRuns_m = length(Runs_m);

% Add offset for simulated runs

for rId = 1:NumRuns_m
    cindex = Rk_m == Runs_m(rId);
    cp= P_m(cindex, 1);
    cp(end-1500:end, 1) = cp(end-1500:end, 1) - 0.5;
    P_m(cindex, 1) = cp;
end


%% Compute reference trajectory
Tj_m    = reshape_full_trajectories(P_m, Rk_m, Tk_m);
Tj_ref  = nanmean(Tj_m, 3);
Twj_m   = reshape_wp_trajectories(P_m, Wk_m, Rk_m, Tk_m);
Twj_ref = cellfun(@(m) mean(m, 3), Twj_m, 'UniformOutput', false);


%% Compute Frechet distance for each waypoint
WpFrechet = nan(NumWaypoints, NumRuns);
for wId = 1:NumWaypoints
    for rId = 1:NumRuns
        cindex = Rk == Runs(rId) & Tk > 0 & Wk == Waypoints(wId) & Xk == true & Nk == true;
        if sum(cindex) <= 1
            continue;
        end
        cpath = P(cindex, :);
        rpath = Twj_ref{wId};
        WpFrechet(wId, rId) = proc_frechet_distance(cpath, rpath);
    
    end
end

FullFrechet = max(WpFrechet)';

%% Compute image map trajectories
FieldSize = [map.info.x(end) map.info.y(end)];
MapResolution = 0.1;
MapSize = ceil(FieldSize./MapResolution);

Maps = nan(MapSize(1), MapSize(2), NumRuns);

for rId = 1:NumRuns
   cindex = Rk == Runs(rId) & Tk > 0 & Nk == true;
   Maps(:, :, rId) = cnbibochum_traj2map(P(cindex, :) - map.info.origin, FieldSize, MapResolution);
end

% Compute average trajectory
Tj     = reshape_full_trajectories(P, Rk, Tk);
Tj_avg = nanmean(Tj, 3);


%% Figure

fig1 = figure;
fig_set_position(fig1, 'All');

NumRows = 2;
NumCols = 4;

subplot(NumRows, NumCols, [1 2]);
boxcolors = colororder;
boxplot((WpFrechet)', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
grid on;
xlabel('waypoint');
ylabel('[m]');
title([subject '| Frechet distance per waypoint']);

subplot(NumRows, NumCols, [5 6]);
plot(1:NumRuns, WpFrechet', '-o');
daychanges = find(diff(Dk))+1;
for dId = 1:length(daychanges)
    plot_vline(Rk(daychanges(dId))-0.5, 'k');
end
grid on;
ylabel('[m]')
xlabel('run');
xlim([1 NumRuns]);
title([subject ' | Frechet distance per run and waypoint']);
legend([repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)])

subplot(NumRows, NumCols, [3 4 7 8])
hold on;
PMaps = mean(Maps, 3)./max(Maps, [], 3);
PMaps(PMaps == 0) = nan;
h1 = imagesc(map.info.y, map.info.x, PMaps, [0 0.5]); 
set(gca, 'YDir', 'normal', 'XDir', 'reverse'); 
axis image;

Tmap = double(map.data);
Tmap(Tmap ~= 100) = nan;
h2 = imagesc(map.info.y, map.info.x, Tmap);
set(h2, 'AlphaData', ~isnan(Tmap));

mTj = Tj_avg - map.info.origin;
plot(mTj(:, 2), mTj(:, 1), 'k', 'LineWidth', 2);
mTj_m = Tj_ref - map.info.origin;
plot(mTj_m(:, 2), mTj_m(:, 1), 'color', boxcolors(2, :), 'LineWidth', 2);

hold off;
 
grid minor;
xlabel('[m]')
ylabel('[m]');
title([subject ' | HitMap over all trajectories and average']);
legend({'Average', 'Optimal'}, 'location', 'NorthEast');

%% Saving figure
figname = fullfile(figpath, [subject '_parkour_frechet.pdf']);
util_disp(['[out] - Saving figure in: ' figname]);
fig_export(fig1, figname, '-pdf', 'landscape', '-fillpage');



%% Functions

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



