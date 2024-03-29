clearvars; clc;

subject = 'BOCH04';

include = {subject, 'mi', 'mi_bhbf', 'processed'};
excludepat  = {};
depthlevel  = 3;

datapath  = 'analysis/navigation/';
eventpath = 'analysis/navigation/new/waypoints/';
figpath   = './figures/parkour/new/';
do_save   = false;

files = util_getfile3(datapath, '.mat', 'include', include, 'exclude', excludepat, 'level', depthlevel);
NumFiles = length(files);

% Create/Check for figures
util_mkdir(pwd, figpath);

% Default Waypoints values
StartEvent    = 500;
Waypoints     = [501 502 503 504];
NumWaypoints  = length(Waypoints);
CommandEvents = [26117 26118];
CommandLabels = {'left', 'right'};


%% Concatenate files
[nav, map, events, labels] = cnbibochum_navigation_concatenate_data(files, eventpath);

% Extract data and events
Wk = proc_get_event3(Waypoints, nav.T, events.POS, events.TYP, events.DUR);
Tk = proc_get_event3(StartEvent, nav.T, events.POS, events.TYP, events.DUR);
P = nav.P;
T = nav.T;
Vx = nav.Vx;
Rk = labels.Rk;
Dk = labels.Dk;
map = map(1);
Xk = cnbibochum_get_data_validity(subject, Rk, Wk);
Nk  = support_get_navigation_labels(events, Rk);


Runs = unique(Rk);
NumRuns = length(Runs);
Days = unique(Dk);
NumDays = length(Days);

% Add offset for simulated runs
if strcmp(subject, 'SIM01')
    for rId = 1:NumRuns
        cindex = Rk == Runs(rId);
        cp= P(cindex, 1);
        cp(end-1500:end, 1) = cp(end-1500:end, 1) - 0.5;
        P(cindex, 1) = cp;
    end
end

%% Path length
PathLength = nan(NumWaypoints, NumRuns);

for rId = 1:NumRuns
    for wId = 1:NumWaypoints
        cindex = Rk == Runs(rId) & Wk == Waypoints(wId) & Tk > 0 & Xk == true & Nk == true;
        
        if(sum(cindex) == 0)
            continue;
        end
        
        cpath = P(cindex, :);

        PathLength(wId, rId) = sum(sqrt(sum((cpath(2:end, :) - cpath(1:end-1, :)).^2, 2)));
    end
    
end
   

%% Hitmaps for trajectories
FieldSize = [map.info.x(end) map.info.y(end)];
MapResolution = 0.1;
MapSize = ceil(FieldSize./MapResolution);

Maps = nan(MapSize(1), MapSize(2), NumRuns);

for rId = 1:NumRuns
   cindex = Rk == Runs(rId) & Tk > 0 & Nk == true;
   Maps(:, :, rId) = cnbibochum_traj2map(P(cindex, :) - map.info.origin, FieldSize, MapResolution);
end


%% Resampled trajectories
% Find max length
maxlength = 0;
for rId = 1:NumRuns
    cindex = Rk == Runs(rId) & Tk > 0 & Nk == true;
    maxlength = max(length(Rk(cindex)), maxlength);
end

rtracking = nan(maxlength, 2, NumRuns);
for rId = 1:NumRuns
    cindex = Rk == Runs(rId) & Tk > 0 & Nk == true;
    clength = sum(cindex);
    cpath = P(cindex, :);
    rtracking(:, :, rId) =  interp1(1:clength, cpath, linspace(1, clength, maxlength));
end

%% Average trajectory
mtracking = nanmean(rtracking, 3);

%% Figure

fig1 = figure;
fig_set_position(fig1, 'All');

NumRows = 2;
NumCols = 4;

subplot(NumRows, NumCols, [1 2]);
boxcolors = colororder;
boxplot((PathLength)', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
grid on;
xlabel('waypoint');
ylabel('[m]');
ylim([0 14]);
title([subject '| Distribution of path length per waypoint']);

subplot(NumRows, NumCols, [5 6]);
h = bar(PathLength', 'stacked');
grid on;
ylabel('[m]')
xlabel('run');
ylim([0 45]);
title([subject ' | Total path length per run']);
legend([repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)])

subplot(NumRows, NumCols, [3 4 7 8])
hold on;
PMaps = mean(Maps, 3)./max(Maps, [], 3);
PMaps(PMaps == 0) = nan;
h1 = imagesc(map.info.y, map.info.x, PMaps, [0 0.5]); 
set(gca, 'YDir', 'normal', 'XDir', 'reverse'); 
axis image;

mtracking = mtracking  - map.info.origin;
plot(mtracking(:, 2), mtracking(:, 1), 'k', 'LineWidth', 2);

Tmap = double(map.data);
% Tmap(Tmap == 100) = 1;
Tmap(Tmap ~= 100) = nan;
h2 = imagesc(map.info.y, map.info.x, Tmap);
set(h2, 'AlphaData', ~isnan(Tmap));


hold off;
 
grid minor;
xlabel('[m]')
ylabel('[m]');
title([subject ' | HitMap over all trajectories and average']);

fig2 = figure;
fig_set_position(fig2, 'Top');
[NumRows2, NumCols2] = plot_subplot_size(NumRuns, 'maxcols', 5);
for rId = 1:NumRuns
    subplot(NumRows2, NumCols2, rId);  
    cindex = Rk == Runs(rId) & Tk > 0;
    
    cnbibochum_show_map(map.info.x, map.info.y, map.data);
    hold on
    plot(P(cindex, 2) - map.info.origin(2), P(cindex, 1)- map.info.origin(1), 'g', 'LineWidth', 1);
    hold off;
    xlabel('[m]');
    ylabel('[m]');
    title([subject ' | Run ' num2str(Runs(rId))]);
end

%% Saving figure
figname = fullfile(figpath, [subject '_parkour_trajectories.pdf']);
figname2 = fullfile(figpath, [subject '_parkour_full_trajectories.pdf']);
util_disp(['[out] - Saving figure in: ' figname]);
fig_export(fig1, figname, '-pdf', 'landscape', '-fillpage');
util_disp(['[out] - Saving figure in: ' figname]);
fig_export(fig2, figname2, '-pdf', 'landscape', '-fillpage');


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

