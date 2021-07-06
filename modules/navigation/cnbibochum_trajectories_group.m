clearvars; clc; close all;

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


%% Path length
PathLength = [];

for sId = 1:NumSubjects
     
    cruns = unique(Rk(Sk == Subjects(sId)));
    cnruns = length(cruns);
    
    cpathlength = nan(NumWaypoints, cnruns);
    
    for rId = 1:cnruns
        for wId = 1:NumWaypoints
            cindex = Rk == cruns(rId) & Wk == Waypoints(wId) & Tk > 0 & Xk == true & Nk == true & Sk == Subjects(sId);

            if(sum(cindex) == 0)
                continue;
            end

            cpath = P(cindex, :);

            cpathlength(wId, rId) = sum(sqrt(sum((cpath(2:end, :) - cpath(1:end-1, :)).^2, 2)));
        end

    end
    
    PathLength = cat(2, PathLength, cpathlength);
end

%% Hitmaps for trajectories
FieldSize = [map.info.x(end) map.info.y(end)];
MapResolution = 0.1;
MapSize = ceil(FieldSize./MapResolution);

Maps = [];

for sId = 1:NumSubjects
    cruns = unique(Rk(Sk == Subjects(sId)));
    cnruns = length(cruns);
    cmap = nan(MapSize(1), MapSize(2), cnruns);
    for rId = 1:cnruns
       cindex = Rk == cruns(rId) & Tk > 0 & Nk == true & Sk == Subjects(sId);
       cmap(:, :, rId) = cnbibochum_traj2map(P(cindex, :) - map.info.origin, FieldSize, MapResolution);
    end
    
    Maps = cat(3, Maps, cmap);
end

%% Resampled trajectories
% Find max length
maxlength = zeros(NumSubjects, 1);
for sId = 1:NumSubjects
    cruns = unique(Rk(Sk == Subjects(sId)));
    cnruns = length(cruns);
    for rId = 1:cnruns
        cindex = Rk == cruns(rId) & Tk > 0 & Nk == true & Sk == Subjects(sId);
        maxlength(sId) = max(length(Rk(cindex)), maxlength(sId));
    end
end

rtracking = cell(NumSubjects, 1);

for sId = 1:NumSubjects
    cruns = unique(Rk(Sk == Subjects(sId)));
    cnruns = length(cruns);
    
    crtracking = nan(maxlength(sId), 2, cnruns);
    for rId = 1:cnruns
        cindex = Rk == cruns(rId) & Tk > 0 & Nk == true & Sk == Subjects(sId);
        clength = sum(cindex);
        cpath = P(cindex, :);
        crtracking(:, :, rId) =  interp1(1:clength, cpath, linspace(1, clength, maxlength(sId)));
    end
    
    rtracking{sId} = crtracking;
end

%% Average trajectory

mtracking = cell(NumSubjects, 1);

for sId = 1:NumSubjects
    mtracking{sId} = nanmean(rtracking{sId}, 3);
end

maxlength = max(cellfun(@length, mtracking));
itracking = nan(maxlength, 2, NumSubjects);

for sId = 1:NumSubjects
    ctrajectory = mtracking{sId};
    itracking(:, :, sId) = interp1(1:length(ctrajectory), ctrajectory, linspace(1, length(ctrajectory), maxlength));
end

%% Figure 1 | Distribution path length
fig1 = figure;
fig_set_position(fig1, 'All');
NumRows = 6;
NumCols = 2;

subplot(NumRows, NumCols, [3 5 7 9]);
boxcolors = colororder;
boxplot((PathLength)', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
grid on;
xlabel('waypoint');
ylabel('[m]');
title('Distribution of path length per waypoint');

for sId = 1:NumSubjects
    subplot(NumRows, NumCols, [2 4] + 4*(sId-1));
    boxcolors = colororder;
    boxplot((PathLength(:, rSk == Subjects(sId)))', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
    grid on;
    xlabel('waypoint');
    ylabel('[m]');
    ylim([0 14]);
    title(sublist{sId});
    
end

sgtitle('Distribution path length');

%% Figure 2 | Total path length
fig2 = figure;
fig_set_position(fig2, 'All');
NumRows = 6;
NumCols = 2;

meanPathLength = nan(NumWaypoints, NumSubjects);
stdPathLength  = nan(NumWaypoints, NumSubjects);
stePathLength  = nan(NumWaypoints, NumSubjects);
for sId = 1:NumSubjects
   meanPathLength(:, sId) = nanmean(PathLength(:, rSk == Subjects(sId)), 2); 
   stdPathLength(:, sId)  = nanstd(PathLength(:, rSk == Subjects(sId)), [], 2); 
   stePathLength(:, sId)  = nanstd(PathLength(:, rSk == Subjects(sId)), [], 2)./sqrt(sum(rSk == Subjects(sId))); 
end

subplot(NumRows, NumCols, [3 5 7 9]);
h = bar(meanPathLength', 'stack');
grid on;
set(gca, 'XTickLabel', sublist);
ylabel('[m]')
xlabel('subject');
title('Average path length per waypoint');
legend([repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)])

for sId = 1:NumSubjects
    subplot(NumRows, NumCols, [2 4] + 4*(sId-1));
    bar(PathLength(:, rSk == Subjects(sId))', 'stacked');
    grid on;
    ylabel('[m]')
    xlabel('run');
    ylim([0 50]);
    title(sublist{sId});
    legend([repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)])
end

sgtitle('Path length');

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
    cmtracking = mtracking{sId} - map.info.origin;
    plot(cmtracking(:, 2), cmtracking(:, 1), 'color', linecolors(sId, :), 'LineWidth', 1);
end
% citracking = nanmedian(itracking, 3) - map.info.origin;
% plot(citracking(:, 2), citracking(:, 1), 'k', 'LineWidth', 1);

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
legend(sublist, 'fontsize', 6);

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
    
    cmtracking = mtracking{sId} - map.info.origin;
    plot(cmtracking(:, 2), cmtracking(:, 1), 'color', linecolors(sId, :), 'LineWidth', 1);
    
    

    
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

%% Figure 4 - only heat maps
fig4 = figure;

hold on;
PMaps = mean(Maps, 3)./max(Maps, [], 3);
PMaps(PMaps == 0) = nan;
h1 = imagesc(map.info.y, map.info.x, PMaps, [0 0.5]); 
set(gca, 'YDir', 'normal', 'XDir', 'reverse'); 
axis image;

%% Saving figures

if do_save == false
    return
end

figname1 = fullfile(figpath, 'group_parkour_trajectories_pathlength_distribution.pdf');
figname2 = fullfile(figpath, 'group_parkour_trajectories_pathlength_total.pdf');
figname3 = fullfile(figpath, 'group_parkour_trajectories_hitmaps.pdf');

util_disp(['[out] - Saving figure in: ' figname1]);
fig_export(fig1, figname1, '-pdf', 'landscape', '-fillpage');
util_disp(['[out] - Saving figure in: ' figname2]);
fig_export(fig2, figname2, '-pdf', 'landscape', '-fillpage');
util_disp(['[out] - Saving figure in: ' figname3]);
fig_export(fig3, figname3, '-pdf', 'landscape', '-fillpage');

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
