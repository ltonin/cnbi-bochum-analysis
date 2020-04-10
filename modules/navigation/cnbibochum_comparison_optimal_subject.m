clearvars; clc;

subject = 'BOCH02';
motion_m  = 'SIM01';

include = {'mi', 'mi_bhbf', 'motion'};
excludepat  = {};
depthlevel  = 2;

datapath  = 'analysis/navigation/';
eventpath = 'analysis/navigation/waypoints/';
figpath   = './figures/parkour/';

files_subject = util_getfile3(datapath, '.mat', 'include', [include subject], 'exclude', excludepat, 'level', depthlevel);
files_manual  = util_getfile3(datapath, '.mat', 'include', [include motion_m],  'exclude', excludepat, 'level', depthlevel);
NumFiles = length(files_subject);

StartEvent    = 500;
Waypoints     = [501 502 503 504];
NumWaypoints  = length(Waypoints);
CommandEvents = [26117 26118];
CommandLabels = {'left', 'right'};

%% Concatenate files
[motion,   map,  events,   labels]   = cnbibochum_concatenate_motion_data(files_subject, eventpath);
[motion_m,   ~,  events_m, labels_m] = cnbibochum_concatenate_motion_data(files_manual,  eventpath);
nsamples  = length(motion.T);

% Subject
Wk = proc_get_event3(Waypoints, motion.T, events.POS, events.TYP, events.DUR);
Tk = proc_get_event3(StartEvent, motion.T, events.POS, events.TYP, events.DUR);
Ck = proc_get_event3(CommandEvents, motion.T, events.POS, events.TYP, events.DUR);
P  = motion.P;
T  = motion.T;
Vx = motion.Vx;
Rk = labels.Rk;
Dk = labels.Dk;
Ck = filter_commands(T, Ck, 1.5);

% Manual
Wk_m = proc_get_event3(Waypoints,     motion_m.T, events_m.POS, events_m.TYP, events_m.DUR);
Tk_m = proc_get_event3(StartEvent,    motion_m.T, events_m.POS, events_m.TYP, events_m.DUR);
Ck_m = proc_get_event3(CommandEvents, motion_m.T, events_m.POS, events_m.TYP, events_m.DUR);
P_m  = motion_m.P;
T_m  = motion_m.T;
Vx_m = motion_m.Vx;
Rk_m = labels_m.Rk;

% Get data information 
Runs = unique(Rk);
NumRuns = length(Runs);
Days = unique(Dk);
NumDays = length(Days);

Runs_m = unique(Rk_m);
NumRuns_m = length(Runs_m);

% Manual corrections for manual runs
P_mo = P_m;
for rId = 1:NumRuns_m
    cindex = Rk_m == Runs_m(rId);
    cp= P_m(cindex, 1);
    cp(end-1500:end, 1) = cp(end-1500:end, 1) - 0.5;
    P_m(cindex, 1) = cp;
end

%% Compute correction of duration of simulation runs (based on velocity)
L = compute_path_length(P, Rk, Tk);
D = compute_path_duration(T, Rk, Tk);
V = compute_path_velocity(Vx, Rk, Tk);

L_m = compute_path_length(P_m, Rk_m, Tk_m);
D_m = compute_path_duration(T_m, Rk_m, Tk_m);
V_m = compute_path_velocity(Vx_m, Rk_m, Tk_m);

WcVx     = L./D;
CmdVelVx = V;

WcVxSim     = L_m./D_m;
CmdVelVxSim = V_m;

RatioVx = mean(WcVx./mean(WcVxSim));

%% Compute waypoint duration
DurationWp = compute_waypoint_duration(T, Wk, Rk, Tk);
DurationWp_raw = compute_waypoint_duration(T_m, Wk_m, Rk_m, Tk_m);

% Correction duration for simulation
DurationWp_m = DurationWp_raw - DurationWp_raw.*(RatioVx - 1);

%% Compute waypoint commands
Commands   = compute_waypoint_commands(Ck, Wk, Rk);
Commands_m = compute_waypoint_commands(Ck_m, Wk_m, Rk_m);

%% Figure

fig1 = figure;
fig_set_position(fig1, 'All');
NumRows = 2;
NumCols = 2;

subplot(NumRows, NumCols, 1);
boxcolors = colororder;
boxplot(100*(DurationWp./DurationWp_m)', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
grid on;
xlabel('waypoint');
plot_hline(100, 'k--');
ylabel('[%]');
title([subject '| Distribution of duration ratios: 100*(BCI/Manual)']);

subplot(NumRows, NumCols, 3);
bar(100*(DurationWp./DurationWp_m)', 'grouped');
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
boxplot(100*squeeze(nansum(Commands)./nansum(Commands_m))', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :));
grid on;
plot_hline(100, 'k--');
xlabel('waypoint');
ylabel('[#]');
title([subject '| Distribution of command ratios: 100*(BCI/Manual)']);

subplot(NumRows, NumCols, 4);
h = bar(100*squeeze(nansum(Commands)./nansum(Commands_m))', 'grouped');
% plot(100*squeeze(sum(Commands)./sum(Commands_m))', 'o-');
plot_hline(100, 'k--');
set(gca, 'XTickLabel', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)]);
grid on;
ylabel('[#]')
xlabel('waypoint');
title([subject ' | Ratio BCI/manual number commands per run']);
legend([repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)])

%% Saving figure
figname = fullfile(figpath, [subject '_parkour_comparison_manual.pdf']);
util_disp(['[out] - Saving figure in: ' figname]);
fig_export(fig1, figname, '-pdf', 'landscape', '-fillpage');

% %% Compute reference trajectory
% Tj_m    = reshape_full_trajectories(P_m, Rk_m, Tk_m);
% Tj_ref  = nanmean(Tj_m, 3);
% Twj_m   = reshape_wp_trajectories(P_m, Wk_m, Rk_m, Tk_m);
% Twj_ref = cellfun(@(m) mean(m, 3), Twj_m, 'UniformOutput', false);
% 
% 
% %% Compute Frechet distance for each waypoint
% WpFrechet = nan(NumWaypoints, NumRuns);
% for wId = 1:NumWaypoints
%     for rId = 1:NumRuns
%         cindex = Rk == Runs(rId) & Tk > 0 & Wk == Waypoints(wId);
%         if sum(cindex) <= 1
%             continue;
%         end
%         cpath = P(cindex, :);
%         rpath = Twj_ref{wId};
%         WpFrechet(wId, rId) = proc_frechet_distance(cpath, rpath);
%     
%     end
% end
% 
% FullFrechet = max(WpFrechet)';
% 
% %% Compute image map trajectories
% FieldSize = [map.info.x(end) map.info.y(end)];
% MapResolution = 0.1;
% MapSize = ceil(FieldSize./MapResolution);
% 
% Maps = nan(MapSize(1), MapSize(2), NumRuns);
% 
% for rId = 1:NumRuns
%    cindex = Rk == Runs(rId) & Tk > 0;
%    Maps(:, :, rId) = cnbibochum_traj2map(P(cindex, :) - map.info.origin, FieldSize, MapResolution);
% end
% 
% % Compute average trajectory
% Tj     = reshape_full_trajectories(P, Rk, Tk);
% Tj_avg = nanmean(Tj, 3);
% 
% 
% %% Figure
% 
% fig1 = figure;
% fig_set_position(fig1, 'All');
% 
% NumRows = 2;
% NumCols = 4;
% 
% subplot(NumRows, NumCols, [1 2]);
% boxcolors = colororder;
% boxplot((WpFrechet)', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
% grid on;
% xlabel('waypoint');
% ylabel('[m]');
% title([subject '| Frechet distance per waypoint']);
% 
% subplot(NumRows, NumCols, [5 6]);
% plot(WpFrechet', '-o');
% daychanges = find(diff(Dk))+1;
% for dId = 1:length(daychanges)
%     plot_vline(Rk(daychanges(dId))-0.5, 'k');
% end
% grid on;
% ylabel('[m]')
% xlabel('run');
% title([subject ' | Frechet distance per run and waypoint']);
% legend([repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)])
% 
% subplot(NumRows, NumCols, [3 4 7 8])
% hold on;
% PMaps = mean(Maps, 3)./max(Maps, [], 3);
% PMaps(PMaps == 0) = nan;
% h1 = imagesc(map.info.y, map.info.x, PMaps, [0 0.5]); 
% set(gca, 'YDir', 'normal', 'XDir', 'reverse'); 
% axis image;
% 
% Tmap = double(map.data);
% Tmap(Tmap ~= 100) = nan;
% h2 = imagesc(map.info.y, map.info.x, Tmap);
% set(h2, 'AlphaData', ~isnan(Tmap));
% 
% mTj = Tj_avg - map.info.origin;
% plot(mTj(:, 2), mTj(:, 1), 'k', 'LineWidth', 2);
% mTj_m = Tj_ref - map.info.origin;
% plot(mTj_m(:, 2), mTj_m(:, 1), 'color', boxcolors(2, :), 'LineWidth', 2);
% 
% hold off;
%  
% grid minor;
% xlabel('[m]')
% ylabel('[m]');
% title([subject ' | HitMap over all trajectories and average']);
% legend({'Average trajectory', 'Optimal trajectory'}, 'location', 'NorthEastOutside');
% 
% % subplot(NumRows, NumCols, [3 4 7 8])
% % hold on;
% % PMaps = mean(Maps, 3)./max(Maps, [], 3);
% % PMaps(PMaps == 0) = nan;
% % h1 = imagesc(map.info.y, map.info.x, PMaps, [0 0.5]); 
% % set(gca, 'YDir', 'normal', 'XDir', 'reverse'); 
% % axis image;
% % 
% % mtracking = nanmean(rtracking, 3) - map.info.origin;
% % plot(mtracking(:, 2), mtracking(:, 1), 'k', 'LineWidth', 2);
% % 
% % Tmap = double(map.data);
% % % Tmap(Tmap == 100) = 1;
% % Tmap(Tmap ~= 100) = nan;
% % h2 = imagesc(map.info.y, map.info.x, Tmap);
% % set(h2, 'AlphaData', ~isnan(Tmap));
% % 
% % 
% % hold off;
% %  
% % grid minor;
% % xlabel('[m]')
% % ylabel('[m]');
% % title([subject ' | HitMap over all trajectories and average']);


%% Functions

function nCk = filter_commands(T, Ck, timetolerance)

    POS = find(Ck == 26117 | Ck == 26118);
    TYP = Ck(Ck == 26117 | Ck == 26118);
    TIM = T(POS);
    
    MSK = [true; diff(TIM) > timetolerance | diff(TYP) ~=0];

    nCk = Ck;
    nCk(POS(~MSK)) = 0;
    
end

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

function L = compute_path_length(P, Rk, Tk)
    Runs    = unique(Rk);
    NumRuns = length(Runs);
    L = nan(NumRuns, 1);

    for rId = 1:NumRuns
        cindex = Rk == Runs(rId) & Tk > 0;
        cpath = P(cindex, :);

        L(rId) = sum(sqrt(sum((cpath(2:end, :) - cpath(1:end-1, :)).^2, 2)));
    end
end

function D = compute_path_duration(T, Rk, Tk)
    
    Runs    = unique(Rk);
    NumRuns = length(Runs);
    D = nan(NumRuns, 1);

    for rId = 1:NumRuns
        cindex = Rk == Runs(rId) & Tk > 0;
        ctime = T(cindex);

        D(rId) = ctime(end) - ctime(1);
    end
end

function D = compute_waypoint_duration(T, Wk, Rk, Tk)
    
    Waypoints    = setdiff(unique(Wk), 0);
    NumWaypoints = length(Waypoints);
    Runs         = unique(Rk);
    NumRuns      = length(Runs);
    
    D = nan(NumWaypoints, NumRuns);

    for rId = 1:NumRuns
        for wId = 1:NumWaypoints
            cindex = Rk == Runs(rId) & Wk == Waypoints(wId) & Tk > 0;
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

function V = compute_path_velocity(Vx, Rk, Tk)
    
    Runs    = unique(Rk);
    NumRuns = length(Runs);
    V = nan(NumRuns, 1);

    for rId = 1:NumRuns
        cindex = Rk == Runs(rId) & Tk > 0;
        cvelocities = Vx(cindex);

        V(rId) = mean(cvelocities);
    end
end

function C = compute_waypoint_commands(Ck, Wk, Rk)
    Runs = unique(Rk);
    NumRuns = length(Runs);
    Commands = setdiff(unique(Ck), 0);
    NumCommands = length(Commands);
    Waypoints    = setdiff(unique(Wk), 0);
    NumWaypoints = length(Waypoints);

    C = nan(NumCommands, NumWaypoints, NumRuns);
    for rId = 1:NumRuns
        for wId = 1:NumWaypoints
            cindex = Rk == Runs(rId) & Wk == Waypoints(wId);
            if sum(cindex) <= 1
                continue;
            end
            for cId = 1:NumCommands
               C(cId, wId, rId) = sum(cindex & Ck == Commands(cId));
            end
        end
    end
end

