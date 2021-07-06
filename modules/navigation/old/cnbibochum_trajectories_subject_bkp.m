clearvars; clc;

subject = 'BOCH04';

include = {subject, 'mi', 'mi_bhbf', 'motion'};
excludepat  = {};
depthlevel  = 2;

datapath  = 'analysis/navigation/';
eventpath = 'analysis/navigation/waypoints/';
figpath   = './figures/parkour/';

files = util_getfile3(datapath, '.mat', 'include', include, 'exclude', excludepat, 'level', depthlevel);
NumFiles = length(files);

StartEvent    = 500;
Waypoints     = [501 502 503 504];
NumWaypoints  = length(Waypoints);
CommandEvents = [26117 26118];
CommandLabels = {'left', 'right'};

%% Concatenate files
[motion, map, events, labels] = cnbibochum_concatenate_motion_data(files, eventpath);
nsamples = length(motion.T);
Wk = proc_get_event3(Waypoints, motion.T, events.POS, events.TYP, events.DUR);
Tk = proc_get_event3(StartEvent, motion.T, events.POS, events.TYP, events.DUR);
Ck = proc_get_event3(CommandEvents, motion.T, events.POS, events.TYP, events.DUR);
P = motion.P;
T = motion.T;
Vx = motion.Vx;
Rk = labels.Rk;
Dk = labels.Dk;
map = map(1);
Ck = filter_commands(T, Ck, 1.5);
Xk = cnbibochum_get_data_validity(subject, Rk, Wk);

Runs = unique(Rk);
NumRuns = length(Runs);
Days = unique(Dk);
NumDays = length(Days);

if strcmp(subject, 'SIM01')
    for rId = 1:NumRuns
        cindex = Rk == Runs(rId);
        cp= P(cindex, 1);
        cp(end-1500:end, 1) = cp(end-1500:end, 1) - 0.5;
        P(cindex, 1) = cp;
    end
end

% Compute length of the path
PathLength = nan(NumWaypoints, NumRuns);

for rId = 1:NumRuns
    for wId = 1:NumWaypoints
        cindex = Rk == Runs(rId) & Wk == Waypoints(wId) & Tk > 0 & Xk == true;
        
        if(sum(cindex) == 0)
            continue;
        end
        
        cpath = P(cindex, :);

        PathLength(wId, rId) = sum(sqrt(sum((cpath(2:end, :) - cpath(1:end-1, :)).^2, 2)));
    end
    
end
   

% Compute map trajectories
FieldSize = [map.info.x(end) map.info.y(end)];
MapResolution = 0.1;
MapSize = ceil(FieldSize./MapResolution);

Maps = nan(MapSize(1), MapSize(2), NumRuns);

for rId = 1:NumRuns
   cindex = Rk == Runs(rId) & Tk > 0;
   Maps(:, :, rId) = cnbibochum_traj2map(P(cindex, :) - map.info.origin, FieldSize, MapResolution);
end


% Compute average trajectory

% Find max length
maxlength = 0;
for rId = 1:NumRuns
    cindex = Rk == Runs(rId) & Tk > 0;
    maxlength = max(length(Rk(cindex)), maxlength);
end

rtracking = nan(maxlength, 2, NumRuns);
for rId = 1:NumRuns
    cindex = Rk == Runs(rId) & Tk > 0;
    clength = sum(cindex);
    cpath = P(cindex, :);
    rtracking(:, :, rId) =  interp1(1:clength, cpath, linspace(1, clength, maxlength));
end

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
xlabel('waypoint');
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

mtracking = nanmean(rtracking, 3)  - map.info.origin;
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

% NumRows = 3;
% NumCols = 4;
% 
% subplot(NumRows, NumCols, [1 2]);
% boxcolors = colororder;
% boxplot(squeeze(sum(CommandPerRuns))', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :));
% grid on;
% xlabel('waypoint');
% ylabel('[#]');
% title([subject '| Distribution of number of commands per waypoint']);
% 
% subplot(NumRows, NumCols, [5 6]);
% h = bar(squeeze(mean(CommandPerRuns, 3))', 'grouped');
% hold on;
% x = [h(1).XEndPoints; h(2).XEndPoints];
% m = squeeze(mean(CommandPerRuns, 3))';
% e = squeeze(std(CommandPerRuns, [], 3)./sqrt(NumRuns))';
% errorbar(x(1, :), m(:, 1), e(:, 1), '.k');
% errorbar(x(2, :), m(:, 2), e(:, 2), '.k');
% hold off;
% set(gca, 'XTickLabel', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)]);
% grid on;
% ylabel('[#]')
% xlabel('waypoint');
% title([subject ' | Average number of commands per waypoint']);
% legend(CommandLabels, 'location', 'northwest')
% 
% subplot(NumRows, NumCols, [9 10]);
% bar(squeeze(sum(CommandPerRuns, 2))', 'stack');
% daychanges = find(diff(Dk))+1;
% for dId = 1:length(daychanges)
%     plot_vline(Rk(daychanges(dId))-0.5, 'k');
% end
% grid on;
% ylabel('[#]')
% xlabel('run');
% title([subject ' | Total number of commands']);
% legend(CommandLabels)
% 
% subplot(NumRows, NumCols, [3 4 7 8 11 12])
% cnbibochum_show_map(map.info.x, map.info.y, map.data);
% for rId = 1:NumRuns
%     for cId = 1:NumCommands
%         cIdx = find(Ck == Commands(cId) & Rk == Runs(rId));
%         hold on;
%         cpoint = P(cIdx, :) - (map.info.origin);
%         plot(cpoint(:, 2), cpoint(:, 1), '.', 'color', boxcolors(cId, :));
%         hold off;
%     end
% end
% title([subject ' | Commands for all the runs (N=' num2str(NumRuns) ')']);
% legend(CommandLabels)

%% Functions

function nCk = filter_commands(T, Ck, timetolerance)

    POS = find(Ck == 26117 | Ck == 26118);
    TYP = Ck(Ck == 26117 | Ck == 26118);
    TIM = T(POS);
    
    MSK = [true; diff(TIM) > timetolerance | diff(TYP) ~=0];

    nCk = Ck;
    nCk(POS(~MSK)) = 0;
    
end



