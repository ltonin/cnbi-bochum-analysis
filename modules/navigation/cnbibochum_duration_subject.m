clearvars; clc;

subject = 'RND01';

include = {subject, 'mi', 'mi_bhbf', 'motion'};
excludepat  = {};
depthlevel  = 2;

datapath  = 'analysis/navigation/';
eventpath = 'analysis/navigation/waypoints/';
figpath   = './figures/parkour/';

files = util_getfile3(datapath, '.mat', 'include', include, 'exclude', excludepat, 'level', depthlevel);
NumFiles = length(files);

StartEvent   = 500;
Waypoints    = [501 502 503 504];
NumWaypoints = length(Waypoints);

%% Concatenate files
[motion, map, events, labels] = cnbibochum_concatenate_motion_data(files, eventpath);
nsamples = length(motion.T);
Wk = proc_get_event3(Waypoints, motion.T, events.POS, events.TYP, events.DUR);
Tk = proc_get_event3(500, motion.T, events.POS, events.TYP, events.DUR);
P = motion.P;
T = motion.T;
Vx = motion.Vx;
Rk = labels.Rk;
Dk = labels.Dk;

Runs = unique(Rk);
NumRuns = length(Runs);
Days = unique(Dk);
NumDays = length(Days);

DurPerWp = nan(NumWaypoints, NumRuns);

for rId = 1:NumRuns
    for wId = 1:NumWaypoints
        cindex = Rk == Runs(rId) & Wk == Waypoints(wId);
        cstart = find(cindex, 1, 'first');
        cstop  = find(cindex, 1, 'last');
        if (isequal(cstart, cstop))
            cdur = nan;
        else
            cdur = T(cstop) - T(cstart);
        end
        DurPerWp(wId, rId) = cdur;
    end
end

% Th = 0.2;
Th = 0.5;
HoldPerRun = nan(NumRuns, 1);
HoldPerRunIdx = cell(NumRuns, 1);
for rId = 1:NumRuns
    cindex = Rk == Runs(rId) & Tk == 500;
    cvx = Vx(cindex);
    cvx(cvx < 0) = nan;
    nvx = cvx./max(cvx);
    
    hIdx = find(nvx < Th);
    ctime = T(hIdx+1) - T(hIdx);
    ttime = T(find(cindex, 1, 'last')) - T(find(cindex, 1, 'first'));
%     HoldPerRun(rId) = sum(nvx < Th)./length(nvx);
    HoldPerRun(rId) = 100*nansum(ctime)./ttime;
    HoldPerRunIdx{rId} = hIdx;
    
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
ylim([30 150]);
title([subject '| Distribution of path duration per waypoint']);

subplot(NumRows, NumCols, [5 6]);
bar(DurPerWp', 'stack');
ylim([0 410]);
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
ylim([0 20]);
for dId = 1:length(daychanges)
    plot_vline(Rk(daychanges(dId))-0.5, 'k');
end
xlim([1 NumRuns]);
grid on;
xlabel('run');
ylabel('[%]');

title([subject ' | Time with reduced speed (vx<' num2str(100*Th) '%)']);

subplot(NumRows, NumCols, [3 4 7 8 11 12])
cnbibochum_show_map(map.info.x, map.info.y, map.data);
for rId = 1:NumRuns
    hold on;
    cpoint = P(HoldPerRunIdx{rId}, :) - (map.info.origin);
    plot(cpoint(:, 2), cpoint(:, 1), 'o');
    hold off;
end
title([subject ' | Positions with reduced speed (vx<' num2str(100*Th) '%)']);
legend([repmat('run ', NumRuns, 1) num2str((1:NumRuns)')]);


%% Saving figure
figname = fullfile(figpath, [subject '_parkour_duration.pdf']);
util_disp(['[out] - Saving figure in: ' figname]);
fig_export(fig1, figname, '-pdf', 'landscape', '-fillpage');


%% %%%%%%%%%%%%%%%%%%%%% Functions %%%%%%%%%%%%%%%%%%%%%%%%
function eventstr = cnbibochum_util_import_event(reffilename, rootpath, subfolder)

    file_identifier = regexp(reffilename, '(\w*\.\d*\.\d*)\.\w*', 'tokens');
    evtfile = util_getfile3(rootpath, '.mat', 'include', [file_identifier{1} subfolder], 'level', 3);
    
    if length(evtfile) > 1
        error(['Cannot find unique event file in: ' fullfile(rootpath, subfolder)]);
    elseif isempty(evtfile)
        error(['Cannot find any event file in: ' fullfile(rootpath, subfolder)]);
    end
    
    cevents = load(evtfile{1});
    
    eventstr = cevents.event;
end
