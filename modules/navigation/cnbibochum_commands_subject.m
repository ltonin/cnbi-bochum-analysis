clearvars; clc;

subject = 'BOCH04';

include = {subject, 'mi', 'mi_bhbf', 'processed'};
excludepat  = {};
depthlevel  = 3;

datapath  = 'analysis/navigation/';
eventpath = 'analysis/navigation/new/waypoints/';
figpath   = './figures/parkour/new/';

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
Ck = proc_get_event3(CommandEvents, nav.T, events.POS, events.TYP, events.DUR);
P = nav.P;
T = nav.T;
Vx = nav.Vx;
Rk = labels.Rk;
Dk = labels.Dk;
map = map(1);
Xk = cnbibochum_get_data_validity(subject, Rk, Wk);
Nk  = support_get_navigation_labels(events, Rk);

Ck = support_filter_commands(T, Ck, 1.5);

Runs = unique(Rk);
NumRuns = length(Runs);
Days = unique(Dk);
NumDays = length(Days);
Commands = setdiff(unique(Ck), 0);
NumCommands = length(Commands);

%% Commands per run and waypoint

CommandPerRuns = nan(NumCommands, NumWaypoints, NumRuns);
for rId = 1:NumRuns
    for wId = 1:NumWaypoints
        cindex = Rk == Runs(rId) & Wk == Waypoints(wId) & Xk == true & Nk == true;
        
        if(sum(cindex) == 0)
            continue;
        end
        
        for cId = 1:NumCommands
           CommandPerRuns(cId, wId, rId) = nansum(cindex & Ck == Commands(cId));
        end
    end
end


%% Figure

fig1 = figure;
fig_set_position(fig1, 'All');
NumRows = 3;
NumCols = 4;

subplot(NumRows, NumCols, [1 2]);
boxcolors = colororder;
boxplot(squeeze(sum(CommandPerRuns))', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :));
ylim([0 60]);
grid on;
xlabel('waypoint');
ylabel('[#]');
title([subject '| Distribution of number of commands per waypoint']);

subplot(NumRows, NumCols, [5 6]);
h = bar(squeeze(nanmean(CommandPerRuns, 3))', 'grouped');
ylim([0 25]);
hold on;
x = [h(1).XEndPoints; h(2).XEndPoints];
m = squeeze(nanmean(CommandPerRuns, 3))';
e = squeeze(nanstd(CommandPerRuns, [], 3)./sqrt(NumRuns))';
errorbar(x(1, :), m(:, 1), e(:, 1), '.k');
errorbar(x(2, :), m(:, 2), e(:, 2), '.k');
hold off;
set(gca, 'XTickLabel', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)]);
grid on;
ylabel('[#]')
xlabel('waypoint');
title([subject ' | Average number of commands per waypoint']);
legend(CommandLabels, 'location', 'northwest')

subplot(NumRows, NumCols, [9 10]);
bar(squeeze(nansum(CommandPerRuns, 2))', 'stack');
ylim([0 100]);
daychanges = find(diff(Dk))+1;
for dId = 1:length(daychanges)
    plot_vline(Rk(daychanges(dId))-0.5, 'k');
end
grid on;
ylabel('[#]')
xlabel('run');
title([subject ' | Number of commands per run']);
legend(CommandLabels)

subplot(NumRows, NumCols, [3 4 7 8 11 12])
cnbibochum_show_map(map.info.x, map.info.y, map.data);
for rId = 1:NumRuns
    for cId = 1:NumCommands
        cIdx = find(Ck == Commands(cId) & Rk == Runs(rId));
        hold on;
        cpoint = P(cIdx, :) - (map.info.origin);
        plot(cpoint(:, 2), cpoint(:, 1), '.', 'color', boxcolors(cId, :));
        hold off;
    end
end
title([subject ' | Commands for all the runs (N=' num2str(NumRuns) ')']);
legend(CommandLabels)

%% Saving figure
figname = fullfile(figpath, [subject '_parkour_commands.pdf']);
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


