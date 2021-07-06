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
Vz = motion.Vz;
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
Vz_m = motion_m.Vz;
Rk_m = labels_m.Rk;

% Get data information 
Runs = unique(Rk);
NumRuns = length(Runs);
Days = unique(Dk);
NumDays = length(Days);

Runs_m = unique(Rk_m);
NumRuns_m = length(Runs_m);


%% Figure check
fig1 = figure;
fig_set_position(fig1, 'All');


% for rId = 1:NumRuns
%     subplot(2, 5, rId);
%     cindex = Rk == Runs(rId) & Tk > 0;
%     ctime = T(cindex) - T(find(cindex, 1, 'first'));
%     
%     cvel = Vz(cindex);
%     ncvel = (cvel - mean(cvel))./std(cvel);
%     
%     cindex_m = Rk_m == Runs(rId) & Tk_m > 0;
%     ctime_m = T_m(cindex_m) - T_m(find(cindex_m, 1, 'first'));
%     
%     cvel_m = Vz_m(cindex_m);
%     ncvel_m = (cvel_m - mean(cvel_m))./std(cvel_m);
%     
%     rough = compute_roghness(cvel);
%     rough_m = compute_roghness(cvel_m);
%     
%     hold on;
%     plot(ctime(3:end), rough, '-');
%     plot(ctime_m(3:end), rough_m);
% %     
% %     plot(ctime(2:end), diff(ncvel).^2/4, '-');
% %     plot(ctime_m(2:end), diff(ncvel_m).^2/4);
%     hold off;
%     
% %     title([num2str(sum(rough) ./ sum(rough_m))]);
% end

rP_m = reshape_wp_trajectories(P_m, Wk_m, Rk_m, Tk_m);


Diff1 = cell(NumWaypoints, 1);
Diff2 = cell(NumWaypoints, 1);
Diff1_m = cell(NumWaypoints, 1);
Diff2_m = cell(NumWaypoints, 1);
for wId = 1:NumWaypoints
    
    subplot(1, NumWaypoints, wId);
    
    hold on;
    csamples = nan(NumRuns, 1);
    tcdiff = [];
    tcdiff2 = [];
    for rId = 1:NumRuns
        
        cindex = Wk == Waypoints(wId) & Rk == Runs(rId);

        if (sum(cindex) <= 1)
            continue;
        end
        
%         cdiff = diff(normalize(P(cindex, :)));
        cdiff = normalize(diff(P(cindex, :)));
%         ncdiff = diff( normalize(cdiff) );
        ncdiff = normalize(diff(cdiff));
        
        tcdiff  = cat(1, tcdiff, cdiff);
        tcdiff2 =cat(1, tcdiff2, ncdiff);
        
        %plot(ncdiff(:, 2), ncdiff(:, 1), '.b');
        
        csamples(rId) = sum(cindex);
    end
    Diff1{wId} = tcdiff;
    Diff2{wId} = tcdiff2;
    
    cP_m = mean(rP_m{wId}, 3);
    cratio = round(length(cP_m)./nanmean(csamples));
    if(cratio == 0)
        cratio = 1;
    end
    dwnP_m   = downsample(cP_m, cratio);
%     cdiff_m  = diff(normalize(dwnP_m));
    cdiff_m  = normalize(diff(dwnP_m));
%     ncdiff_m = diff( normalize(cdiff_m) );
    ncdiff_m = normalize(diff(cdiff_m));

    
    Diff1_m{wId} = cdiff_m;
    Diff2_m{wId} = ncdiff_m;
    %plot(ncdiff_m(:, 2), ncdiff_m(:, 1), '.r');
    
   
end

for wId = 1:NumWaypoints
   subplot(2, 4, wId);
   hold on;
   plot(Diff1{wId}(:, 2), Diff1{wId}(:, 1), '.b');
   plot(Diff1_m{wId}(:, 2), Diff1_m{wId}(:, 1), '.r');
   hold off;
   xlim([-0.2 0.2]);
   ylim([-0.2 0.2]);
   axis square;
   grid on;
   xlabel('x-dim');
   ylabel('y-dim');
   title(['Waypoint ' num2str(wId) ' | normalized diff']);
   
   subplot(2, 4, wId + 4);
   hold on;
   plot(Diff2{wId}(:, 2), Diff2{wId}(:, 1), '.b');
   plot(Diff2_m{wId}(:, 2), Diff2_m{wId}(:, 1), '.r');
   xlim([-0.4 0.4]);
   ylim([-0.4 0.4]);
   hold off;
   axis square;
   grid on;
   xlabel('x-dim');
   ylabel('y-dim');
   title(['Waypoint ' num2str(wId) ' | normilized diff2']);
end

sgtitle([subject ' | Smoothness (?)']);

%% Saving figure
figname = fullfile(figpath, [subject '_parkour_smoothness.pdf']);
util_disp(['[out] - Saving figure in: ' figname]);
fig_export(fig1, figname, '-pdf', 'landscape', '-fillpage');

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

function R = compute_roghness(x)
    
    diffX = diff(x);
    ndiffX = (diffX - mean(diffX))./std(diffX);
    R = (diff(ndiffX).^2)./4;
end


