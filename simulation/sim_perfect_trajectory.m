clearvars; clc;

subject = 'PERFECT';

includepat  = {subject, 'simulation', 'sharedcontrol'};
excludepat  = {};
depthlevel  = 3;

rootpath    = 'analysis/simulation/';
savedir     = 'analysis/simulation/';
figpath     = 'figures/simulation/';

StartEvent = 800;
WayPointEvent = [801 802 803 804];
WpPosition(:, 1) = [-0.771 -1.479];
WpPosition(:, 2) = [ 0.256  2.733];
WpPosition(:, 3) = [-0.771 -1.479];
WpPosition(:, 4) = [-2.012 -5.655];
CommandEvent = [26117 26118];

%% Get datafiles
files = util_getfile3(rootpath, '.mat', 'include', includepat, 'exclude', excludepat, 'level', depthlevel);

nfiles = length(files);
if(nfiles > 0)
    util_bdisp(['[io] - Found ' num2str(nfiles) ' files with the inclusion/exclusion criteria: (' strjoin(includepat, ', ') ') / (' strjoin(excludepat, ', ') '), depth: ' num2str(depthlevel)]);
else
    error(['[io] - No files found with the inclusion/exclusion criteria: (' strjoin(includepat, ', ') ') / (' strjoin(excludepat, ', ') '), depth: ' num2str(depthlevel)]);
end

% Create analysis directory
util_mkdir('./', savedir);

%% Concatenate data
util_bdisp(['[io] - Importing ' num2str(nfiles) ' files from ' rootpath ':']);
[data, events, labels, settings] = sim_concatenate_data(files);

nsamples = length(data.t);

%% Create label vectors
RacK = proc_get_event2(StartEvent, nsamples, events.POS, events.TYP, events.DUR);
WayK = sim_get_waypoint_label(WayPointEvent, nsamples, events.POS, events.TYP, events.DUR);
Rk = labels.Rk;
Ck = labels.Ck;
nwaypoints = length(WayPointEvent);

%% Reshaping trajectory to have the same length
rtrajectories = reshape_trajectories(data.pose, Rk);
mtrajectory   = mean(rtrajectories, 3);
[distance, index] = sim_find_minimum_distance(mtrajectory(:, 1), mtrajectory(:, 2), WpPosition);

mWayK = zeros(length(mtrajectory), 1);
cstart = 1;
for wId = 1:length(index)
    cstop = index(wId);
    mWayK(cstart:cstop) = WayPointEvent(wId);
    cstart = index(wId);
end

%% Saving perfect trajectory
sfilename = [savedir '/PERFECT_trajectory.mat'];
util_bdisp(['[out] - Saving PERFECT trajectory data in: ' sfilename]);
save(sfilename, 'mtrajectory', 'rtrajectories', 'mWayK'); 

%% Figure
fig1 = figure;
WpColors = {'r', 'g', 'y', 'c'};
hold on;

for wId = 1:nwaypoints
    cindex = mWayK == WayPointEvent(wId);
    plot(mtrajectory(cindex, 1), mtrajectory(cindex, 2), 'Color', WpColors{wId});
end

for wId = 1:nwaypoints
    cpos = index(wId);
    plot(mtrajectory(cpos, 1), mtrajectory(cpos, 2), 'o', 'Color', WpColors{wId});
end
hold off;

axis equal;
grid on;



%% Function
function trajectories = reshape_trajectories(pose, Rk)

    runs  = unique(Rk);
    nruns = length(runs);
    
    % Find max length
    maxlen = 0;
    for rId = 1:nruns
        cindex = Rk == runs(rId);
        maxlen = max(length(Rk(cindex)), maxlen);
    end

	trajectories = nan(maxlen, 2, nruns);
    for rId = 1:nruns
        cindex = Rk == runs(rId);
        clength = sum(cindex);
        ctrajectory = pose(cindex, 1:2);
        trajectories(:, :, rId) =  interp1(1:clength, ctrajectory, linspace(1, clength, maxlen));
    end
end

function evtvec = sim_get_waypoint_label(evt, nsamples, POS, TYP, DUR)

    % Compute the selected event index
    evtId = false(length(POS), 1);
    for eId = 1:length(evt)
        evtId = evtId | TYP == evt(eId);
    end

    % Extract the selected events
    pos_s    = POS(evtId);
    typ_s    = TYP(evtId);
    dur_s    = DUR(evtId);
    nsevents = length(pos_s);

    % Create label vector for the selected events
    evtvec     = zeros(nsamples, 1); 

    for eId = 1:nsevents
        cpos = pos_s(eId);
        ctyp = typ_s(eId);
        cdur = dur_s(eId);
        
        cstop = cpos;
        cstart = cpos - cdur + 1;
      
        evtvec(cstart:cstop) = ctyp;
       
    end

    evtvec = evtvec(1:nsamples);
end