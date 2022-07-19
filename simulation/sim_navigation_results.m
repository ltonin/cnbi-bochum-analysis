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

%% Loading perfect trajectory
refdata = load([savedir '/PERFECT_trajectory.mat']);

%% Concatenate data
util_bdisp(['[io] - Importing ' num2str(nfiles) ' files from ' rootpath ':']);
[data, events, labels, settings] = sim_concatenate_data(files);

nsamples = length(data.t);

%% Create label vectors
util_bdisp('[proc] - Creating label vectors');
RacK = proc_get_event2(StartEvent, nsamples, events.POS, events.TYP, events.DUR);
WayK = sim_get_waypoint_label(WayPointEvent, nsamples, events.POS, events.TYP, events.DUR);

% Add additional events (WayPoint)
events = add_waypoint_event(WayK, events);

Rk = labels.Rk;
Ck = labels.Ck;

runs  = unique(Rk);
nruns = length(runs);
nwaypoints = length(WayPointEvent);

RunControlK = zeros(nruns, 1);

for rId = 1:nruns
    cindex = Rk == runs(rId);
    RunControlK(rId) = unique(Ck(cindex));
end

%% Computing performances
util_bdisp('[proc] - Computing performances');
performances = false(nwaypoints, nruns);
for rId = 1:nruns
    for wId = 1:nwaypoints
        crunid = runs(rId);
        cwypid = WayPointEvent(wId);
        cindex = events.TRK == crunid;

        creachedwp = find(events.TYP(cindex) == cwypid);

        if isempty(creachedwp) == false
            performances(wId, rId) = true;
        end
    end
end

%% Computing duration
util_bdisp('[proc] - Computing durations');
durations = nan(nwaypoints, nruns);
for rId = 1:nruns
    for wId = 1:nwaypoints
        crunid = runs(rId);
        cwypid = WayPointEvent(wId);
        cindex = events.TRK == crunid & events.TYP == cwypid;
        
        if sum(cindex) ~= 0
            durations(wId, rId) = events.DUR(cindex);
        end
    end
end

%% Computing number of commands
util_bdisp('[proc] - Computing number of commands');
commands = nan(nwaypoints, nruns);
for rId = 1:nruns
    for wId = 1:nwaypoints
        crunid = runs(rId);
        cwypid = WayPointEvent(wId);
        cindex = events.TRK == crunid & events.WYP == cwypid & (events.TYP == CommandEvent(1) | events.TYP == CommandEvent(2));

        if sum(cindex) ~= 0
            commands(wId, rId) = sum(cindex);
        end
    end
end

%% Computing frechet distance
util_bdisp('[proc] - Computing frechet distances');
Step = 128;
reftraj = refdata.mtrajectory(1:Step:end, :);
refWk   = refdata.mWayK(1:Step:end);
traj    = data.pose(1:Step:end, 1:2);
dWayK   = WayK(1:Step:end);
dRk     = Rk(1:Step:end);

frechet = nan(nwaypoints, nruns);
for rId = 1:nruns
    util_disp_progress(rId, nruns, '        ');
    for wId = 1:nwaypoints
        crunid = runs(rId);
        cwypid = WayPointEvent(wId);
        cindex = dRk == crunid & dWayK == cwypid;
        if sum(cindex) ~= 0
            creftraj = reftraj(refWk == cwypid, :);
            ctraj    = traj(cindex, :);
            frechet(wId, rId) = proc_frechet_distance(ctraj, creftraj);
        end
    end
end

%% Saving results
sfilename = [savedir '/' subject '_navigation_results.mat'];
util_bdisp(['[out] - Saving navigation results in: ' sfilename]);
save(sfilename, 'performances', 'durations', 'commands', 'frechet', 'RunControlK'); 




%% Functions

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

function events = add_waypoint_event(WayK, events)
    nevents = length(events.POS);
    
    WYP = nan(nevents, 1);
    for eId = 1:nevents
        cpos = events.POS(eId);
        cwyp = WayK(cpos);
    
        if(cwyp ~= 0)
            WYP(eId) = cwyp;
        end
    end
    events.WYP = WYP;
end

