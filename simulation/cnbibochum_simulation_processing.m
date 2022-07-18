clearvars; clc;

subject = 'PERFECT';

includepat  = {subject, 'simulation', 'sharedcontrol'};
excludepat  = {};
depthlevel  = 3;

rootpath    = '/mnt/data/rosneuro_data/';
folder      = 'bochum_simulation';
gdfpath     = [rootpath '/' folder '/'];
savedir     = 'analysis/simulation/';
recompute   = true;

SampleRate = 512;

CmdAngleValue = [45 0 -45];
CmdAngleLabel = {'left', 'forward', 'right'};
CmdEventId    = [26117 26115 26118];
velthreshold = 0.15;

WpType = [801 802 803 804];
WpPosition(:, 1) = [-0.771 -1.479];
WpPosition(:, 2) = [ 0.256  2.733];
WpPosition(:, 3) = [-0.771 -1.479];
WpPosition(:, 4) = [-2.012 -5.655];

WpLabels = {'WP1', 'WP2', 'WP3', 'WP4'};

nwaypoints = size(WpPosition, 2);

%% Get datafiles
files = util_getfile3(gdfpath, '.bag', 'include', includepat, 'exclude', excludepat, 'level', depthlevel);

nfiles = length(files);
if(nfiles > 0)
    util_bdisp(['[io] - Found ' num2str(nfiles) ' files with the inclusion/exclusion criteria: (' strjoin(includepat, ', ') ') / (' strjoin(excludepat, ', ') '), depth: ' num2str(depthlevel)]);
else
    error(['[io] - No files found with the inclusion/exclusion criteria: (' strjoin(includepat, ', ') ') / (' strjoin(excludepat, ', ') '), depth: ' num2str(depthlevel)]);
end

%% Create/Check for savepath
util_mkdir(pwd, savedir);

%% Processing files
totposes  = cell(nfiles, 1);
totevents = cell(nfiles, 1);

for fId = 1:nfiles

    cfullname = files{fId};
    [cfilepath, cfilename, cfileext] = fileparts(cfullname);
    
    util_bdisp(['[io] + Loading file ' num2str(fId) '/' num2str(nfiles)]);
    disp(['     |-File: ' cfullname]);
    
    %% Check if the file has been already processed
    [~, pfilename] = fileparts(cfullname);
    if (recompute == false) && exist([savedir pfilename '.mat'], 'file') == 2
        disp('     |-Processed bandpass already exists. Skipping the recomputing');
        continue;
    end

    %% Loading file
    cbag = rosbag(cfullname);

    sel_cmdvel = select(cbag, 'Topic', '/cmd_vel');
    msg_cmdvel = readMessages(sel_cmdvel, 'DataFormat', 'struct');

    sel_cmdbci = select(cbag, 'Topic', '/bci_command');
    msg_cmdbci = readMessages(sel_cmdbci, 'DataFormat', 'struct');

    sel_odom = select(cbag, 'Topic', '/odom');
    msg_odom = readMessages(sel_odom, 'DataFormat', 'struct');

    %% Extracting messages
    % X and Z command velocities
    X_cmdvel = cellfun(@(m) double(m.Linear.X), msg_cmdvel);
    Z_cmdvel = cellfun(@(m) double(m.Angular.Z), msg_cmdvel);
    t_cmdvel = sel_cmdvel.MessageList.Time - cbag.StartTime;

    % X and Z positions and odom velocities
    pX_odom   = cellfun(@(m) double(m.Pose.Pose.Position.X), msg_odom);
    pY_odom   = cellfun(@(m) double(m.Pose.Pose.Position.Y), msg_odom);
    pQ_odom   = cellfun(@(m) double(quat2eul(rosReadQuaternion(m.Pose.Pose.Orientation))), msg_odom, 'UniformOutput',false);
    pQ_odom   = cell2mat(pQ_odom);
    pZ_odom   = pQ_odom(:, 1);
    vX_odom   = cellfun(@(m) double(m.Twist.Twist.Linear.X), msg_odom);
    vZ_odom   = cellfun(@(m) double(m.Twist.Twist.Angular.Z), msg_odom);
    t_odom   = sel_odom.MessageList.Time - cbag.StartTime;

    % X and Y bci command
    X_cmdbci = cellfun(@(m) double(m.Point.X), msg_cmdbci);
    Y_cmdbci = cellfun(@(m) double(m.Point.Y), msg_cmdbci);
    t_cmdbci = sel_cmdbci.MessageList.Time - cbag.StartTime;

    %% Converting analog signals in timeseries
    ts_cX = timeseries(X_cmdvel, t_cmdvel);
    ts_cZ = timeseries(Z_cmdvel, t_cmdvel);

    ts_pX = timeseries(pX_odom, t_odom);
    ts_pY = timeseries(pY_odom, t_odom);
    ts_pZ = timeseries(pZ_odom, t_odom);
    ts_vX = timeseries(vX_odom, t_odom);
    ts_vZ = timeseries(vZ_odom, t_odom);

    t = 0:1/SampleRate:ceil(cbag.EndTime - cbag.StartTime);

    %% Resample analog signals with respect to uniform time support
    tts_cX = resample(ts_cX, t, 'zoh');
    tts_cZ = resample(ts_cZ, t, 'zoh');
    tts_pX = resample(ts_pX, t, 'zoh');
    tts_pY = resample(ts_pY, t, 'zoh');
    tts_pZ = resample(ts_pZ, t, 'zoh');
    tts_vX = resample(ts_vX, t, 'zoh');
    tts_vZ = resample(ts_vZ, t, 'zoh');

    %% Create events structure and synchronized with time support
    cmdevt = extract_command_event(X_cmdbci, Y_cmdbci, t_cmdbci, t, CmdAngleLabel, CmdEventId);

    %% Renaming analog variables
    cmdvel = [tts_cX.Data tts_cZ.Data];
    pose   = [tts_pX.Data tts_pY.Data tts_pZ.Data];
    posvel = [tts_vX.Data tts_vZ.Data];

    %% Extract waypoint events
    wpevt = extract_waypoint_event(pose, WpPosition, WpType, cmdvel, velthreshold);
    
    %% Merge event structures
    events = f_merge_events(cmdevt, wpevt);

    %% Saving the navigation data
    sfilename = [savedir '/' pfilename '.mat'];
    util_bdisp(['[out] - Saving navigation data in: ' sfilename]);
    save(sfilename, 'cmdvel', 'pose', 'posvel', 't', 'events', 'SampleRate'); 

    totposes{fId} = pose;
    totevents{fId} = events;
end

%% Figure
figure;
fig_set_position(gcf, 'All');
nrows = 2;
ncols = ceil(nfiles/nrows);
WpColors = {'r', 'g', 'y', 'c'};
    
for fId = 1:nfiles
    
    subplot(nrows, ncols, fId);

    cx = totposes{fId}(:, 1);
    cy = totposes{fId}(:, 2);
    cwptyp = totevents{fId}.TYP;
    cwppos = totevents{fId}.POS;

    hold on;
    plot(cx, cy);
    for wId = 1:nwaypoints
        cindex = find(cwptyp == WpType(wId));
        
        if(isnan(cindex))
            continue;
        end
        cposidx = cwppos(cindex);

        plot(cx(cposidx), cy(cposidx), 'o', 'Color', WpColors{wId}, 'LineWidth', 2);
    end
    hold off;

    axis equal;
    grid on;
    xlabel('[m]');
    ylabel('[m]');
    title(['run ' num2str(fId)]);
end

%% Functions
function EVENT = extract_command_event(x, y, t, t_support, CmdAngleLabel,CmdEventId)

    epsilon = 0.001;

    ncmd = length(x);
    TYP = nan(ncmd, 1);
    POS = nan(ncmd, 1);
    DUR = ones(ncmd, 1);
    
    for eId = 1:ncmd
        cx = x(eId);
        cy = y(eId);
        ct = t(eId);

        cangle = rad2deg(atan2(cx,cy)) - 90;

        if (cangle > epsilon)
            ccmdLb = 'right';
        elseif (cangle < -epsilon)
            ccmdLb = 'left';
        else
            ccmdLb = 'forward';
        end
        
        [~, cIdx] = ismember(ccmdLb, CmdAngleLabel);
        cTYP = CmdEventId(cIdx);
        [~, cPOS] = min(abs(t_support - ct));

        TYP(eId) = cTYP;
        POS(eId) = cPOS;
    end

    EVENT.TYP = TYP;
    EVENT.POS = POS;
    EVENT.DUR = DUR;

end

function EVENT = extract_waypoint_event(pose, wppositions, wptypes, vel, velth)
    
    nwaypoints = size(wppositions, 2);

    startpos = sim_find_start_index(vel(:, 1), velth);
    [distance, index] = sim_find_minimum_distance(pose(:, 1), pose(:, 2), wppositions);
    
    TYP = [];
    POS = [];
    DIS = [];

    for wId = 1:nwaypoints
        cpos      = index(wId);
        ctype     = wptypes(wId);
        cdistance = distance(wId);
        if (isnan(cpos) == false)
            TYP = cat(1, TYP, ctype);
            POS = cat(1, POS, cpos);
            DIS = cat(1, DIS, cdistance);
        end
    end

    durindex = [startpos; index];
    DUR = diff(durindex);
    
    EVENT.POS = [startpos; POS];
    EVENT.TYP = [800; TYP];
    
    if(isempty(POS) == true)
        EVENT.DUR = nan;
    else
        EVENT.DUR = [(POS(end) - startpos); DUR];
    end
    

end

function event = f_merge_events(cmdevt, wpevt)


    cPOS = [cmdevt.POS; wpevt.POS];
    cTYP = [cmdevt.TYP; wpevt.TYP];
    cDUR = [cmdevt.DUR; wpevt.DUR];


    [POS, idx] = sort(cPOS);
    TYP = cTYP(idx);
    DUR = cDUR(idx);

    event.POS = POS;
    event.TYP = TYP;
    event.DUR = DUR;
    
end
