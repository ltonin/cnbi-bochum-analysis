clearvars; clc;

subject = 'RND01';

includepat  = {subject, 'mi', 'mi_bhbf', 'control', 'navigation'};
excludepat  = {};
depthlevel  = 2;

experiment  = 'mi_wheelchair';
datapath    = ['/mnt/data/Research/mi_wheelchair_bochum/parkours/' subject '_' experiment '/'];

savedir     = 'analysis/navigation/motion/';

files = util_getfile3(datapath,  '.bag', 'include', includepat, 'exclude', excludepat, 'level', depthlevel);
numfiles = length(files);

% Create/Check for savepath
util_mkdir(pwd, savedir);

% Default values
SmoothOrder   = 1;
SmoothBand    = 0.2;
SmoothFreq    = 1;
TopicVelocity = '/cmd_vel';
SimTime = true;
NumCols = 5;
tmpdata = load('/mnt/data/Git/Codes/cnbi-bochum-analysis/analysis/navigation/motion/BOCH02.20190128.142219.online.mi.mi_bhbf.wheelchair.control.navigation.mat');

for fId = 1:numfiles
    [path, name, ext] = fileparts(files{fId});
    
    util_disp(['[io]   + Loading file ' num2str(fId) '/' num2str(numfiles) ' from ' path ':'], 'b'); 
    util_disp(['       - File: ' [name ext]]);
    
    %% Bag extraction
    util_disp('[io]   + Extracting data messages from topics', 'b'); 
    util_disp('       - Pose messages');
    util_disp('       - Velocity messages');
    
    [rpose, Tpose]         = cnbibochum_import_pose(files{fId}, '/odom');
    rspose.xy              = smoothfilter(rpose.xy, SmoothOrder, SmoothBand, SmoothFreq);  
    [rvelocity, Tvelocity] = cnbibochum_import_velocity(files{fId}, '/cmd_vel');
    
    % Add simtime
    if(SimTime == true)
        cdate = regexp(name, '\.+(\d+.\d+)', 'tokens');
        cdate = cell2mat(cdate{1});
        offset = posixtime(datetime(cdate, 'InputFormat', 'yyyyMMdd.HHmmss'));
        Tpose = Tpose + offset;
        Tvelocity = Tvelocity + offset;
    end
    
    %% Unification and alignment
    util_disp('[proc] + Unifying the timeseries', 'b');
    
    util_disp('       - Creating unique time support');
    T = sort([Tpose; Tvelocity]);
    
    util_disp('       - Aligning pose timeseries');
    apose.xy = cnbibochum_align_timeseries(T, rpose.xy, Tpose);
    apose.orientation = cnbibochum_align_timeseries(T, rpose.orientation, Tpose);
    aspose.xy = cnbibochum_align_timeseries(T, rspose.xy, Tpose);
    
    util_disp('       - Aligning velocity timeseries');
    avelocity.vx = cnbibochum_align_timeseries(T, rvelocity.vx, Tvelocity);
    avelocity.vz = cnbibochum_align_timeseries(T, rvelocity.vz, Tvelocity);
    
    %% Fillment NaNs values
    util_disp('[proc] + Filling missing values for the timeseries', 'b');
    util_disp('       - Filling NaNs for pose timeseries');
    pose.xy = fillmissing(apose.xy, 'linear');
    pose.orientation = fillmissing(apose.xy, 'linear');
    spose.xy = fillmissing(aspose.xy, 'linear');
    
    util_disp('       - Filling NaNs for velocity timeseries');
    velocity.vx = fillmissing(avelocity.vx, 'linear');
    velocity.vz = fillmissing(avelocity.vz, 'linear');
    
    %% Importing and align events
    util_disp('[proc] - Extracting events from related file', 'b'); 
    ievents = cnbibochum_util_import_event(files{fId}, './analysis/navigation/', 'events');
    [event, evtmismatch] = cnbibochum_events_align(T, ievents, 1);
    
    pose.xy(:, 1) = pose.xy(:, 1) - 0.5;
    spose.xy(:, 1) = spose.xy(:, 1) - 0.5;
    pose.xy(:, 2) = pose.xy(:, 2) + 0.3;
    spose.xy(:, 2) = spose.xy(:, 2) + 0.3;
    map = tmpdata.map;
    
    %% Saving data
    sfilename = fullfile(savedir, [name '.mat']);
    util_disp(['[out] - Saving motion data in: ' sfilename], 'b');
    save(sfilename, 'pose', 'spose', 'velocity', 'map', 'event', 'evtmismatch', 'T'); 
    
    %% Figure
    subplot(ceil(numfiles/NumCols), NumCols, fId);
    cnbibochum_show_map(map.info.x, map.info.y, map.data);
    hold on
    plot(pose.xy(:, 2) - map.info.origin(2), pose.xy(:, 1)- map.info.origin(1), '.r');
    plot(spose.xy(:, 2) - map.info.origin(2), spose.xy(:, 1)- map.info.origin(1), 'g', 'LineWidth', 1);
    hold off;
    
end


%%%%%%%%%%%%% Functions %%%%%%%%%%%%%%%%%%

function aseries = cnbibochum_align_timeseries(T, series, Tseries)

    aseries = nan(length(T), size(series, 2));
    idx = [];
    for tId = 1:length(Tseries)
        [~, tidx] = min( abs(T - Tseries(tId)) );
        idx = cat(1, idx, tidx);
    end
    
    for cId = 1:size(series, 2)
        aseries(idx, cId) = series(:, cId);
    end
    
end

function [path, name] = fileparts2(file)
    [path, name, ext] = fileparts(file);
    name = [name ext];
end

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

function fdata = smoothfilter(data, order, band, freq)

    fdata = nan(size(data));
    for cId = 1:size(data, 2)
        fdata(:, cId) = filt_highlow(data(:, cId), order, band, freq, 'low');
    end
    
end


function [pose, t] = cnbibochum_import_pose(bagfile, topic)

    % Importing bag
    bag = rosbag(bagfile);
    
    % Selecting topic
    sel = select(bag, 'Topic', topic);
    
    % Reading message 
    msg = readMessages(sel, 'DataFormat', 'struct');
    
    
    pose.xy(:, 1)    = cellfun(@(m) double(m.Pose.Pose.Position.X), msg);
    pose.xy(:, 2)    = cellfun(@(m) double(m.Pose.Pose.Position.Y), msg);
    pose.orientation(:, 1) = cellfun(@(m) (m.Pose.Pose.Orientation.X), msg);
    pose.orientation(:, 2) = cellfun(@(m) (m.Pose.Pose.Orientation.Y), msg);
    pose.orientation(:, 3) = cellfun(@(m) (m.Pose.Pose.Orientation.Z), msg);
    pose.orientation(:, 4) = cellfun(@(m) (m.Pose.Pose.Orientation.W), msg);
    t = sel.MessageList.Time;
end

function map = cnbibochum_import_map(bagfile, topic)

    % Importing bag
    bag = rosbag(bagfile);
    
    % Selecting topic
    sel = select(bag, 'Topic', topic);
    
    % Reading message 
    msg = readMessages(sel, 'DataFormat', 'struct');
    
    map.info.resolution = double(msg{1}.Info.Resolution);
    map.info.width      = double(msg{1}.Info.Width);
    map.info.height     = double(msg{1}.Info.Height);
    map.info.x          = 0:map.info.resolution: (map.info.width*map.info.resolution - map.info.resolution);
    map.info.y          = 0:map.info.resolution: (map.info.height*map.info.resolution - map.info.resolution);
    map.info.origin     = [msg{1}.Info.Origin.Position.X msg{1}.Info.Origin.Position.Y];
    map.data            = reshape(msg{1}.Data, map.info.width, map.info.height);
end

function [velocity, T] = cnbibochum_import_velocity(bagfile, topic)

    % Importing bag
    bag = rosbag(bagfile);
    
    % Selecting topic
    sel = select(bag, 'Topic', topic);
    
    % Reading message 
    msg = readMessages(sel, 'DataFormat', 'struct');
    
    velocity.vx = cellfun(@(m) double(m.Linear.X), msg);
    velocity.vz = cellfun(@(m) double(m.Angular.Z),msg);
    T = sel.MessageList.Time;
end
