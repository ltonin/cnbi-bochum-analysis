clearvars; clc;

subject = 'BOCH02';

includepat  = {subject, 'mi', 'mi_bhbf', 'vodom'};
excludepat  = {};
depthlevel  = 2;

experiment  = 'mi_wheelchair';
datapath    = ['/mnt/data/Research/mi_wheelchair_bochum/navigation/' subject '_' experiment '/'];
savedir     = 'analysis/navigation/path/';

files = util_getfile3(datapath, '.bag', 'include', includepat, 'exclude', excludepat, 'level', depthlevel);
NumFiles = length(files);

% Create/Check for savepath
util_mkdir(pwd, savedir);

%% Default values
SmoothOrder = 1;
SmoothBand  = 0.2;
SmoothFreq  = 1;

%% Import bag files

for fId = 1:NumFiles
    
    cfilepath = files{fId};
    [cfolder, cfilename, cext] = fileparts(cfilepath);
    util_disp(['[io] - Loading file ' num2str(fId) '/' num2str(NumFiles) ' from : ' cfolder], 'b'); 
    util_disp(['     - File: ' cfilename cext]); 
    
    % Initialize structures
    pose = []; map = []; 
    
    % Extracting filebag
    util_disp('     - Extracting bag'); 
    bag = rosbag(cfilepath);
    
    % Extracting pose localization
    util_disp('     - Extracting pose localization'); 
    spose = select(bag, 'Topic', '/rtabmap/localization_pose');
    
    % Selecting pose messages
    util_disp('     - Selecting pose messages'); 
    msgpose = readMessages(spose, 'DataFormat', 'struct');
    
    % Extracting map
    util_disp('     - Extracting map'); 
    smap = select(bag, 'Topic', '/rtabmap/proj_map');
    
    % Selecting map messages
    util_disp('     - Selecting pose messages'); 
    msgmap = readMessages(smap, 'DataFormat', 'struct');
    
    
    util_disp('[proc] - Extracting pose coordinates', 'b');
    pose.raw.xy(:, 1)    = cellfun(@(m) double(m.Pose.Pose.Position.X),msgpose);
    pose.raw.xy(:, 2)    = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgpose);
    pose.raw.orientation = rmfield(cellfun(@(m) (m.Pose.Pose.Orientation),msgpose), 'MessageType');

    util_disp('[proc] - Smoothing pose coordinates', 'b');
    pose.smt.xy    = smoothfilter(pose.raw.xy, SmoothOrder, SmoothBand, SmoothFreq);
    pose.smt.order = SmoothOrder;
    pose.smt.band  = SmoothBand;
    pose.smt.freq  = SmoothFreq;
    
    util_disp('[proc] - Extracting map data and coordinates', 'b');
    map.info.resolution = double(msgmap{1}.Info.Resolution);
    map.info.width      = double(msgmap{1}.Info.Width);
    map.info.height     = double(msgmap{1}.Info.Height);
    map.info.x          = 0:map.info.resolution: (map.info.width*map.info.resolution - map.info.resolution);
    map.info.y          = 0:map.info.resolution: (map.info.height*map.info.resolution - map.info.resolution);
    map.info.origin     = [msgmap{1}.Info.Origin.Position.X msgmap{1}.Info.Origin.Position.Y];
    map.data            = reshape(msgmap{1}.Data, map.info.width, map.info.height);
  
    util_disp('[proc] - Extracting time and file offset', 'b');
    time   = spose.MessageList.Time;
    offset = spose.MessageList.FileOffset; 
    
    sfilename = [savedir '/' cfilename '.mat'];
    util_disp(['[out] - Saving odometry data in: ' sfilename]);
    save(sfilename, 'pose', 'map', 'time', 'offset'); 
    
    
    subplot(2, 5, fId);
    cnbibochum_show_map(map.info.x, map.info.y, map.data);
    hold on
    plot(pose.raw.xy(:, 2) - map.info.origin(2), pose.raw.xy(:, 1)- map.info.origin(1), '.r');
    plot(pose.smt.xy(:, 2) - map.info.origin(2), pose.smt.xy(:, 1)- map.info.origin(1), 'g', 'LineWidth', 1);
    hold off;
end

function fdata = smoothfilter(data, order, band, freq)

    fdata = nan(size(data));
    for cId = 1:size(data, 2)
        fdata(:, cId) = filt_highlow(data(:, cId), order, band, freq, 'low');
    end
    
end
