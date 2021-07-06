clearvars; clc;

subject = 'BOCH04';

include_v  = {subject, 'mi', 'mi_bhbf', 'control', 'vodom'};
include_n  = {subject, 'mi', 'mi_bhbf', 'control', 'navigation'};
excludepat  = {'20190523'};
depthlevel  = 2;

experiment  = 'mi_wheelchair';
datapath    = ['/mnt/data/Research/mi_wheelchair_bochum/parkours/' subject '_' experiment '/'];

savedir     = 'analysis/navigation/new/raw/';

files_v = util_getfile3(datapath, '.bag', 'include', include_v, 'exclude', excludepat, 'level', depthlevel);
files_n = util_getfile3(datapath, '.bag', 'include', include_n, 'exclude', excludepat, 'level', depthlevel);
if(isequal(length(files_v), length(files_n)) == false), error('Different number of files'); end

NumFiles = length(files_v);

% Create/Check for savepath
util_mkdir(pwd, savedir);

% Default topic names values
if( strcmp(subject, 'BOCH04') || strcmp(subject, 'BOCH05') )
    TopicPose        = '/rtabmap/odom'; 
else
    TopicPose        = '/rtabmap/localization_pose'; 
end
TopicMap         = '/rtabmap/proj_map';
TopicVelocity    = '/cmd_vel';
TopicTidEvent    = '/rostid_cnbi2ros';
TopicSystemEvent = '/control/system_state';

% Default event types and labels
EventTypes  = [781 897 26113 26117 26118];
EventLabels = {'continuous feedback', 'target hit', 'bci start', 'bci left', 'bci right'};

for fId = 1:NumFiles
    [path_v, name_v, ext_v] = fileparts(files_v{fId});
    [path_n, name_n, ext_n] = fileparts(files_n{fId});
    
    util_disp(['[io]   + Loading file ' num2str(fId) '/' num2str(NumFiles) ' from ' path_v ':'], 'b'); 
    util_disp(['       |- File: ' [name_v ext_v]]); 
    util_disp(['       |- File: ' [name_n ext_n]]);
    
    %% Bag extraction
    util_disp('[proc] + Extracting data messages from topics', 'b'); 
    util_disp('       |- Pose messages');
    util_disp('       |- Velocity messages');
    util_disp('       |- Map messages');
    util_disp('       |- Events');
    
    % Default topic names values
    if( strcmp(subject, 'BOCH05') && fId == 4 )
        cTopicPose = '/rtabmap/localization_pose';
    else 
        cTopicPose = TopicPose;
    end
    
    % Import pose from visual odometry bags
    [P, TP] = support_import_pose(files_v{fId}, cTopicPose);  
    
    % Import pose from visual odometry bags
    map = support_import_map(files_v{fId}, TopicMap);
    
    % Import velocity from navigation bags
    [V, TV] = support_import_velocity(files_n{fId}, TopicVelocity);
    
    % Import events from navigation bags
    events_e = support_import_events(files_n{fId}, TopicTidEvent, TopicSystemEvent);
    
    % Compute events duration
    events = support_compute_durations(events_e);
    
    % Assign labels to event
    events = support_assign_labels(events, EventTypes, EventLabels);
    
    
    % Create data structures
    pose   = P;
    pose.T = TP;
    
    velocity   = V;
    velocity.T = TV;
    
    %% Saving data
    sfilename = fullfile(savedir, [name_n '.mat']);
    util_disp(['[out] - Saving navigation data in: ' sfilename], 'b');
    save(sfilename, 'pose', 'velocity', 'map', 'events'); 
end




%% Support functions

function [P, T] = support_import_pose(bagfile, topic)

    % Importing bag
    bag = rosbag(bagfile);
    
    % Selecting topic
    sel = select(bag, 'Topic', topic);
    
    % Reading message 
    msg = readMessages(sel, 'DataFormat', 'struct');
    
    P.xy(:, 1)    = cellfun(@(m) double(m.Pose.Pose.Position.X), msg);
    P.xy(:, 2)    = cellfun(@(m) double(m.Pose.Pose.Position.Y), msg);
    P.orientation(:, 1) = cellfun(@(m) (m.Pose.Pose.Orientation.X), msg);
    P.orientation(:, 2) = cellfun(@(m) (m.Pose.Pose.Orientation.Y), msg);
    P.orientation(:, 3) = cellfun(@(m) (m.Pose.Pose.Orientation.Z), msg);
    P.orientation(:, 4) = cellfun(@(m) (m.Pose.Pose.Orientation.W), msg);
    T = sel.MessageList.Time;
end

function map = support_import_map(bagfile, topic)

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

function [V, T] = support_import_velocity(bagfile, topic)

    % Importing bag
    bag = rosbag(bagfile);
    
    % Selecting topic
    sel = select(bag, 'Topic', topic);
    
    % Reading message 
    msg = readMessages(sel, 'DataFormat', 'struct');
    
    V.vx = cellfun(@(m) double(m.Linear.X), msg);
    V.vz = cellfun(@(m) double(m.Angular.Z),msg);
    T = sel.MessageList.Time;
end

function evt = support_import_events(bagfile, TopicTid, TopicSys)
        
    % Importing bag
    bag = rosbag(bagfile);
    
    % Selecting topic
    seltid = select(bag, 'Topic', TopicTid);
    selsys = select(bag, 'Topic', TopicSys);
    
    % Reading messages
    msgtid = readMessages(seltid, 'DataFormat', 'struct');
    msgsys = readMessages(selsys, 'DataFormat', 'struct');
    
    % Extracting events typ from tid and sys
    evttid_typ = cellfun(@(m) double(m.Event), msgtid);
    evtsys_typ = cellfun(@(m) double(m.Id), msgsys);
    evtsys_lbl = cellfun(@(m) (m.Label), msgsys, 'UniformOutput', false);
    
    % Extracting event time
    evttid_tim = seltid.MessageList.Time;
    evtsys_tim = selsys.MessageList.Time;
    
    % Unify events (TYP and TIM)
    evttim = [evttid_tim; evtsys_tim];
    evttyp = [evttid_typ; evtsys_typ];
    evtlbl = [cell(length(evttid_typ), 1); evtsys_lbl]; % Empty label for TiD events
    
    % Sorting events by ascending order
    [evttim, evttim_idx] = sort(evttim);
    evttyp = evttyp(evttim_idx);
    evtlbl = evtlbl(evttim_idx);
    
    % Creating event structure
    evt.TIM = evttim;
    evt.TYP = evttyp;
    evt.LBL = evtlbl;

end

function events = support_compute_durations(events, EVENT_OFF)

    warning backtrace off;
    
    if nargin == 1
        EVENT_OFF = '8000';
    end
    
    TYP = events.TYP;
    TIM = events.TIM;
    LBL = events.LBL;
    
    EVENT_VALUES = unique(TYP);
    NumEvents    = length(EVENT_VALUES);
    
    DUR = zeros(length(TYP), 1);
    offeventIdx = [];
    oneventIdx_not_valid = [];
    for eId = 1:NumEvents
        cEVENT_ON  = EVENT_VALUES(eId);
        cEVENT_OFF = EVENT_VALUES(eId) + hex2dec(EVENT_OFF);
        cEVENT_ON_IDX  = find(TYP == cEVENT_ON);
        cEVENT_OFF_IDX = find(TYP == cEVENT_OFF);
        
        % If there is not stop events, continue to the next type (leave duration to 0)
        if isempty(cEVENT_OFF_IDX)
            continue;
        end
        
        % Otherwise, iterate along event onset indexes and find the corresponding offset indexes
        cEVENT_OFF_IDX_VALID = [];
        cEVENT_ON_IDX_VALID  = [];
        for onId = 1:length(cEVENT_ON_IDX)
            cIdx = find(cEVENT_OFF_IDX >= cEVENT_ON_IDX(onId), 1, 'first');
            
            if( isempty(cIdx) )
                warning(['Missing offset event for: 0x' dec2hex(cEVENT_ON) '|0x' dec2hex(cEVENT_OFF) ' [#: ' num2str(onId) '/' num2str(length(cEVENT_ON_IDX)) ']']);
                continue;
            end
            
            cEVENT_OFF_IDX_VALID = cat(1, cEVENT_OFF_IDX_VALID, cEVENT_OFF_IDX(cIdx));
            cEVENT_ON_IDX_VALID  = cat(1, cEVENT_ON_IDX_VALID,  cEVENT_ON_IDX(onId));
        end
        
        % Check and reporting the excluded offset events.
        cEXCLUDED_OFF_IDX = setdiff(cEVENT_OFF_IDX, cEVENT_OFF_IDX_VALID);
        
        for offId = 1:length(cEXCLUDED_OFF_IDX)
            warning(['No correspondance for offset event: 0x' dec2hex(cEVENT_OFF) ' [#: ' num2str(cEXCLUDED_OFF_IDX(offId)) '/' num2str(length(cEVENT_OFF_IDX)) ']. Excluded.']);
            
        end
             
        % Compute duration of the event (for all events. They not valid will be excluded afterwards)
        DUR(cEVENT_ON_IDX_VALID) = TIM(cEVENT_OFF_IDX_VALID) - TIM(cEVENT_ON_IDX_VALID);
        
        % Store all offset events
        offeventIdx = cat(1, offeventIdx, cEVENT_OFF_IDX);  
        
        % Store unvalid onset events
        if ( isempty(setdiff(cEVENT_ON_IDX, cEVENT_ON_IDX_VALID)) == false)
            oneventIdx_not_valid = cat(1, oneventIdx_not_valid, setdiff(cEVENT_ON_IDX, cEVENT_ON_IDX_VALID));
        end

        
    end

    % Create label vector for the offset events
    eventvalid = true(length(TYP), 1);
    eventvalid(offeventIdx) = false;
    eventvalid(oneventIdx_not_valid) = false;
    
    events.TYP = TYP(eventvalid);
    events.TIM = TIM(eventvalid);
    events.DUR = DUR(eventvalid);
    events.LBL = LBL(eventvalid);
    
    warning backtrace on;
end

function events = support_assign_labels(events, EventTypes, EventLabels) 

    TYP = events.TYP;
    eLBL = events.LBL;
    
    nevt = length(TYP);
    LBL  = cell(nevt, 1);
    
    for eId = 1:nevt
        
        if(cellfun(@(m) isempty(m), eLBL(eId)) == false)
            LBL(eId) = eLBL(eId);
            continue;
        end
        ctyp = TYP(eId);
        
        ctypIdx = find(EventTypes == ctyp, 1);
        
        if(isempty(ctypIdx))
            continue;
        end
        
        LBL(eId) = EventLabels(ctypIdx);
    end
    events.LBL = LBL;

end
