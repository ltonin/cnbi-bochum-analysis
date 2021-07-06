clearvars; clc;

subject = 'RND01';

include  = {subject, 'mi', 'mi_bhbf', 'control', 'navigation'};
excludepat  = {};
depthlevel  = 2;

experiment  = 'mi_wheelchair';
datapath    = ['/mnt/data/Research/mi_wheelchair_bochum/parkours/' subject '_' experiment '/'];

savedir     = 'analysis/navigation/new/raw/';

files = util_getfile3(datapath, '.bag', 'include', include, 'exclude', excludepat, 'level', depthlevel);
NumFiles = length(files);

% Create/Check for savepath
util_mkdir(pwd, savedir);

% Default topic names values
TopicPose        = '/odom'; 
TopicVelocity    = '/cmd_vel';
TopicBciEvent    = '/bci_command';
TopicSystemEvent = '/control/system_state';
SimTime          = true;
DefaultMapPath   = '/mnt/data/Git/Codes/cnbi-bochum-analysis/analysis/navigation/new/raw/BOCH02.20190128.142219.online.mi.mi_bhbf.wheelchair.control.navigation.mat';

% Default event types and labels
EventTypes  = [781 897 26113 26117 26118];
EventLabels = {'continuous feedback', 'target hit', 'bci start', 'bci left', 'bci right'};

for fId = 1:NumFiles
    [path, name, ext] = fileparts(files{fId});
    
    util_disp(['[io]   + Loading file ' num2str(fId) '/' num2str(NumFiles) ' from ' path ':'], 'b'); 
    util_disp(['       |- File: ' [name ext]]); 
    
    %% Bag extraction
    util_disp('[proc] + Extracting data messages from topics', 'b'); 
    util_disp('       |- Pose messages');
    util_disp('       |- Velocity messages');
    util_disp('       |- Events');
    
    % Import pose from visual odometry bags
    [P, TP] = support_import_pose(files{fId}, TopicPose);  
    
    % Import velocity from navigation bags
    [V, TV] = support_import_velocity(files{fId}, TopicVelocity);
    
    % Import map from default subject
    map = support_import_map(DefaultMapPath);
    
    % Import events from navigation bags
    events_e = support_import_events(files{fId}, TopicBciEvent, TopicSystemEvent);
    
    % Add simtime
    if(SimTime == true)
        cdate = regexp(name, '\.+(\d+.\d+)', 'tokens');
        cdate = cell2mat(cdate{1});
        offset = posixtime(datetime(cdate, 'InputFormat', 'yyyyMMdd.HHmmss'));
        TP = TP + offset;
        TV = TV + offset;
        events_e.TIM = events_e.TIM + offset;
    end
    
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
    sfilename = fullfile(savedir, [name '.mat']);
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

function map = support_import_map(filepath)

    if(exist(filepath, 'file') == 0)
        warning('Default file map does not exist');
    else
        cdata = load(filepath);
        map = cdata.map;
    end
end

function evt = support_import_events(bagfile, TopicBci, TopicSys)
        
    % Importing bag
    bag = rosbag(bagfile);
    
    % Selecting topic
    selbci = select(bag, 'Topic', TopicBci);
    selsys = select(bag, 'Topic', TopicSys);
    
    % Reading messages
    msgbci = readMessages(selbci, 'DataFormat', 'struct');
    msgsys = readMessages(selsys, 'DataFormat', 'struct');
    
    % Extracting events typ from tid and sys
    evtbci_typ = cellfun(@(m) double(m.Point.Y), msgbci);
    evtsys_typ = cellfun(@(m) double(m.Id), msgsys);
    evtsys_lbl = cellfun(@(m) (m.Label), msgsys, 'UniformOutput', false);
    
    % Conversion from bci point to event
    evtbci_typ(evtbci_typ == -0.7070) = 26117;
    evtbci_typ(evtbci_typ == 0.7070)  = 26118;
    
    
    % Extracting event time
    evtbci_tim = selbci.MessageList.Time;
    evtsys_tim = selsys.MessageList.Time;
    
    
    % Unify events (TYP and TIM)
    evttim = [evtbci_tim; evtsys_tim];
    evttyp = [evtbci_typ; evtsys_typ];
    evtlbl = [cell(length(evtbci_typ), 1); evtsys_lbl]; % Empty label for TiD events
    
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
 
    nevt = length(TYP);
    LBL  = cell(nevt, 1);
    
    for eId = 1:nevt
        ctyp = TYP(eId);
        
        ctypIdx = find(EventTypes == ctyp, 1);
        
        if(isempty(ctypIdx))
            continue;
        end
        
        LBL(eId) = EventLabels(ctypIdx);
    end
    events.LBL = LBL;

end
