clearvars; clc;

subject = 'BOCH04';

includepat  = {subject, 'mi', 'mi_bhbf', 'control'};
%excludepat  = {'20190121', '20190125', 'video', 'vodom'};
excludepat  = {'20190607', '20190617', 'video', 'vodom'};
depthlevel  = 2;

experiment  = 'mi_wheelchair';
datapath    = ['/mnt/data/Research/mi_wheelchair_bochum/parkours/' subject '_' experiment '/'];
savedir     = 'analysis/navigation/events/';

files = util_getfile3(datapath, '.bag', 'include', includepat, 'exclude', excludepat, 'level', depthlevel);
NumFiles = length(files);

% Create/Check for savepath
util_mkdir(pwd, savedir);

EventTypes  = [781 897 26113 26117 26118];
EventLabels = {'continuous feedback', 'target hit', 'bci start', 'bci left', 'bci right'};

%% Import bag files
for fId = 1:NumFiles
    cfilepath = files{fId};
    [cfolder, cfilename, cext] = fileparts(cfilepath);
    util_disp(['[io] - Loading file ' num2str(fId) '/' num2str(NumFiles) ' from : ' cfolder], 'b'); 
    util_disp(['     - File: ' cfilename cext]); 
    
    % Extracting filebag
    util_disp('     - Extracting bag'); 
    bag = rosbag(cfilepath);
    
    % Extracting events topic
    util_disp('     - Extracting event topics'); 
    stid = select(bag, 'Topic', '/rostid_cnbi2ros');
    ssys = select(bag, 'Topic', '/control/system_state');
    
    % Reading event messages
    util_disp('     - Selecting event messages'); 
    msgtid = readMessages(stid, 'DataFormat', 'struct');
    msgsys = readMessages(ssys, 'DataFormat', 'struct');
    
    % Extracting events typ from tid and sys
    evttid_typ = cellfun(@(m) double(m.Event), msgtid);
    evtsys_typ = cellfun(@(m) double(m.Id), msgsys);
    evtsys_lbl = cellfun(@(m) (m.Label), msgsys, 'UniformOutput', false);
    
    % Extracting event time
    evttid_tim = stid.MessageList.Time;
    evtsys_tim = ssys.MessageList.Time;
    
    % Unify events
    [evttim, evttim_idx] = sort([evttid_tim; evtsys_tim]);
    evttyp = [evttid_typ; evtsys_typ];
    evttyp = evttyp(evttim_idx);
    evtlbl = [cell(length(evttid_typ), 1); evtsys_lbl];
    evtlbl = evtlbl(evttim_idx);
    
    % Compute event duration (checking for the off event)
    event.TYP = evttyp;
    event.TIM = evttim;
    event.LBL = evtlbl;
    [event.DUR, offevents] = compute_event_duration(event.TYP, event.TIM);
    
    % Remove off events
    event.TYP = event.TYP(~offevents);
    event.TIM = event.TIM(~offevents);
    event.LBL = event.LBL(~offevents);
    event.DUR = event.DUR(~offevents);
    
    % Assign labels to event
    event.LBL = assign_event_label(event.TYP, event.LBL, EventTypes, EventLabels);
    
    
    sfilename = [savedir '/' cfilename '.mat'];
    util_disp(['[out] - Saving events data in: ' sfilename], 'b');
    save(sfilename, 'event'); 
end


function [DUR, offevents] = compute_event_duration(TYP, TIM, offevent)
    
    if nargin == 2
        offevent = '8000';
    end
    
    Events = unique(TYP);
    NumEvents = length(Events);
    
    DUR = zeros(length(TYP), 1);
    offeventIdx = [];
    
    for eId = 1:NumEvents
        cevton  = Events(eId);
        cevtoff = Events(eId) + hex2dec(offevent);
        cevtonIdx  = find(TYP == cevton);
        cevtoffIdx = find(TYP == cevtoff);
        
        % If there is not stop event, continue to the next type (leave
        % duration to 0)
        if isempty(cevtoffIdx)
            continue;
        end
        
        nevtoffIdx = [];
        for i = 1:length(cevtonIdx)
            cIdx = find(cevtoffIdx >= cevtonIdx(i), 1, 'first');
            
            if( isempty(cIdx) )
                warning(['Missing off event for: 0x' dec2hex(cevton) '|0x' dec2hex(cevtoff)]);
                continue;
            end
            nevtoffIdx = cat(1, nevtoffIdx, cevtoffIdx(cIdx));
        end
        cevtoffIdx = nevtoffIdx;
        
        % Check if there is equal number of cevtonIdx and coffIdx, otherwise
        % continue to the next event type (leave duration to 0)
        if( isequal(length(cevtonIdx), length(cevtoffIdx)) == false )
            warning(['Different number of events: 0x' dec2hex(cevton) '|0x' dec2hex(cevtoff)]);
            continue;
        end
        
        
%         % Check if coffIdx come after conIdx
%         if ( isequal(sum((cevtoffIdx-cevtonIdx) > 0), length(cevtoffIdx)) == false )
%              warning(['Wrong order for events: 0x' dec2hex(cevton) '|0x' dec2hex(cevtoff)]);
%              keyboard
%              continue;
%         end
        
        % Compute duration of the event
        DUR(cevtonIdx) = TIM(cevtoffIdx) - TIM(cevtonIdx);
        offeventIdx = cat(1, offeventIdx, cevtoffIdx);  
    end

    offevents = false(length(TYP), 1);
    offevents(offeventIdx) = true;
end

function LBL = assign_event_label(TYP, LBL, EventTypes, EventLabels) 

    nevt = length(TYP);
    
    for eId = 1:nevt
        ctyp = TYP(eId);
        
        ctypIdx = find(EventTypes == ctyp, 1);
        
        if(isempty(ctypIdx))
            continue;
        end
        
        LBL(eId) = EventLabels(ctypIdx);
    end

end
