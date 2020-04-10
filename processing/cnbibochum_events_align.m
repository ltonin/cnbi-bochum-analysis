function [nevents, mismatchIdx] = cnbibochum_events_align(timeline, events, maxmismatch)

    if nargin == 2
        maxmismatch = 1;
    end

    if(isfield(events, 'TIM') == false)
        error('Events structure does not have TIM field. Events cannot be aligned to the provided timeline');
    end
    
    nevents = events;
    
    % Cut events if outside the provided timeline
    start = timeline(1);
    stop  = timeline(end);
    indexes = nevents.TIM >= start & nevents.TIM <= stop;
    
    evtnames = fieldnames(nevents);
    
    for enId = 1:length(evtnames)
        cvalue = nevents.(evtnames{enId});
        nevents.(evtnames{enId}) = cvalue(indexes);
    end
    
    
    
    numevents = length(nevents.TIM);
    ERR = nan(numevents, 1);
    POS = nan(numevents, 1);
    for eId = 1:numevents
        cevttime = nevents.TIM(eId);
        [~, cpos] = min(abs(timeline - cevttime));
        ERR(eId) = timeline(cpos) - cevttime;
        POS(eId) = cpos;  
    end
    nevents.ERR = ERR;
    nevents.POS = POS;
    
    nevents = orderfields(nevents);
    
    delayIdx = find(abs(nevents.ERR) > maxmismatch);
    mismatchIdx = [];
    if(isempty(delayIdx) == false)
        ndelays = length(delayIdx);
        warning(['Alignment mismatch: ' num2str(ndelays) '/' num2str(numevents) ' (' num2str(100*ndelays/numevents, '%5.2f') '%) have a mismatch greater than ' num2str(maxmismatch) ' s.']);
        mismatchIdx = delayIdx;
    end
    

end
