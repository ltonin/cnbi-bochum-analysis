function [evtvec, evtstr] = proc_get_event3(evt, T, POS, TYP, DUR)
% [evtvec, evtstr] = proc_get_event3(evt, T, POS, TYP, DUR)
%
% The function create a vector related to the provided events. Similar to
% proc_get_event2, in this case it allows to provide a time vector and
% floating duration. This allows to use the function with data without
% uniform sampling rate.
%
% Input:
%   - evt               List of the events to be extracted.
%   - T                 Time vector
%   - POS, TYP, DUR     Vector with positions, types and durations of all
%                       events (e.g., typical fields in gdf EVENT
%                       structure. The three vectors must have the same
%                       length. Optionally dur can be an integer. In this
%                       case, such a fixed duration is applied to all the
%                       extracted events.
%
% Output:
%   - evtvec            Vector of length nsamples. Value of each sample is
%                       the type of the extracted event or zeros otherwise.
%   - evtstr            Structure with the extracted events in the gdf
%                       EVENT structure format (evtstr.POS, evtstr.TYP,
%                       evstr.DUR). This structure refers to the new
%                       position and duration (for instance if an offset is
%                       provided).


    if length(DUR) == 1
        DUR = DUR*ones(size(POS));
    end

    if(isequal(length(POS), length(TYP)) == false)
        error('chk:length', 'POS and TYP must have the same lenght');
    end
    
    if(isequal(length(POS), length(DUR)) == false)
        error('chk:length', 'DUR must have the same length of POS and TYP')
    end
    
    % Get the number of samples
    nsamples = length(T);
    
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
    evtstr.POS = zeros(nsevents, 1);
    evtstr.TYP = zeros(nsevents, 1);
    evtstr.DUR = zeros(nsevents, 1);
    
    for eId = 1:nsevents
        cpos = pos_s(eId);
        ctyp = typ_s(eId);
        cdur = dur_s(eId);
        
        
        if isnan(cdur)
            cdur = 0;
        end
           
        
        
        cstart = cpos;
        tstop = T(cpos) + cdur;
        [~, cstop] = min(abs(T - tstop));
        
%         cstop  = cpos + cdur - 1;
        evtvec(cstart:cstop) = ctyp;
        evtstr.POS(eId) = cstart;
        evtstr.TYP(eId) = ctyp;
        evtstr.DUR(eId) = length(cstart:cstop);
    end
    
    evtvec = evtvec(1:nsamples);

end
