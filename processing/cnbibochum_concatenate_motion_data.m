function [motion, map, events, labels] = cnbibochum_concatenate_motion_data(files, eventpath)


    numfiles = length(files);
    
    P = []; Vx = []; Vz = []; R = []; T = []; map = [];
    POS = []; TYP= []; DUR = []; TIM = []; LBL = []; ERR = [];
    Mk = []; Rk = []; Dk = [];
    pdate = '';
    dId = 0;
    for fId = 1:numfiles
        
        % Import data
        [cpath, cfilename, cext] = fileparts(files{fId});
        util_disp('[io] - Import motion data:', 'b');
        util_disp(['     + Filename: ' files{fId}]);
        mdata = load(fullfile(cpath, [cfilename cext]));
        
        % Import waypoint data events
        util_disp(['[io] + Import and waypoint events from : ' eventpath], 'b');
        ievents  = cnbibochum_util_import_event(cfilename, eventpath);
        wpevents = cnbibochum_events_align(mdata.T, ievents);
        
        % Merge events
        mevents = cnbibochum_util_event_merge(mdata.event, wpevents);
        
        % Concatenate events
        POS = cat(1, POS, mevents.POS + length(T));
        TYP = cat(1, TYP, mevents.TYP);
        DUR = cat(1, DUR, mevents.DUR);
        TIM = cat(1, TIM, mevents.TIM);
        LBL = cat(1, LBL, mevents.LBL);
        ERR = cat(1, ERR, mevents.ERR);
        
        % Compare and save map
        if ( isequal(map, mdata.map) == false)
            map = cat(1, map, mdata.map);
            mapId = length(map);
        end
        
               
        % Get current date
        [pdate, dId] = getdate(cfilename, pdate, dId); 
        
        % Create labels
        Mk = cat(1, Mk, mapId*ones(length(mdata.T), 1));
        Rk = cat(1, Rk, fId*ones(length(mdata.T), 1));
        Dk = cat(1, Dk, dId*ones(length(mdata.T), 1));
        
        % Concatenate data
        P  = cat(1, P,  mdata.spose.xy);
        Vx = cat(1, Vx, mdata.velocity.vx);
        Vz = cat(1, Vz, mdata.velocity.vz);
        R  = cat(1, R,  mdata.pose.xy);
        T  = cat(1, T,  mdata.T);
        
        
 
        
       
        
        
    end
    
    motion.P  = P;
    motion.Vx = Vx;
    motion.Vz = Vz;
    motion.R  = R;
    motion.T  = T;
    
    events.POS = POS;
    events.TYP = TYP;
    events.DUR = DUR;
    events.TIM = TIM;
    events.LBL = LBL;
    events.ERR = ERR;
    
    labels.Mk = Mk;
    labels.Rk = Rk;
    labels.Dk = Dk;

end

%%%%%%%%%%%%%%%%%%%%% Functions %%%%%%%%%%%%%%%%%%%%%%%
function eventstr = cnbibochum_util_import_event(reffilename, eventpath)

    file_identifier = regexp(reffilename, '(\w*\.\d*\.\d*)\.\w*', 'tokens');
    evtfile = util_getfile3(eventpath, '.mat', 'include', file_identifier{1}, 'level', 1);
    
    if length(evtfile) > 1
        error(['Cannot find unique event file in: ' eventpath]);
    elseif isempty(evtfile)
        error(['Cannot find any event file in: ' eventpath]);
    end
    
    cevents = load(evtfile{1});
    
    eventstr = cevents.event;
end

function evtout = cnbibochum_util_event_merge(srcstr, dststr)

    POS = [srcstr.POS; dststr.POS];
    TYP = [srcstr.TYP; dststr.TYP];
    DUR = [srcstr.DUR; dststr.DUR];
    TIM = [srcstr.TIM; dststr.TIM];
    LBL = [srcstr.LBL; dststr.LBL];
    ERR = [srcstr.ERR; dststr.ERR];
    
    [uPOS, uIdx] = sort(POS);
    uTYP = TYP(uIdx);
    uDUR = DUR(uIdx);
    uTIM = TIM(uIdx);
    uLBL = LBL(uIdx);
    uERR = ERR(uIdx);
    
    evtout.POS = uPOS;
    evtout.TYP = uTYP;
    evtout.DUR = uDUR;
    evtout.TIM = uTIM;
    evtout.LBL = uLBL;
    evtout.ERR = uERR;
end

function [cdate, dId] = getdate(cfilename, pdate, cdId)
   
    cdate = cell2mat(regexp(cfilename, '\.(\d*)\.', 'tokens', 'once'));
    
    if(strcmp(cdate, pdate) == false)
        dId = cdId + 1;
    else
        dId = cdId;
    end
end
