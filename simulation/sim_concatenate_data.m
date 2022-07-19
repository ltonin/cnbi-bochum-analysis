function [data, events, labels, settings] = sim_concatenate_data(files)

    %warning('off', 'backtrace');
    numfiles = length(files);

    datalength = get_data_length(files);
    nsamples   = sum(datalength);

    cmdvel = nan(nsamples, 2);
    posvel = nan(nsamples, 2);
    pose   = nan(nsamples, 3);
    t      = nan(nsamples, 1);
    
    Ck = zeros(nsamples, 1);
    Rk = zeros(nsamples, 1);
    TYP = []; POS = []; DUR = []; TRK = [];
    samplerate = [];

    fileseek = 1;
    for fId = 1:numfiles

        cfile = files{fId};
        util_disp_progress(fId, numfiles, '        ');
        cdata = load(cfile, 't', 'events', 'cmdvel', 'posvel', 'pose', 'SampleRate');

        % Get current position 
        cstart   = fileseek;
        cstop    = cstart + datalength(fId) - 1;

        % Get current control modality from name
        if(contains(cfile, 'sharedcontrol.enabled'))
            ccontrol = 1;
        else
            ccontrol = 2;
        end

        % Concatenate data
        cmdvel(cstart:cstop, :) = cdata.cmdvel;
        posvel(cstart:cstop, :) = cdata.posvel;
        pose(cstart:cstop, :)   = cdata.pose;
        t(cstart:cstop)    = cdata.t;

        % Concatenate labels
        Ck(cstart:cstop) = ccontrol;
        Rk(cstart:cstop) = fId;

        % Concatenate events
        TYP = cat(1, TYP, cdata.events.TYP);
        DUR = cat(1, DUR, cdata.events.DUR);
        POS = cat(1, POS, cdata.events.POS + fileseek -1);
        TRK = cat(1, TRK, fId*ones(length(cdata.events.TYP), 1));

        % Samplerate
        if(isempty(samplerate))
            samplerate = cdata.SampleRate;
        end

        if(isequal(samplerate, cdata.SampleRate) == false)
            warning('chk:smp', 'Warning: differente samplerate between runs');
        end

        % Update the fileseek position
        fileseek = cstop + 1;
    end

    data.cmdvel = cmdvel;
    data.posvel = posvel;
    data.pose   = pose;
    data.t      = t;
    labels.Ck   = Ck;
    labels.Rk   = Rk;
    events.TYP = TYP;
    events.POS = POS;
    events.DUR = DUR;
    events.TRK = TRK;
    settings.samplerate = samplerate;
end

function dlength = get_data_length(filepaths)

    nfiles = length(filepaths);
  
    
    dlength = zeros(nfiles, 1);
    
    for fId = 1:nfiles
        cfilepath = filepaths{fId};
        cinfo = whos('-file', cfilepath, 't');
        
        dlength(fId) = cinfo.size(2);    
    end

end