function [F, events, labels, classifiers, settings] = cnbibochum_concatenate_data(files)

    warning('off', 'backtrace');
    numfiles = length(files);
    
    % Getting size info to allocate memory and speedup the concatenation
    datasize   = get_data_size(files);
    NumSamples = sum(datasize(1, :));
    NumFreqs   = unique(datasize(2, :));
    NumChans   = unique(datasize(3, :));

    F  = nan(NumSamples, NumFreqs, NumChans);
    Rk = nan(NumSamples, 1);
    Mk = nan(NumSamples, 1);
    Dk = nan(NumSamples, 1);
    Wk = nan(NumSamples, 1);
    Nk = nan(NumSamples, 1);
    Dl = [];
    
    classifiers = [];
    settings = [];
    TYP = []; POS = []; DUR = [];
    runId = 1;
    currday  = 0;
    cweekId  = 0;
    cmonthId = 0;
    lastday  = [];
    lweek    = [];
    lmonth   = [];
    
    fileseek = 1;
    for fId = 1:numfiles
    
        cfile = files{fId};
        util_disp_progress(fId, numfiles, '        ');
        cdata = load(cfile, 'psd', 'events', 'settings', 'classifier');

        % Get current position 
        cstart   = fileseek;
        cstop    = cstart + datasize(1, fId) - 1;
        
        % Get run modality
        cmodality_name = cdata.settings.modality.name;
        switch(cmodality_name)
            case 'offline'
                cmodality = 0;
            case 'online'
                cmodality = 1;
            otherwise
                error(['Unknown modality: ' cmodality_name]);
        end
        
        % Get run protocol
        cprotocol_name = cdata.settings.protocol.name;
        switch(cprotocol_name)
            case 'positive'     
                cprotocol = 1;
            case 'online'       % full feedback
                cprotocol = 2;
            case ''
                cprotocol = 1;
                warning('Empty protocol');
            case 'none'
                cprotocol = 1;
                warning('Empty protocol');
            otherwise
                keyboard
                error(['Unknown protocol: ' cprotocol_name]);
        end

        % Get day id and label
        if strcmpi(cdata.settings.date, lastday) == false
            currday = currday + 1;
            Dl = cat(1, Dl, cdata.settings.date);
            lastday = cdata.settings.date;
        end

        % Get week id
        cweek = week(datetime(cdata.settings.date, 'InputFormat', 'yyyyMMdd'));
        if isequal(cweek, lweek) == false
            cweekId = cweekId +1;
            lweek = cweek;
        end


        % Get month id
        cmonth = month(datetime(cdata.settings.date, 'InputFormat', 'yyyyMMdd'));
        if isequal(cmonth, lmonth) == false
            cmonthId = cmonthId +1;
            lmonth = cmonth;
        end


        Mk(cstart:cstop) = cmodality;
        Pk(cstart:cstop) = cprotocol;
        Rk(cstart:cstop) = runId;
        Dk(cstart:cstop) = currday;
        Wk(cstart:cstop) = cweekId;
        Nk(cstart:cstop) = cmonthId;


        % Concatenate events
        TYP = cat(1, TYP, cdata.events.TYP);
        DUR = cat(1, DUR, cdata.events.DUR);
        POS = cat(1, POS, cdata.events.POS + fileseek -1);

        % Getting psd
        F(cstart:cstop, :, :) = cdata.psd;
        
        % Save the current settings, remove number of sample and filename,
        % and compare it with the previous settings. If different, it
        % raises an error.
        csettings = cdata.settings;
        csettings.data.nsamples = nan;
        csettings.data.filename = nan;
        csettings.date = nan;
        csettings.protocol = nan;
        csettings.modality.name = nan;
        
        if(isempty(settings))
            settings = csettings;
        end
        
        if (isequaln(csettings, settings) == false)
            error('chk:stn', ['Different processing settings for file: ' cfile]);
        end

        % Getting classifier
        classifiers = cat(1, classifiers, cdata.classifier);

        % Update runId
        runId = runId + 1;
        
        % Update the fileseek position
        fileseek = cstop + 1;
    end
    
    events.TYP = TYP;
    events.POS = POS;
    events.DUR = DUR;
    
    labels.Mk = Mk;
    labels.Pk = Pk;
    labels.Rk = Rk;
    labels.Dk = Dk;
    labels.Wk = Wk;
    labels.Nk = Nk;
   
    warning('on', 'backtrace');
end

function dsizes = get_data_size(filepaths)

    nfiles = length(filepaths);
    ndimensions = 3;                            % samples x freqs x chans
    
    dsizes = zeros(ndimensions, nfiles);
    
    for fId = 1:nfiles
        cfilepath = filepaths{fId};
        cinfo = whos('-file', cfilepath, 'psd');
        dsizes(:, fId) = cinfo.size;    
    end

end



