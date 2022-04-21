clearvars; clc;

subject = 'BOCH02';

includepat  = {subject, 'mi', 'mi_bhbf', 'online'};
excludepat  = {'control', 'offline'};
depthlevel  = 2;

experiment  = 'mi_wheelchair';
datapath    = ['/mnt/data/Research/mi_wheelchair_bochum/' subject '_' experiment '/'];
savedir     = 'analysis/latency/';

modalities  = {'offline','online'};
feedbacks   = {'fake', 'positive', 'full', 'unknown'};
protocols   = {'bci-calibration', 'bci-training', 'wheelchair-training', 'wheelchair-control', 'unknown'};

%% Get datafiles
files = util_getfile3(datapath, '.gdf', 'include', includepat, 'exclude', excludepat, 'level', depthlevel);
NumFiles = length(files);
if(NumFiles > 0)
    util_bdisp(['[io] - Found ' num2str(NumFiles) ' files with the inclusion/exclusion criteria: (' strjoin(includepat, ', ') ') / (' strjoin(excludepat, ', ') '), depth: ' num2str(depthlevel)]);
else
    error(['[io] - No files found with the inclusion/exclusion criteria: (' strjoin(includepat, ', ') ') / (' strjoin(excludepat, ', ') '), depth: ' num2str(depthlevel)]);
end

%% Create/Check for savepath
util_mkdir(pwd, savedir);

%% Extracting trial duration
CFeedbackTyp = 781;
CorrectTyp   = 897;
WrongTyp     = 898;

lastdate = '';
dayId = 0;
weekId = 0;
monthId = 0;
TrialLatency = [];
Xk = [];
Rk = [];
Mk = [];
Fk = [];
Pk = [];
Dk = [];
Wk = [];
Nk = [];
for fId = 1:NumFiles

    cfilename = files{fId};
    
    util_bdisp(['[io] + Loading file ' num2str(fId) '/' num2str(NumFiles)]);
    disp(['     |-File: ' cfilename]);

    % Loading data
    disp('     |-Loading GDF data');
    try
        [~, h] = sload(cfilename);
    catch ME
        warning('[warning] - Cannot load filename. Skipping it.');
        warning(['[warning] - Error: ' ME.message]);
        continue;
    end

    % Extract run info
    util_disp('[proc] - Extracting information');
    info = util_getfile_info(cfilename);
    modality = get_modality_run(info.modality, modalities);
    feedback = get_feedback_run(info.extra{2}, feedbacks);
    protocol = get_protocol_run(info.extra{2}, protocols);
    dayId   = get_info_day(info.date, lastdate, dayId);
    weekId  = get_info_week(info.date, lastdate, weekId);
    monthId = get_info_month(info.date, lastdate, monthId);


    % Extract events and latency
    util_disp('[proc] - Extracting latency');
    events = h.EVENT;
    ntrials = sum(events.TYP == CFeedbackTyp);
    latency = events.DUR(events.TYP == CFeedbackTyp)./events.SampleRate;
    
    % Extract trial correctness
    util_disp('[proc] - Extracting trial results');
    results = events.TYP(events.TYP == CorrectTyp | events.TYP == WrongTyp);
    results(results == CorrectTyp) = true;
    results(results == WrongTyp) = false;
    if (isempty(results) == true)
        results = zeros(ntrials, 1);
    elseif (length(results) ~= ntrials)
        if(strcmpi(protocols{protocol}, 'wheelchair-control') == true)
            results = nan(ntrials, 1);
        else
            keyboard;
        end
    end


    util_disp('[proc] - Concatenate data');
    TrialLatency = cat(1, TrialLatency, latency);
    Xk = cat(1, Xk, results);
    Rk = cat(1, Rk, fId*ones(ntrials, 1));
    Mk = cat(1, Mk, modality*ones(ntrials, 1));
    Fk = cat(1, Fk, feedback*ones(ntrials, 1));
    Pk = cat(1, Pk, protocol*ones(ntrials, 1));
    Dk = cat(1, Dk, dayId*ones(ntrials, 1));
    Wk = cat(1, Wk, weekId*ones(ntrials, 1));
    Nk = cat(1, Nk, monthId*ones(ntrials, 1));

    lastdate = info.date;

end


latency = TrialLatency;
labels.trial.Xk = Xk;
labels.trial.Rk = Rk;
labels.trial.Mk = Mk;
labels.trial.Fk = Fk;
labels.trial.Pk = Pk;
labels.trial.Dk = Dk;
labels.trial.Wk = Wk;
labels.trial.Nk = Nk;

sfilename = [savedir '/' subject '_latency.mat'];
util_disp(['[out] - Saving latencies in: ' sfilename], 'b');
save(sfilename, 'latency', 'labels'); 

%% Functions
function modality = get_modality_run(name, legend)

    if(strcmpi(name, 'unknown'))
        error(['Unknown modality type: ' name]);
    else
        [~, modality] = intersect(legend, name);
    end
end

function feedback = get_feedback_run(iname, legend)
    
    switch(iname)
        case 'offline'
            name = 'fake';
        case 'positive'
            name = 'positive';
        case 'online'
            name = 'full';
        case 'guided'
            name = 'full';
        case 'control'
            name = 'full';
        otherwise
            name = 'unknown';
    end

    % Get run feebdack
    if(strcmpi(name, 'unknown'))
        error(['Unknown feedback type: ' name]);
    else
        [~, feedback] = intersect(legend, name);
    end
end

function protocol = get_protocol_run(iname, legend)

    switch(iname)
        case 'offline'
            name = 'bci-calibration';
        case 'positive'
            name = 'bci-training';
        case 'online'
            name = 'bci-training';
        case 'guided'
            name = 'wheelchair-training';
        case 'control'
            name = 'wheelchair-control';
        otherwise
            name = 'unknown';
    end

    % Get run protocol
    if(strcmpi(name, 'unknown'))
        error(['Unknown protocol type: ' name]);
    else
        [~, protocol] = intersect(legend, name);
    end

end

function dayId = get_info_day(currdate, lastdate, dayId)
    % Get day id
    cday = day(datetime(currdate, 'InputFormat', 'yyyyMMdd'));
    lday = day(datetime(lastdate, 'InputFormat', 'yyyyMMdd'));
    if isequal(cday, lday) == false
        dayId = dayId +1;
    end
end

function weekId = get_info_week(currdate, lastdate, weekId)
    
    % Get week id
    cweek = week(datetime(currdate, 'InputFormat', 'yyyyMMdd'));
    lweek = week(datetime(lastdate, 'InputFormat', 'yyyyMMdd'));
    if isequal(cweek, lweek) == false
        weekId = weekId +1;
    end
end

function monthId = get_info_month(currdate, lastdate, monthId)
    
    % Get month id
    cmonth = month(datetime(currdate, 'InputFormat', 'yyyyMMdd'));
    lmonth = month(datetime(lastdate, 'InputFormat', 'yyyyMMdd'));
    if isequal(cmonth, lmonth) == false
        monthId = monthId +1;
    end

end