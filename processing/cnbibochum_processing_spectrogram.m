clearvars; clc;

subject = 'BOCH02';

includepat  = {subject, 'mi', 'mi_bhbf'};
excludepat  = {};
depthlevel  = 2;

experiment  = 'mi_wheelchair';
datapath    = ['/mnt/data/Research/mi_wheelchair_bochum/' subject '_' experiment '/'];

artifactrej       = 'none'; % {'FORCe', 'none'}
ForceWinLength    = 1.0;
chanlocs32        = 'antneuro32.mat';
spatialfilter     = 'laplacian';
savedir           = ['analysis/' artifactrej '/psd/' spatialfilter '/'];
recompute         = true;


%% Processing parameters
mlength    = 1;
wlength    = 0.5;
pshift     = 0.25;                  
wshift     = 0.0625;                
selfreqs   = 4:2:96;
winconv = 'backward'; 

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

%% Processing files
for fId = 1:NumFiles
    cfilename = files{fId};
    
    util_bdisp(['[io] + Loading file ' num2str(fId) '/' num2str(NumFiles)]);
    disp(['     |-File: ' cfilename]);
    
    % Check if the file has been already processed
    [~, pfilename] = fileparts(cfilename);
    if (recompute == false) && exist([savedir pfilename '.mat'], 'file') == 2
        disp('     |-Processed PSD already exists. Skipping the recomputing');
        continue;
    end
    
    % Get information from filename
    cinfo = util_getfile_info(cfilename);
    
    % Loading data
    disp('     |-Loading GDF data');
    try
        [s, h] = sload(cfilename);
    catch ME
        warning('[warning] - Cannot load filename. Skipping it.');
        warning(['[warning] - Error: ' ME.message]);
        continue;
    end
    
    % Processing data
    util_bdisp('[proc] + Processing the data');
    
    % Extract channels
    switch(size(s, 2))
        case 33
            layout = 'eeg.antneuro.32.mi';
        case 65
            layout = 'eeg.antneuro.64.mi';
        otherwise
            error(['Unknown layout with this number of channels: ' num2str(size(s, 2))]);
    end
    [montage, labels] = proc_get_montage(layout);
    
    if isequal(h.Label(1:end-1), upper(labels)') == false
        warning('Different labels in the data file and in the provided layout');
    end
    s = s(:, 1:end-1);
    disp(['       |-Number of channels: ' num2str(size(s, 2))]);
    disp(['       |-Layout: ' layout]);
    
    % Artifact removal
    disp(['       |-Artifact removal: ' artifactrej]);
    switch(artifactrej) 
        case 'FORCe'
            s_artrem = cnbibochum_artifact_force(s, ForceWinLength, h.SampleRate, chanlocs32);
        case 'none'
            s_artrem = s;
        otherwise
            error('Unknown artifact rejection method');
    end
    
    % Computed DC removal
    disp('       |-DC removal');
    s_dc = s_artrem-repmat(mean(s_artrem),size(s_artrem,1),1);
    
    % Compute Spatial filter
    disp(['       |-Spatial filter: ' spatialfilter]);
    lap = proc_laplacian_mask(montage, 1, 'cross');
    switch(spatialfilter)
        case 'none'
            s_filt = s_dc;
        case 'car'
            s_filt = proc_car(s_dc);
        case 'laplacian'
            s_filt = s_dc*lap;
        otherwise
            error(['Unknown spatial filter selected ' spatialfilter]);
    end
    
    
    % Compute spectrogram
    disp('       |-Spectrogram');
    [psd, freqgrid] = proc_spectrogram(s_filt, wlength, wshift, pshift, h.SampleRate, mlength);
    
    % Selecting desired frequencies
    [freqs, idfreqs] = intersect(freqgrid, selfreqs);
    psd = psd(:, idfreqs, :);
    
    % Extracting events
    disp('       |-Extract events');
    cevents     = h.EVENT;
    events.TYP = cevents.TYP;
    events.POS = proc_pos2win(cevents.POS, wshift*h.SampleRate, winconv, mlength*h.SampleRate);
    events.DUR = floor(cevents.DUR/(wshift*h.SampleRate)) + 1;
    events.conversion = winconv;
    
    % Modality
    disp('       |-Extract additional info (modality, protocol, date)');
    modality = cinfo.modality;
    
    % Feedback
    switch(cinfo.extra{2})
        case 'offline'
            feedback = 'fake';
        case 'positive'
            feedback = 'positive';
        case 'online'
            feedback = 'full';
        case 'guided'
            feedback = 'full';
        case 'control'
            feedback = 'full';
        otherwise
            feedback = 'unknown';
    end
    
    % Protocol
    switch(cinfo.extra{2})
        case 'offline'
            protocol = 'bci-calibration';
        case 'positive'
            protocol = 'bci-training';
        case 'online'
            protocol = 'bci-training';
        case 'guided'
            protocol = 'wheelchair-training';
        case 'control'
            protocol = 'wheelchair-control';
        otherwise
            protocol = 'unknown';
    end
    
    % Date
    date = cinfo.date;
    
    % Get classifier from log file
    classifier.filename    = '';
    classifier.gau         = [];
    classifier.features    = [];
    classifier.rejection   = [];
    classifier.integration = [];
    classifier.thresholds  = [];
    classifier.classes     = [];
    if strcmpi(modality, 'offline') == false
        clogfile = [datapath '/'  cinfo.date '/' cinfo.subject '.' cinfo.date '.log'];
        
        [~, cfile, cext] = fileparts(cfilename);
        ctarget = char(regexp(cfile, '(\w*)\.(\d*)\.(\d*)', 'match'));
        
        clogstr = cnbibochum_read_logfile(clogfile, ctarget);
        
        try
            canalysis = load([datapath '/classifiers/' clogstr.classifier]);
        catch 
            error('chk:classifier', ['Classifier ''' clogstr.classifier ''' not found in ' datapath '/classifiers/']);
        end
        
        classifier.filename    = clogstr.classifier;
        classifier.gau         = canalysis.analysis.tools.net.gau;
        classifier.features    = canalysis.analysis.tools.features;
        classifier.rejection   = clogstr.rejection;
        classifier.integration = clogstr.integration;
        classifier.thresholds  = clogstr.thresholds;
        classifier.classes     = canalysis.analysis.settings.task.classes_old;
        
        disp(['       |-Imported classifier belonging to this file: ' clogstr.classifier]);
    end
    
    % Create settings structure
    settings.data.filename          = cfilename;
    settings.data.nsamples          = size(s, 1);
    settings.data.nchannels         = size(s, 2);
    settings.data.lchannels         = labels;
    settings.data.samplerate        = h.SampleRate;
    settings.artifact.name          = artifactrej;
    settings.artifact.force.wlength = ForceWinLength;
    settings.spatial.laplacian      = lap;
    settings.spatial.filter         = spatialfilter;
    settings.spectrogram.wlength    = wlength;
    settings.spectrogram.wshift     = wshift;
    settings.spectrogram.pshift     = pshift;
    settings.spectrogram.freqgrid   = freqs;
    settings.modality.legend        = {'offline','online'};
    settings.modality.name          = modality;
    settings.feedback.legend        = {'fake', 'positive', 'full', 'unknown'};
    settings.feedback.name          = feedback;
    settings.protocol.legend        = {'bci-calibration', 'bci-training', 'wheelchair-training', 'wheelchair-control', 'unknown'};
    settings.protocol.name          = protocol;
    settings.date                   = date;
    
    sfilename = [savedir '/' pfilename '.mat'];
    util_bdisp(['[out] - Saving psd in: ' sfilename]);
    save(sfilename, 'psd', 'freqs', 'events', 'settings', 'classifier'); 
end
