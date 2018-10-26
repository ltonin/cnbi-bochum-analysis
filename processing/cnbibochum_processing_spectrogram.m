clearvars; clc;

% subject = 'BOCH02';
subject = 'aj1';

pattern = '*.mi.*.gdf';

% experiment  = 'mi_wheelchair';
% datapath    = ['/mnt/data/Research/analysis_wheelchair_bochum/' subject '_' experiment '/'];

experiment  = 'micontinuous';
datapath    = ['/mnt/data/Research/micontinuous/' subject '_' experiment '/'];

spatialfilter = 'laplacian';
savedir       = ['analysis/psd/' spatialfilter '/'];
recompute     = false;

%% Processing parameters
mlength    = 1;
wlength    = 0.5;
pshift     = 0.25;                  
wshift     = 0.0625;                
selfreqs   = 4:2:96;
selchans   = 1:32;                  
% load('extra/laplacian32.mat');
% chanlabels = {'Fz', 'FC5', 'FC1', 'FC2', 'FC6', 'C3', 'Cz', 'C4',  ...
% 			  'CP5', 'CP1', 'CP2', 'CP6', 'P3', 'Pz', 'P4', 'POz', ...
% 			  'EOG', 'F1', 'F2', 'FC3', 'FCz', 'FC4', 'C5', 'C1',  ...
% 			  'C2', 'C6', 'CP3', 'CP4', 'P5', 'P1', 'P2', 'P6'};
selchans   = 1:16;
load('extra/laplacian16.mat');
chanlabels = {'Fz', 'FC3', 'FC1', 'FCz', 'FC2', 'FC4', ...
              'C3', 'C1', 'Cz', 'C2', 'C4', ...
              'CP3', 'CP1', 'CPz', 'CP2', 'CP4'};

winconv = 'backward'; 

%% Get datafiles
files = cnbibochum_util_getdata(datapath, pattern);
NumFiles = length(files);

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
    catch
        warning(['[warning] - Error loading filename ' cfilename '. Skipping it.']);
        continue;
    end
    s = s(:, selchans);
    
    
    % Computed DC removal
    util_bdisp('[proc] + Processing the data');
    disp('       |-DC removal');
    s_dc = s-repmat(mean(s),size(s,1),1);
    
    % Compute Spatial filter
    disp(['       |-Spatial filter: ' spatialfilter]);
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
    
    % Protocol
    try
        if ( isempty(cinfo.extra{2}) == false)
            protocol = cinfo.extra{2};
        else
            protocol = 'none';
        end
    catch
        protocol = 'none';
    end
    
    % Date
    date = cinfo.date;
    
    % Get classifier from log file
    classifier = [];
    if strcmpi(modality, 'offline') == false
        clogfile = [datapath '/'  cinfo.date '/' cinfo.subject '.' cinfo.date '.log'];
        
        [~, cfile, cext] = fileparts(cfilename);
        ctarget = char(regexp(cfile, '(\w*)\.(\d*)\.(\d*)', 'match'));
        
        clogstr = cnbibochum_read_logfile(clogfile, ctarget);
        
        try
            canalysis = load([datapath '/' cinfo.date '/' clogstr.classifier]);
        catch 
            error('chk:classifier', ['Classifier ''' clogstr.classifier ''' not found in ' datapath '/' cinfo.date '/']);
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
    settings.data.lchannels         = chanlabels;
    settings.data.samplerate        = h.SampleRate;
    settings.spatial.laplacian      = lap;
    settings.spatial.filter         = spatialfilter;
    settings.spectrogram.wlength    = wlength;
    settings.spectrogram.wshift     = wshift;
    settings.spectrogram.pshift     = pshift;
    settings.spectrogram.freqgrid   = freqs;
    settings.modality.legend        = {'offline','online'};
    settings.modality.name          = modality;
    settings.protocol.legend        = {'none', 'positive', 'full'};
    settings.protocol.name          = protocol;
    settings.date                   = date;
    
    [~, name] = fileparts(cfilename);
    sfilename = [savedir '/' name '.mat'];
    util_bdisp(['[out] - Saving psd in: ' sfilename]);
    save(sfilename, 'psd', 'freqs', 'events', 'settings', 'classifier'); 
end