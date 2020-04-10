clearvars; clc;

subject = 'BOCH05';

includepat  = {subject, 'mi', 'mi_bhbf'};
excludepat  = {'ciao'};
spatialfilter = 'laplacian';
artifactrej   = 'none'; % {'FORCe', 'none'}
datapath    = ['analysis/' artifactrej '/psd/' spatialfilter '/'];
files = util_getfile3(datapath, '.mat', 'include', includepat, 'exclude', excludepat);
nfiles = length(files);
util_bdisp(['[io] - Found ' num2str(nfiles) ' files with the inclusion/exclusion criteria: (' strjoin(includepat, ', ') ') / (' strjoin(excludepat, ', ') ')']);


%% Concatenate data
util_bdisp(['[io] - Importing ' num2str(nfiles) ' files from ' datapath ':']);
[~, events, labels, classifiers, settings] = cnbibochum_concatenate_labels(files);

Runs    = unique(labels.samples.Rk);
NumRuns = length(Runs);
Modalities = unique(labels.samples.Mk);
NumModalities = length(Modalities);
Days = unique(labels.samples.Dk);
NumDays = length(Days);
Protocols = unique(labels.samples.Pk);
NumProtocols = length(Protocols);
Feedbacks = unique(labels.samples.Fk);
NumFeedbacks = length(Feedbacks);

Rk = nan(NumRuns, 1);
Mk = nan(NumRuns, 1);
Pk = nan(NumRuns, 1);
Fk = nan(NumRuns, 1);

Phase = nan(NumRuns, 1);

PhaseLb = {'bci-calibration', 'bci-training-positive', 'bci-training-full', 'navigation-training', 'navigation-free'}; 

for rId = 1:NumRuns
    
    cindex = labels.samples.Rk == Runs(rId);
    
    Rk(rId) = unique(labels.samples.Rk(cindex));
    Mk(rId) = unique(labels.samples.Mk(cindex));
    Pk(rId) = unique(labels.samples.Pk(cindex));
    Fk(rId) = unique(labels.samples.Fk(cindex));
    
    
    Phase(rId) = Pk(rId);
    if(Mk(rId) == Modalities(ismember(settings.modality.legend, 'offline')))
        Phase(rId) = find(ismember(PhaseLb, 'bci-calibration'));
    elseif (Mk(rId) == Modalities(ismember(settings.modality.legend, 'online'))) && (Fk(rId) == Feedbacks(ismember(settings.feedback.legend, 'positive'))) && (Pk(rId) == Protocols(ismember(settings.protocol.legend, 'bci-training')))
        Phase(rId) = find(ismember(PhaseLb, 'bci-training-positive'));
    elseif (Mk(rId) == Modalities(ismember(settings.modality.legend, 'online'))) && (Fk(rId) == Feedbacks(ismember(settings.feedback.legend, 'full'))) && (Pk(rId) == Protocols(ismember(settings.protocol.legend, 'bci-training')))
        Phase(rId) = find(ismember(PhaseLb, 'bci-training-full'));
    elseif (Mk(rId) == Modalities(ismember(settings.modality.legend, 'online'))) && (Fk(rId) == Feedbacks(ismember(settings.feedback.legend, 'full'))) && (Pk(rId) == Protocols(ismember(settings.protocol.legend, 'wheelchair-training')))
        Phase(rId) = find(ismember(PhaseLb, 'navigation-training'));
    elseif (Mk(rId) == Modalities(ismember(settings.modality.legend, 'online'))) && (Fk(rId) == Feedbacks(ismember(settings.feedback.legend, 'full'))) && (Pk(rId) == Protocols(ismember(settings.protocol.legend, 'wheelchair-control')))
        Phase(rId) = find(ismember(PhaseLb, 'navigation-free'));
    else
        error('error');
    end
end


