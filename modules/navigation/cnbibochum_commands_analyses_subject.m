clearvars; clc;

subject = 'BOCH04';

includepat  = {subject, 'mi', 'mi_bhbf', 'control'};
excludepat  = {};
depthlevel  = 2;

experiment  = 'mi_wheelchair';
datapath    = ['/mnt/data/Research/mi_wheelchair_bochum/' subject '_' experiment '/'];

%% Get datafiles
files = util_getfile3(datapath, '.gdf', 'include', includepat, 'exclude', excludepat, 'level', depthlevel);
NumFiles = length(files);
if(NumFiles > 0)
    util_bdisp(['[io] - Found ' num2str(NumFiles) ' files with the inclusion/exclusion criteria: (' strjoin(includepat, ', ') ') / (' strjoin(excludepat, ', ') '), depth: ' num2str(depthlevel)]);
else
    error(['[io] - No files found with the inclusion/exclusion criteria: (' strjoin(includepat, ', ') ') / (' strjoin(excludepat, ', ') '), depth: ' num2str(depthlevel)]);
end

events = [26113 26117 26118];

interevt = cell(NumFiles,1);
nocmdtime  = nan(NumFiles, 1);
totcmdtime = nan(NumFiles, 1);
cmdduration = 5;

for fId = 1:NumFiles
    [path, name, ext] = fileparts(files{fId});
    
    util_disp(['[io]   + Loading file ' num2str(fId) '/' num2str(NumFiles) ' from ' path ':'], 'b'); 


    [~, h] = sload(files{fId});


    nevents = length(h.EVENT.TYP);
    index = false(nevents, 1);

    for eId = 1:length(events)
        index = index | h.EVENT.TYP == events(eId);
    end
    
    interevt{fId} = diff(h.EVENT.POS(index));

    overduration = interevt{fId}/h.EVENT.SampleRate - cmdduration;

    nocmdtime(fId) = sum(overduration(overduration > 0));
    totcmdtime(fId) = (h.EVENT.POS(end) - h.EVENT.POS(1))/h.EVENT.SampleRate;


end